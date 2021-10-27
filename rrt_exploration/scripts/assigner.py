#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import tf
from rrt_exploration.msg import PointArray, invalidArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount,calcDynamicTimeThreshold, calculateLocationDistance
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
invalidFrontier=[]
globalmaps=[]
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,globalmaps,invalidFrontier
	rospy.init_node('assigner', anonymous=False)

	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')
	inv_frontier_topic= rospy.get_param('~invalid_frontier','/invalid_points')
	time_per_meter  = rospy.get_param('~time_per_meter',12)
	message_time_interval  = rospy.get_param('~message_time_interval',1.0)
	invalid_distance  = rospy.get_param('~invalid_distance',0.5)
	robot_namelist = rospy.get_param('~robot_namelist', "robot1")
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	# services for the robot
	global_frame = rospy.get_param('~global_frame', 'map')
	plan_service = rospy.get_param('~plan_service', 'move_base_node/NavfnROS/make_plan')
	base_link = rospy.get_param('~base_link', 'base_link')
	move_base_service = rospy.get_param('~move_base_service', '/move_base')

	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------
	robot_namelist = robot_namelist.split(',')
	# print(robot_namelist)
#---------------------------------------------------------------------------------------------------------------
	start_time = rospy.get_rostime().secs
	robot_assigned_goal = []

# wait if no frontier is received yet
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[]
	if len(robot_namelist) > 0:
		for i in range(0,len(robot_namelist)):
			rospy.loginfo('working on initializing robot  :' + robot_namelist[i])
			robots.append(robot(robot_namelist[i],move_base_service=move_base_service,plan_service=plan_service,global_frame=global_frame,base_link=base_link))

	for i in range(0,len(robot_namelist)):
		rospy.loginfo('setting initial position for robot: %s' %(robot_namelist[i]))
		robots[i].sendGoal(robots[i].getPosition())
		robot_assigned_goal.append({'robot_id':i, 'goal':robots[i].getPosition(), 'startLoc':robots[i].getPosition(),
		 'time_start':start_time, 'valid':True, 'time_thres':-1, 'lastgoal':robots[i].getPosition()})


	invCenPub = rospy.Publisher('invalid_centroids', Marker, queue_size=10)
	invPub    = rospy.Publisher('invalid_points', invalidArray, queue_size=10)

	# -------------------- define the basic information for the invalid centroid to publish
	points = Marker()
	points.header.frame_id = mapData.header.frame_id
	points.header.stamp = rospy.Time.now()
	points.ns = "invalidCentroid"
	points.id = 0
	points.type = Marker.POINTS
	# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points.action = Marker.ADD
	points.pose.orientation.w = 1.0
	points.scale.x = 0.2
	points.scale.y = 0.2
	points.color.r = 255.0/255.0
	points.color.g = 0.0/255.0
	points.color.b = 255.0/255.0
	points.color.a = 1
	points.lifetime = rospy.Duration()


#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	robot_goal_record = []
	temp_inv_array = []
	next_assign_time = rospy.get_rostime().secs
	next_time_interval = rospy.get_rostime().secs
	time_interval_due = False
	while not rospy.is_shutdown():
		centroids=copy(frontiers)
#-------------------------------------------------------------------------			
		if next_time_interval <= rospy.get_rostime().secs:
			time_interval_due = True
			next_time_interval = rospy.get_rostime().secs + message_time_interval
#-------------------------------------------------------------------------			
		invalid_goal_array = invalidArray()
		invalid_goal_array.points = []
		
		for iz in range(0,len(invalidFrontier)):
			tempPoint = Point()
			tempPoint.z = 0.0
			tempPoint.x = invalidFrontier[iz][0]
			tempPoint.y = invalidFrontier[iz][1]
			invalid_goal_array.points.append(copy(tempPoint))
#-------------------------------------------------------------------------
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0, len(robot_namelist)):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)
		if time_interval_due:
			rospy.loginfo("available robots: "+str(na))	
#-------------------------------------------------------------------------
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------
		revenue_record=[]
		centroid_record=[]
		id_record=[]

		for ir in na:
			for ip in range(0,len(centroids)):
				cost=norm(robots[ir].getPosition()-centroids[ip])
				threshold=1
				information_gain=infoGain[ip]
				if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):

					information_gain*=hysteresis_gain
				revenue=information_gain*info_multiplier-cost
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)

		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain

					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)

		# rospy.loginfo("available robots: "+str(na))	
		if time_interval_due:
			if len(na) >= 1:
				rospy.loginfo("revenue record: "+str(revenue_record))	
				rospy.loginfo("centroid record: "+str(centroid_record))	
				rospy.loginfo("robot IDs record: "+str(id_record))	
				for ih in range(0, len(robot_namelist)):
					currTime = rospy.get_rostime().secs
					timeRemaining = (robot_assigned_goal[ih]['time_start'] + robot_assigned_goal[ih]['time_thres']) - currTime
					rospy.loginfo("robot " + robot_namelist[ih] + " - last goal:" + str(robot_assigned_goal[ih]['lastgoal']))
			else:
				rospy.loginfo("- no idling robots detected, status update -")
				for ih in range(0, len(robot_namelist)):
					currTime = rospy.get_rostime().secs
					timeRemaining = (robot_assigned_goal[ih]['time_start'] + robot_assigned_goal[ih]['time_thres']) - currTime
					rospy.loginfo("robot " + robot_namelist[ih] + " working on goal:" + str(robot_assigned_goal[ih]['goal']) 
					+ " -remaining time: " + str(int(timeRemaining)) + "s")

#-------------------------------------------------------------------------
	goal_repeated   = False
	assigned_winner = -1	
	if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			# check if the goal is the same or not same
			cond_goal = True
			for xi in range(0, len(invalidFrontier)):
				if calculateLocationDistance(robot_assigned_goal[id_record[winner_id]]['goal'], invalidFrontier[xi]) < 0.01:
					cond_goal = False
					print(">> invalid goal detected. distance: %.3f" %(calculateLocationDistance(robot_assigned_goal[id_record[winner_id]]['goal'], invalidFrontier[xi])))
					break
			cond_history = True
			try:
				history = []
				for xj in range(0,len(robot_namelist)):
						history.extend(robot[xj].getGoalHistory())
				for xh in range(0, len(history)):
						if calculateLocationDistance(centroid_record[winner_id], history[xh]) < 0.01:
							remainingTime = ((robot_assigned_goal[xh]['time_start'] + robot_assigned_goal[xh]['time_thres']) - rospy.get_rostime().secs) 
							if remainingTime < 0.0:
								cond_history = False
								print('history repeat the same goal assignment')
								break
			except:
				pass
			## check forthe goal assignment condition.
			# also need to check whether the goal is repeated i nthe goal or not
			if rospy.get_rostime().secs >= next_assign_time:
				if cond_history and cond_goal:
# 					robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
					robots[id_record[winner_id]].sendGoalTransformed(centroid_record[winner_id])
					robots[id_record[winner_id]].setGoalHistory(centroid_record[winner_id])	
					robot_assigned_goal[id_record[winner_id]]['lastgoal']   = robot_assigned_goal[id_record[winner_id]]['goal'].copy()
					robot_assigned_goal[id_record[winner_id]]['goal']       = centroid_record[winner_id]
					robot_assigned_goal[id_record[winner_id]]['startLoc']   = robots[id_record[winner_id]].getPosition()
					robot_assigned_goal[id_record[winner_id]]['time_start'] = rospy.get_rostime().secs
					robot_assigned_goal[id_record[winner_id]]['time_thres'] = calcDynamicTimeThreshold(robots[id_record[winner_id]].getPosition(), centroid_record[winner_id], time_per_meter)
					rospy.loginfo(robot_namelist[id_record[winner_id]] + " has been assigned to  "+str(centroid_record[winner_id]) + 	
					" mission start at " + str(robot_assigned_goal[id_record[winner_id]]['time_start']) + " sec  - Limit: " + str(robot_assigned_goal[id_record[winner_id]]['time_thres']) + "s") 
					next_assign_time = rospy.get_rostime().secs + delay_after_assignement
			# if the goal is not invalid and repeated 
			if cond_history and not cond_goal:
				goal_repeated   = True
				assigned_winner = id_record[winner_id]
				faultyGoal      = centroid_record[winner_id]

#-------------------------------------------------------------------------				
				if goal_repeated:
			repeatInvalid = False
			for ie in range(0,len(invalidFrontier)):
				if calculateLocationDistance(invalidFrontier[ie], robot_assigned_goal[ix]['goal']) < 0.01:
					repeatInvalid = True
			if repeatInvalid == False:
				tempInvGoal = Point()
				tempInvGoal.x = robot_assigned_goal[ix]['goal'][0]				
				tempInvGoal.y = robot_assigned_goal[ix]['goal'][1]	
				tempInvGoal.z = 0.0	
				# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit 
				if distance <= invalid_distance:
					invalid_goal_array.points.append(copy(tempInvGoal))
					invalidFrontier.append(robot_assigned_goal[ix]['goal'])		
					temp_inv_array.append(robot_assigned_goal[ix]['goal'])
			print('Robot: %d assigned goal is repeated in the history' %(winner_id))
			robot_assigned_goal[assigned_winner]['lastgoal']   = faultyGoal
			robot_assigned_goal[assigned_winner]['goal']       = robots[assigned_winner].getPosition()
			robot_assigned_goal[assigned_winner]['time_start'] = rospy.get_rostime().secs
			robot_assigned_goal[assigned_winner]['startLoc']   = robots[assigned_winner].getPosition()
			robot_assigned_goal[assigned_winner]['time_thres'] = -1
			# cancel goal
			rospy.loginfo("!!!> Robot " + robot_namelist[ix] + " give up goal: " + str(robot_assigned_goal[ix]['goal'])
			+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance) + " -- goal repeated")
			robots[ix].cancelGoal()

		else:
			for ix in range(0,len(robot_namelist)):
				currTime = rospy.get_rostime().secs
				distance = calculateLocationDistance(robots[ix].getPosition(), robot_assigned_goal[ix]['goal'])
				if robot_assigned_goal[ix]['time_thres'] != -1:		
					if (((robot_assigned_goal[ix]['time_start'] + robot_assigned_goal[ix]['time_thres']) - currTime) <= 0):
						# for for repeated invalid frontier
						repeatInvalid = False
						for ie in range(0,len(invalidFrontier)):
							if calculateLocationDistance(invalidFrontier[ie], robot_assigned_goal[ix]['goal']) < 0.01:
								repeatInvalid = True
						if repeatInvalid == False:
							tempInvGoal = Point()
							tempInvGoal.x = robot_assigned_goal[ix]['goal'][0]				
							tempInvGoal.y = robot_assigned_goal[ix]['goal'][1]	
							tempInvGoal.z = 0.0	
							# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit 
							if distance <= invalid_distance:
								invalid_goal_array.points.append(copy(tempInvGoal))
								invalidFrontier.append(robot_assigned_goal[ix]['goal'])		
								temp_inv_array.append(robot_assigned_goal[ix]['goal'])

						# need to reset the assigned tasks to the robot
						robot_assigned_goal[ix]['lastgoal']   = robot_assigned_goal[ix]['goal'].copy()
						robot_assigned_goal[ix]['goal']       = robots[ix].getPosition()
						robot_assigned_goal[ix]['time_start'] = rospy.get_rostime().secs
						robot_assigned_goal[ix]['startLoc'] = robots[ix].getPosition()
						robot_assigned_goal[ix]['time_thres'] = -1
						# cancel goal
						rospy.loginfo("!!!> Robot " + robot_namelist[ix] + " give up goal: " + str(robot_assigned_goal[ix]['goal'])
						+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance) + " -- goal repeated")
						robots[ix].cancelGoal()

#-------------------------------------------------------------------------					
		if time_interval_due:
			rospy.loginfo('publishing invalid goals :' + str(temp_inv_array))
			# print out the distance traveled by each of the robot
			for ii in range(0, len(robot_namelist)):
				print("Robot - %d   total distance traveled: %.2fm" %(ii, robots[ii].getDistanceTraveled()))

		# publish invalid location for the points
		invPub.publish(invalid_goal_array)
		points.points = copy(invalid_goal_array.points)
		invCenPub.publish(points)
		time_interval_due = False

#-------------------------------------------------------------------------
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
