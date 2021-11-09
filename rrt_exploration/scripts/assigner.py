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
import random
import time
from functions import robot,informationGain,discount2,calcDynamicTimeThreshold, calculateLocationDistance, relativePositionMetric, getMaxRevenueForRobot, getHigestIndex
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
	map_topic                = rospy.get_param('~map_topic','/map')
	info_radius              = rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier          = rospy.get_param('~info_multiplier',3.0)
	hysteresis_radius        = rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain          = rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic          = rospy.get_param('~frontiers_topic','/filtered_points')
	inv_points_topic         = rospy.get_param('~invalid_frontier','/invalid_points')	
	inv_frontier_topic       = rospy.get_param('~invalid_centroids','/invalid_centroids')	
	time_per_meter           = rospy.get_param('~time_per_meter',6.0)
	message_time_interval    = rospy.get_param('~message_time_interval',2.0)
	invalid_distance         = rospy.get_param('~invalid_distance',0.5)
	robot_namelist           = rospy.get_param('~robot_namelist', "robot1")
	delay_after_assignement  = rospy.get_param('~delay_after_assignement',1.0)
	invalid_distance         = rospy.get_param('~invalid_distance',0.5)
	rp_metric_distance       = rospy.get_param('~rp_metric_distance',10.0)
	start_delay              = rospy.get_param('~start_delay',10.0)
	rateHz                   = rospy.get_param('~rate',100)
	# services for the robot
	global_frame      = rospy.get_param('~global_frame', 'map')
	plan_service      = rospy.get_param('~plan_service', 'move_base_node/NavfnROS/make_plan')
	base_link         = rospy.get_param('~base_link', 'base_link')
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
		# robots[i].sendGoal(robots[i].getPosition())
		robots[i].sendGoalTransformed(robots[i].getPosition())
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
	points.scale.x = 0.6
	points.scale.y = 0.6
	points.color.r = 255.0/255.0
	points.color.g = 0.0/255.0
	points.color.b = 255.0/255.0
	points.color.a = 0.8
	points.lifetime = rospy.Duration()

# ------------------------------------------------------------------------
# delay for initializing the robot
	rospy.sleep(start_delay)

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	robot_goal_record = []
	temp_inv_array = []
	next_assign_time = rospy.get_rostime().secs
	next_time_interval = rospy.get_rostime().secs
	time_interval_due = False
	centroid_null_count = 0
	lastTrackTime = rospy.get_rostime().secs
	robot_goal_cancel = []
	while not rospy.is_shutdown():
		centroids=copy(frontiers)
#-------------------------------------------------------------------------
		if (lastTrackTime + 1.0) > rospy.get_rostime().secs:
			if len(centroids) == 0:
				centroid_null_count += 1
			else:
				centroid_null_count = 0
			lastTrackTime = rospy.get_rostime().secs
		if centroid_null_count >= 100:
			rospy.loginfo("No more goal detected - terminating at " + str(rospy.get_rostime().secs) + "s")
			break
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
# check for the status of the movebase
		# robot_goal_cancel = []
		for ii in range(0, len(robot_namelist)):
				movebase_status = robots[ii].get_movebase_status()
				# check the move_base status #4 is having difficulties.
				# need liase with brandon to make sure the nubmer of retry is limited and more prone to trigger this condition based on the move base condition.
				if movebase_status >= 4:
					currTime = rospy.get_rostime().secs
					rospy.loginfo("\n\n Robot " + str(ii) + " stucked at goal:" + str(robot_assigned_goal[ii]['goal']) +  " and cancel goal now.\n")

					# marks point as invalid
					distance = calculateLocationDistance(robots[ii].getPosition(), robot_assigned_goal[ii]['goal'])
					repeatInvalid = False
					for ie in range(0,len(invalidFrontier)):
						if calculateLocationDistance(invalidFrontier[ie], robot_assigned_goal[ii]['goal']) < 0.01:
							repeatInvalid = True
					if repeatInvalid == False:
						tempInvGoal   = Point()
						tempInvGoal.x = robot_assigned_goal[ii]['goal'][0]
						tempInvGoal.y = robot_assigned_goal[ii]['goal'][1]
						tempInvGoal.z = 0.0
						invalid_goal_array.points.append(copy(tempInvGoal))
						invalidFrontier.append(robot_assigned_goal[ii]['goal'])
						temp_inv_array.append(robot_assigned_goal[ii]['goal'])
						# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit
					robot_assigned_goal[ii]['lastgoal']   = robot_assigned_goal[ii]['goal']
					robot_assigned_goal[ii]['goal']       = robots[ii].getPosition()
					robot_assigned_goal[ii]['time_start'] = rospy.get_rostime().secs
					robot_assigned_goal[ii]['startLoc']   = robots[ii].getPosition()
					robot_assigned_goal[ii]['time_thres'] = -1
					# cancel goal
					rospy.loginfo("\n!!!> Robot " + robot_namelist[ii] + " give up goal: " + str(robot_assigned_goal[ii]['goal'])
					+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance) + " -- robot path planning failed \n")
					robots[ii].cancelGoal()
					robot_goal_cancel.append(ii)

#-------------------------------------------------------------------------
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0, len(robot_namelist)):
				if ii in robot_goal_cancel:
					na.append(i)
				else:
					if (robots[i].getState()==1):
						nb.append(i)
					else:
						na.append(i)

		# remove the content of the goal cancel list and start from the scratch again
		robot_goal_cancel = []

		if time_interval_due:
			rospy.loginfo("available robots: "+str([robot_namelist[xyz] for xyz in na]))	

#-------------------------------------------------------------------------
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*info_multiplier)
#-------------------------------------------------------------------------
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount2(mapData,robot_assigned_goal[i]['lastgoal'],centroids,infoGain,info_radius)
#-------------------------------------------------------------------------
#-------------------------------------------------------------------------
		revenue_record=[]
		centroid_record=[]
		robots_position = []
		robots_goals    = []
				# initilization of the array
		for qq in range(0, len(robot_namelist)):
			revenue_record.append([])
			centroid_record.append([])
		# get robot position and goals
		for qq in range(0, len(robot_namelist)):
  			robots_position.append(robots[qq].getPosition())
  			robots_goals.append(robot_assigned_goal[qq]['goal'])
		for ir in na+nb:
			for ip in range(0,len(centroids)):
				cost=norm(robots_position[ir]-centroids[ip])
				# if cost <= (rp_metric_distance*1.5):
				information_gain=infoGain[ip]
				if ir in nb:
					if norm(centroids[ip]-robots_position[ir])<=hysteresis_radius:
						information_gain*=hysteresis_gain

					if norm(robots_goals[ir]-centroids[ip])<=hysteresis_radius:
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain
				else:
					if norm(centroids[ip]-robots_position[ir])<=hysteresis_radius:
						information_gain*=hysteresis_gain

				# if norm(centroids[ip]-robots_goals[ir])<hysteresis_radius:
				rp_metric = relativePositionMetric(centroids[ip], ir, robots_goals, rp_metric_distance)

				if information_gain >= 0:
					information_gain = information_gain * rp_metric
				else:
					information_gain = information_gain / rp_metric

				revenue=(information_gain)-cost
				revenue_record[ir].append(revenue)
				centroid_record[ir].append(centroids[ip])

#-------------------------------------------------------------------------
		if time_interval_due:
			if len(na) >= 1:
				rospy.loginfo("revenue record: "+str(revenue_record))
				rospy.loginfo("centroid record: "+str(centroid_record))
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
					+ " -remaining time: " + str(float(timeRemaining)) + "s")


#-------------------------------------------------------------------------
	
#-------------------------------------------------------------------------	
		if (len(centroids)>0):
			goal_repeated   = False
			robotListTemp = na + nb
			random.shuffle(robotListTemp)
			if rospy.get_rostime().secs >= next_assign_time:
				for xxx in robotListTemp:
					'''
					the idea here allocate retry function for each of the next highest revenue frontier.
					for Available robot, there will be no condition imposed to allocate the revenue when compared to the Busy robot.
					For busy robot, the condition imposed will be based on three metrics - revenue, distance, time.
					'''
					notAssign = True
					attempt  = 0
					start = time.time()
					# condition setting up here:
						# not assigned a goal
						# attempt will not be larger than the list of centroids
						# the robot is in available state	
					while notAssign and attempt < len(centroids) and xxx in na:
						winner_id = getHigestIndex(revenue_record[xxx], attempt)
						if winner_id != -1:
							# condition for the general robot	
							cond_goal       = True
							cond_history    = True
							cond_farAssign  = True
							for xi in range(0, len(invalidFrontier)):
								if calculateLocationDistance(centroid_record[xxx][winner_id], invalidFrontier[xi]) < 0.1:
									cond_goal = False
									print(">> invalid goal detected. distance: %.3f" %(calculateLocationDistance(centroid_record[xxx][winner_id], invalidFrontier[xi])))
									break	
							# check if history appear in other robot history
							history = []
							for xj in range(0,len(robot_namelist)):
								history.append(robots[xj].getGoalHistory())
							for xj in range(0, len(robot_namelist)):
								#if the goal is assigned to other robot
								if xxx != xj:
									for xh in range(0, len(history[xj])):
										distance_apart = calculateLocationDistance(centroid_record[xxx][winner_id], history[xj][xh])
										if distance_apart <= 0.3:
											cond_farAssign = False
											print('Goal [%f,%f] for robot %s repeated at other robot %s history' %(centroid_record[xxx][winner_id][0],centroid_record[xxx][winner_id][1], robot_namelist[xxx],robot_namelist[xj]))
											break
								# if the goal is repeated at own history
								else:
									for xh in range(0, len(history[xj])):
										distance_apart = calculateLocationDistance(centroid_record[xxx][winner_id], history[xj][xh])
										if distance_apart <= 0.1:
											remainingTime = ((robot_assigned_goal[xj]['time_start'] + robot_assigned_goal[xj]['time_thres']) - rospy.get_rostime().secs) 
											if remainingTime < (0.0-delay_after_assignement):
												cond_history = False
												print('history repeat the same goal assignment in robot %s history' %(robot_namelist[xxx]))
												break
							if cond_history and cond_goal and cond_farAssign:
								# robots[xxx].sendGoal(centroid_record[xxx][winner_id])
								robots[xxx].sendGoalTransformed(centroid_record[xxx][winner_id])
								robots[xxx].setGoalHistory(centroid_record[xxx][winner_id])	
								robot_assigned_goal[xxx]['lastgoal']   = robot_assigned_goal[xxx]['goal'].copy()
								robot_assigned_goal[xxx]['goal']       = centroid_record[xxx][winner_id]
								robot_assigned_goal[xxx]['startLoc']   = robots[xxx].getPosition()
								robot_assigned_goal[xxx]['time_start'] = rospy.get_rostime().secs
								robot_assigned_goal[xxx]['time_thres'] = calcDynamicTimeThreshold(robots[xxx].getPosition(), centroid_record[xxx][winner_id], time_per_meter)
								rospy.loginfo("@@@ " + robot_namelist[xxx] + " has been assigned to  "+str(centroid_record[xxx][winner_id]) + 	
								" mission start at " + str(robot_assigned_goal[xxx]['time_start']) + " sec  - Limit: " + str(int(robot_assigned_goal[xxx]['time_thres'])) + "s") 
								notAssign        = False
								# next_assign_time = rospy.get_rostime().secs + delay_after_assignement
								# next_assign_time = rospy.get_rostime().secs + (delay_after_assignement/float(len(robot_namelist)))
							else:
								# if not assigned
								attempt = attempt + 1
								print("======== Robot: %s attempt no %d to assign goal ==========" %(robot_namelist[xxx], attempt))
						else:
							print("++++++== Robot: %s give up assigning goal after %d attempt ==++++++" %(robot_namelist[xxx], attempt))
						
						# next_assign_time = rospy.get_rostime().secs + (delay_after_assignement/float(len(robot_namelist)))
					print('    Assignment of goal for robot %s used %.6f seconds \n' %(robot_namelist[xxx], (time.time()-start)))

					# check whether there is invalid goal or history after numerous attempt
					if (cond_history or cond_farAssign) and not cond_goal:
						goal_repeated   = True
						faultyGoal      = centroid_record[xxx][winner_id]
						if goal_repeated:
							repeatInvalid = False
							for ie in range(0,len(invalidFrontier)):
								if calculateLocationDistance(invalidFrontier[ie], robot_assigned_goal[xxx]['goal']) < 0.01:
									repeatInvalid = True
							if repeatInvalid == False:
								tempInvGoal = Point()
								tempInvGoal.x = robot_assigned_goal[xxx]['goal'][0]				
								tempInvGoal.y = robot_assigned_goal[xxx]['goal'][1]	
								tempInvGoal.z = 0.0	
								# if the distance is very far then do not publish as invalid point, mostly cant reach goal within the given time limit 
								distance = calculateLocationDistance(robots[xxx].getPosition(), robot_assigned_goal[xxx]['goal'])
								invalid_goal_array.points.append(copy(tempInvGoal))
								invalidFrontier.append(robot_assigned_goal[xxx]['goal'])		
								temp_inv_array.append(robot_assigned_goal[xxx]['goal'])
							print('Robot: %d assigned goal is repeated in the history' %(winner_id))
							robot_assigned_goal[xxx]['lastgoal']   = faultyGoal
							robot_assigned_goal[xxx]['goal']       = robots[xxx].getPosition()
							robot_assigned_goal[xxx]['time_start'] = rospy.get_rostime().secs
							robot_assigned_goal[xxx]['startLoc']   = robots[xxx].getPosition()
							robot_assigned_goal[xxx]['time_thres'] = -1
							# cancel goal
							rospy.loginfo("\n !!!> Robot " + robot_namelist[xxx] + " give up goal: " + str(robot_assigned_goal[xxx]['goal'])
							+ " at time: "  + str(int(currTime)) + " sec  -- distance: " + str(distance) + " -- goal repeated \n")
							robots[xxx].cancelGoal()
							robot_goal_cancel.append(xxx)
				
				# add delay time after assigning robot to the location
				next_assign_time = rospy.get_rostime().secs + delay_after_assignement
#-------------------------------------------------------------------------	
		

#-------------------------------------------------------------------------

#-------------------------------------------------------------------------
		# wont print so frequent
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
