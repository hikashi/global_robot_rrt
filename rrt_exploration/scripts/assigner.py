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
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

# def invalidCallBack(data):
# 	global invalidFrontier
# 	invalidFrontier=[]
# 	for point in data.points:
# 		invalidFrontier.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps,invalidFrontier
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
	robot_namelist = rospy.get_param('~robot_namelist', "robot1")
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	
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
			robots.append(robot(robot_namelist[i]))

	for i in range(0,len(robot_namelist)):
		rospy.loginfo('setting initial position for robot: %s' %(robot_namelist[i]))	
		robots[i].sendGoal(robots[i].getPosition())
		robot_assigned_goal.append({'robot_id':i, 'goal':robots[i].getPosition(), 'startLoc':robots[i].getPosition(),
		 'time_start':start_time, 'valid':True, 'time_thres':-1})


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
	while not rospy.is_shutdown():
		centroids=copy(frontiers)		
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
		if len(na) >= 1:
			rospy.loginfo("revenue record: "+str(revenue_record))	
			rospy.loginfo("centroid record: "+str(centroid_record))	
			rospy.loginfo("robot IDs record: "+str(id_record))	
		else:
			rospy.loginfo("- no idling robots detected, status update -")
			for ih in range(0, len(robot_namelist)):
				currTime = rospy.get_rostime().secs
				timeRemaining = (robot_assigned_goal[ih]['time_start'] + robot_assigned_goal[ih]['time_thres']) - currTime
				rospy.loginfo("robot " + robot_namelist[ih] + " working on goal (" + str(robot_assigned_goal[ih]['goal']) 
				+ ") - remaining time: " + str(timeRemaining) + "s")
				rospy.sleep(0.9)

			
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			# check if the goal is the same or not same
			cond_goal = True
			# if abs(robot_assigned_goal[id_record[winner_id]]['goal'][0] - centroid_record[winner_id][0]) < 0.000000001 and \
			# 	abs(robot_assigned_goal[id_record[winner_id]]['goal'][1] - centroid_record[winner_id][1]) < 0.000000001:
			# 	cond_goal = False	
			if robot_assigned_goal[id_record[winner_id]]['valid'] == True:
				history = robots[id_record[winner_id]].getGoalHistory()	
				for i in range(0, len(history)):
					if abs(history[i][0] - centroid_record[winner_id][0]) < 0.000000001 and abs(history[i][1] - centroid_record[winner_id][1]) < 0.000000001:
						cond_goal = False	
						print(">> Goal repeated in robot history")
						# robots[id_record[winner_id]].cancelGoal()
						break
				for i in range(0, len(invalidFrontier)):
					if robot_assigned_goal[id_record[winner_id]]['goal'][0] == invalidFrontier[i][0] and robot_assigned_goal[id_record[winner_id]]['goal'][1] == invalidFrontier[i][1]:
						cond_goal = False
						print(">> invalid goal detected.")
						# robots[id_record[winner_id]].cancelGoal()
						break
			currTime = rospy.get_rostime().secs
			# progress = (currTime - robot_assigned_goal[id_record[winner_id]]['time_start'])/robot_assigned_goal[id_record[winner_id]]['time_thres']
			# if progress < 0.2 and robot_assigned_goal[id_record[winner_id]]['time_thres'] != -1:
				# prevent early assignment and keep switching goal (progress 20% or less will be removed)
				# cond_goal = False
			
			if cond_goal:
				robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
				robot_assigned_goal[id_record[winner_id]]['goal']       = centroid_record[winner_id]
				robot_assigned_goal[id_record[winner_id]]['startLoc']   = robots[id_record[winner_id]].getPosition()
				robot_assigned_goal[id_record[winner_id]]['time_start'] = rospy.get_rostime().secs
				robot_assigned_goal[id_record[winner_id]]['valid']      = True
				robot_assigned_goal[id_record[winner_id]]['time_thres'] = calcDynamicTimeThreshold(robots[id_record[winner_id]].getPosition(), centroid_record[winner_id], time_per_meter)
				rospy.loginfo(robot_namelist[id_record[winner_id]] + " has been assigned to  "+str(centroid_record[winner_id]) + 	
				" mission start at " + str(robot_assigned_goal[id_record[winner_id]]['time_start']) + " sec  - Limit: " + str(robot_assigned_goal[id_record[winner_id]]['time_thres']) + "s") 
				rospy.sleep(delay_after_assignement)
		
		invalid_goal_array = invalidArray()
		invalid_goal_array.points = []
		
		for iz in range(0,len(invalidFrontier)):
			tempPoint = Point()
			tempPoint.z = 0.0
			tempPoint.x = invalidFrontier[iz][0]
			tempPoint.y = invalidFrontier[iz][1]
			invalid_goal_array.points.append(copy(tempPoint))

		for ix in range(0,len(robot_namelist)):
			currTime = rospy.get_rostime().secs
			if (currTime - robot_assigned_goal[ix]['time_start']) >= robot_assigned_goal[ix]['time_thres'] and robot_assigned_goal[ix]['time_thres'] != -1:		
				distance = calculateLocationDistance(robots[ix].getPosition(), robot_assigned_goal[ix]['goal'])
				repeatInvalid = False
				for ie in range(0,len(invalidFrontier)):
					if abs(invalidFrontier[ie][0] - robot_assigned_goal[ix]['goal'][0]) < 0.000000001 and abs(invalidFrontier[ie][1] - robot_assigned_goal[ix]['goal'][1]) < 0.000000001:
						repeatInvalid = True
				if repeatInvalid == False:
					tempInvGoal = Point()
					tempInvGoal.x = robot_assigned_goal[ix]['goal'][0]				
					tempInvGoal.y = robot_assigned_goal[ix]['goal'][1]	
					invalidFrontier.append(robot_assigned_goal[ix]['goal'])		
					tempInvGoal.z = 0.0	
					invalid_goal_array.points.append(copy(tempInvGoal))
					temp_inv_array.append(robot_assigned_goal[ix]['goal'])
					rospy.loginfo(" !!!!! > Robot " + robot_namelist[ix] + " give up goal: " + str(robot_assigned_goal[ix]['goal']) 
					+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance))
					# need to reset the assigned tasks to the robot
					robot_assigned_goal[ix]['goal'] = robots[ix].getPosition()
					robot_assigned_goal[ix]['valid'] = False
					robot_assigned_goal[ix]['time_start'] = rospy.get_rostime().secs
					robot_assigned_goal[ix]['startLoc'] = robots[ix].getPosition()
					robot_assigned_goal[ix]['time_thres'] = -1
							# else:
				# 	rospy.loginfo(" !---- > Robot " + namespace+str(i+namespace_init_count) + " repeat goal: " + str(robot_assigned_goal[ix]['goal']) 
				# 	+ " at time: "  + str(currTime) + " sec  -- distance: " + str(distance))

				# cancel goal
				robots[ix].cancelGoal()
			# elif (currTime - robot_assigned_goal[ix]['time_start']) == (robot_assigned_goal[ix]['time_thres'] + 5) and robot_assigned_goal[ix]['time_thres'] != -1:	
			# 		rospy.loginfo("Robot "  + namespace+str(i+namespace_init_count)+ " - Mission: " + str(robot_assigned_goal[ix]['goal']) + " - left less than 5 seconds")
		# header = Header()
		# header.frame_id = mapFrame
		# header.stamp = rospy.Time.now()
		# invalid_goal_array.header = header

		rospy.loginfo('publishing invalid goals :' + str(temp_inv_array))
		invPub.publish(invalid_goal_array)
		# publish invalid location for the points
		points.points = copy(invalid_goal_array.points)
		invCenPub.publish(points)


#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
