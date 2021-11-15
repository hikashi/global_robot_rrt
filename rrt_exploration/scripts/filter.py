#!/usr/bin/env python

# --------Include modules---------------
import rospy
import tf
import time
from copy import copy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from numpy import array, vstack, delete, round
from functions import gridValue, informationGain, gridValueMergedMap
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray, invalidArray


# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []
localmaps  = []
invalidFrontier=[]


def callBack(data, args):
    global frontiers
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    # x = [array([data.point.x, data.point.y])]
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x

def mapCallBack(data):
    global mapData
    mapData = data

def invalidCallBack(data):
	global invalidFrontier
	invalidFrontier=[]
	for point in data.points:
		invalidFrontier.append(array([point.x,point.y]))

def localMapCallBack(data):
    global localmaps, robot_namelist
    topic_breakdownlist = str(data._connection_header['topic']).split('/')

    for ib in range(0, len(robot_namelist)):
        if robot_namelist[ib] in topic_breakdownlist:
            indx = ib
    localmaps[indx] = data

def globalCostMapCallBack(data):
    global globalmaps, robot_namelist
    topic_breakdownlist = str(data._connection_header['topic']).split('/')
    for ia in range(0, len(robot_namelist)):
        if robot_namelist[ia] in topic_breakdownlist:
            indx = ia
    globalmaps[indx] = data


# Node----------------------------------------------


def node():
    global frontiers, mapData,localmaps, globalmaps, robot_namelist, invalidFrontier
    rospy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 1.0)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    robot_namelist = rospy.get_param('~robot_namelist', 'robot1')
    rateHz = rospy.get_param('~rate', 100)
    global_costmap_topic = rospy.get_param(
        '~global_costmap_topic', '/move_base_node/global_costmap/costmap')
    local_map_topic = rospy.get_param('~local_map', '/map')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    cluster_bandwith = rospy.get_param('~cluster_bandwith', 0.5)
    inv_frontier_topic= rospy.get_param('~invalid_frontier','/invalid_points')
    localMapSub        = rospy.get_param('~localMapSub',False)	
    computeCycle       = rospy.get_param('~computeCycle',0.0)	
    debug1             = rospy.get_param('~debug1',False)	
    debug2             = rospy.get_param('~debug2',False)	
    # -------------------------------------------
    robot_namelist = robot_namelist.split(',')
    rate = rospy.Rate(rateHz)

    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(inv_frontier_topic, invalidArray, invalidCallBack)

#---------------------------------------------------------------------------------------------------------------
    for i in range(0, len(robot_namelist)):
        globalmaps.append(OccupancyGrid())
        localmaps.append(OccupancyGrid())


    for i in range(0, len(robot_namelist)):
        rospy.loginfo('waiting for  ' + robot_namelist[i])
        print(robot_namelist[i] + global_costmap_topic)
        rospy.Subscriber(robot_namelist[i] + global_costmap_topic, OccupancyGrid, globalCostMapCallBack)
        if localMapSub:
            rospy.Subscriber(robot_namelist[i] + local_map_topic, OccupancyGrid, localMapCallBack)

# wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)
        pass
# wait if any of robots' global costmap map is not received yet
    for i in range(0, len(robot_namelist)):
        while (len(globalmaps[i].data) < 1):
            rospy.loginfo('Waiting for the global costmap')
            rospy.sleep(0.1)
            pass

        if localMapSub:
            while (len(localmaps[i].data) < 1):
                rospy.loginfo('Waiting for the local map')
                rospy.sleep(0.1)
                pass

    global_frame = "/"+mapData.header.frame_id

    # adding in the try except for the tf tranformer to solve the issue
    try:
        tfLisn = tf.TransformListener()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.sleep(0.1)
        pass

    rospy.loginfo('Waiting for TF Transformer')
    for i in range(0, len(robot_namelist)):
        rospy.loginfo('Transforming - ' + robot_namelist[i] +'/'+robot_frame + ' in ' + global_frame[1:])
        tfLisn.waitForTransform(global_frame[1:], robot_namelist[i] +'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=callBack,
                     callback_args=[tfLisn, global_frame[1:]])
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)



    rospy.loginfo("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "frontiers"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.8
    points.scale.y = 0.8

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 0.8
    points.lifetime = rospy.Duration()

    p = Point()

    p.z = 0

    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "centroid"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.8
    points_clust.scale.y = 0.8
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 0.8
    points_clust.lifetime = rospy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    nextComputeTime = time.time() 

    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        if (time.time() + computeCycle) >= nextComputeTime:
            if debug2:
                print("now working on new cycle of the filtering process at time: %.3f" %(time.time()))
    
            # Clustering frontier points
            centroids = []
            front = copy(frontiers)

            if debug2:
                startTime = time.time()
            if len(front) > 1:
                ms = MeanShift(bandwidth=cluster_bandwith)
                ms.fit(front)
                centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

            # if there is only one frontier no need for clustering, i.e. centroids=frontiers
            if len(front) == 1:
                centroids = front
                if debug2:
                    print(">> The compute time for clustering took around %.4fs" %(time.time() - startTime))

            frontiers = copy(centroids)
            # -------------------------------------------------------------------------
            # clearing old frontiers


            if debug1 or debug2:
                startTime = time.time()
            z = 0
            while z < len(centroids):
                cond1 = False
                cond2 = False
                cond3 = False
                temppoint.point.x = centroids[z][0]
                temppoint.point.y = centroids[z][1]

                for i in range(0, len(robot_namelist)):
                    startTime2 = time.time()

                    transformedPoint = tfLisn.transformPoint(
                        globalmaps[i].header.frame_id, temppoint)
                    x = array([transformedPoint.point.x, transformedPoint.point.y])

                    cond1 = (gridValue(globalmaps[i], x) > threshold) or cond1
                    for jj in range(0, len(invalidFrontier)):
                        try:
                            if transformedPoint.point.x == invalidFrontier[jj][0] and transformedPoint.point.y == invalidFrontier[jj][1]:
                                cond2 = True
                        except:
                            print("@@@@@ point -> %d got problem with the following point: [%f %f]" %(jj, temppoint.point.x, temppoint.point.y))
                    if localMapSub:
                        localMapValue = gridValueMergedMap(localmaps[i], x)
                        if localMapValue > 90:
                            cond4 = True
                    # debugging tools
                    if debug2:
                        print(">> The local map grid map for robot %s calculation used time: %.3fs" %(robot_namelist[i], time.time()-startTime2))



                startTime2 = time.time()
                # now working with the cond3
                mapValue = gridValueMergedMap(mapData, [centroids[z][0], centroids[z][1]])
                if mapValue > 90: # if the map value is unknown or obstacle
                    cond3 = True
                # information gain function
                infoGain = informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius*0.5)
                if debug1:
                    print("info gain and merged map value calculation took around %.3fs" %(time.time()-startTime2))
               # if (cond1 or cond2 or (cond3 and cond4) or infoGain < 0.2):
                if localMapSub:
                    if (cond1 or cond2 or cond3 or cond4 or infoGain < 0.2):
                        # print('point: [%f, %f] Condition: %s %s %s %s %s - %f' %(centroids[z][0], centroids[z][1], str(cond1),str(cond2),str(cond3),str(cond4),str(infoGain < 0.2),infoGain))
                        centroids = delete(centroids, (z), axis=0)
                        z = z-1 
                else:
                    if (cond1 or cond3 or infoGain < 0.2):
                        # print('point: [%f, %f] Condition: %s %s %s %s %s - %f' %(centroids[z][0], centroids[z][1], str(cond1),str(cond2),str(cond3),str(cond4),str(infoGain < 0.2),infoGain))
                        centroids = delete(centroids, (z), axis=0)
                        z = z-1 
                z += 1
            if debug1 or debug2:
                print(">> The time to compute centroids and info gain took around %.4fs" %(time.time() - startTime))
            # -------------------------------------------------------------------------
            # publishing
            arraypoints.points = []
            for i in range(0, len(centroids)):
                invalidPts = False
                for j in range(0, len(invalidFrontier)):
                    if invalidFrontier[j][0] == centroids[i][0] and invalidFrontier[j][1] == centroids[i][1]:
                        invalidPts = True
                if not invalidPts:
                    tempPoint.x = round(centroids[i][0],2)
                    tempPoint.y = round(centroids[i][1],2)
                    arraypoints.points.append(copy(tempPoint))
            filterpub.publish(arraypoints)
            # rospy.loginfo('publish the Point array')
            pp = []
            for q in range(0, len(frontiers)):
                p.x = frontiers[q][0]
                p.y = frontiers[q][1]
                pp.append(copy(p))
            points.points = pp
            pp = []
            for q in range(0, len(centroids)):
                p.x = centroids[q][0]
                p.y = centroids[q][1]
                pp.append(copy(p))
            points_clust.points = pp
            pub.publish(points)
            pub2.publish(points_clust)
                    # rate.sleep() not sure need this or not.
            # rate.sleep()
            # update the compute cycle
            nextComputeTime = time.time() + computeCycle
            if debug2:
                print("next debug cycle time is: %.2f" %(nextComputeTime))
            rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
