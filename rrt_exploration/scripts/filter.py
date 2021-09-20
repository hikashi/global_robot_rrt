#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf
from numpy import array, vstack, delete
from functions import gridValue, informationGain, checkSurroundingWall
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray, invalidArray


# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []
invalidFrontier=[]


def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
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

def globalMapCallBack(data):
    global globalmaps, litraIndx, robot_namelist
    # search the topic based on the robot name arrangement suplied by the user
    topic_breakdownlist = str(data._connection_header['topic']).split('/')
    for ia in range(0, len(robot_namelist)):
        if robot_namelist[ia] in topic_breakdownlist:
            indx = ia
    globalmaps[indx] = data

# Node----------------------------------------------


def node():
    global frontiers, mapData, globalmaps, litraIndx, robot_namelist, invalidFrontier
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
    # global_costmap_topic = rospy.get_param(
    #     '~global_costmap_topic', '/move_base/global_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    inv_frontier_topic= rospy.get_param('~invalid_frontier','/invalid_points')	
# -------------------------------------------
    robot_namelist = robot_namelist.split(',')
    litraIndx = len(robot_namelist)
    rate = rospy.Rate(rateHz)

    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(inv_frontier_topic, invalidArray, invalidCallBack)

#---------------------------------------------------------------------------------------------------------------
    for i in range(0, len(robot_namelist)):
        globalmaps.append(OccupancyGrid())


    for i in range(0, len(robot_namelist)):
        rospy.loginfo('waiting for  ' + robot_namelist[i])
        print(robot_namelist[i] + global_costmap_topic)
        rospy.Subscriber(robot_namelist[i] + global_costmap_topic, OccupancyGrid, globalMapCallBack)

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

    global_frame = "/"+mapData.header.frame_id

    tfLisn = tf.TransformListener()

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
        # rospy.loginfo("no frontier")

    points = Marker()
    points_clust = Marker()
# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 1
    points.lifetime = rospy.Duration()

    p = Point()

    p.z = 0

    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
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
    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

            # rospy.loginfo('clustering')
        # if there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            centroids = front

        frontiers = copy(centroids)
# -------------------------------------------------------------------------
# clearing old frontiers

        z = 0
        while z < len(centroids):
            cond1 = False
            cond2 = False
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]

            for i in range(0, len(robot_namelist)):

                transformedPoint = tfLisn.transformPoint(
                    globalmaps[i].header.frame_id, temppoint)
                x = array([transformedPoint.point.x, transformedPoint.point.y])

                if len(invalidFrontier) > 0:
                    # print('----yyyyy-----')
                    # print(centroids[z])
                    print(x)
                    # if x in invalidFrontier:
                    #     print(transformedPoint.point)
                    #     print(invalidFrontier)
                    cond_wall = checkSurroundingWall(globalmaps[i], x, 0.3)
                    # print(cond_wall)
                    # print('old frontier')
                cond1 = (gridValue(globalmaps[i], x) > threshold) or cond1
                # rospy.loginfo('clearing old frontiers- cond1')
                for j in range(0, len(invalidFrontier)):
                    if transformedPoint.point.x == invalidFrontier[j][0] and transformedPoint.point.y == invalidFrontier[j][1]:
                        cond2 = True
                        # print('ya cond2 set to true')
                    # rospy.loginfo('clearing old frontiers- cond2')
            # print("-------------------------------")
            # print((cond1 or cond2 or (informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius*0.5)) < 0.2))
            # print(invalidFrontier)
            infoGain = informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius*0.5)
            # if infoGain >= 0.2:
            #     print('>>>>> [%f,%f] - gain: %.4f' %(temppoint.point.x, temppoint.point.y, infoGain))
            if (cond1 or cond2 or infoGain < 0.2):
                centroids = delete(centroids, (z), axis=0)
                # if cond2:
                    # print('deleted the frontier list')
                # rospy.loginfo('clearing old frontiers - cpmbined')
                z = z-1 
            z += 1
# -------------------------------------------------------------------------
        for ii in range(0, len(centroids)):
            for jj in range(0, len(invalidFrontier)):
                if invalidFrontier[jj][0] == centroids[ii][0] and invalidFrontier[jj][1] == centroids[ii][1]:
                    print("!!!!!! invalid point detected -- [%f,%f]" %(centroids[ii][0], centroids[ii][1]))
# publishing
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
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
        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass