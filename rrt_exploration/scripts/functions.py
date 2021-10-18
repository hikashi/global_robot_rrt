import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
import numpy as np
import random
# ________________________________________________________________________________


class robot:
    def __init__(self, name, move_base_service, plan_service, global_frame, base_link):
        rospy.loginfo('setting up robot init for robot - ' + name)
        self.assigned_point = []
        self.goal_history = []
        ###################################################################
        self.name = name
        self.goal = MoveBaseGoal()
        self.start = PoseStamped()
        self.end = PoseStamped()
        ###################################################################
        self.global_frame = rospy.get_param('~global_frame', global_frame)
        self.robot_frame = rospy.get_param('~robot_frame', "map")
        self.plan_service =  rospy.get_param('~plan_service', plan_service)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        ###################################################################
        self.total_distance = 0
        self.first_run = True
        self.sub = rospy.Subscriber(name + "/odometry/tracking_filtered", Odometry, self.odom_callback)
        ###################################################################
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(
            self.name+move_base_service, MoveBaseAction)
        self.client.wait_for_server()
        self.goal.target_pose.header.frame_id = self.name + "/" + self.robot_frame # self.global_frame
        self.goal.target_pose.header.stamp = rospy.Time.now()
        ###################################################################
        rospy.wait_for_service(self.name+self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name+self.plan_service, GetPlan)
        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame        
        ###################################################################

    def odom_callback(self, data):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, data.header.frame_id, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0

        if self.first_run == True:
            self.previous_x = trans[0]
            self.previous_y = trans[1]
        x = trans[0]
        y = trans[1]
        d_increment = np.sqrt(((x - self.previous_x)*(x - self.previous_x)) + ((y - self.previous_y)*(y - self.previous_y)))
        self.total_distance = self.total_distance + d_increment
        # print("Total distance traveled is %.2f" %(self.total_distance))
        self.first_run = False
        self.previous_x = x
        self.previous_y = y
        
    def getDistanceTraveled(self):
        return self.total_distance

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def transformPointToRobotFrame(self, trans):
        while not rospy.is_shutdown():
            try:
                point = PoseStamped()
                t = self.listener.getLatestCommonTime(self.name + '/' + self.robot_frame, self.global_frame)
                point.header.frame_id = self.global_frame

                point.pose.position.x = trans[0]
                point.pose.position.y = trans[1]

                point.pose.orientation.w = 1.0

                transformed = self.listener.transformPose(self.name + '/' + self.robot_frame, point)
                return (transformed.pose.position.x, transformed.pose.position.y)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def sendGoalTransformed(self, point):
        transform_point = self.transformPointToRobotFrame(point)
        self.goal.target_pose.pose.position.x = transform_point[0]
        self.goal.target_pose.pose.position.y = transform_point[1]
        self.goal.target_pose.pose.orientation.w = 1.0
        # print('robot: %s  x: %f y: %f' %(self.name, point[0],point[1]))
        self.client.send_goal(self.goal)
        self.goal_history.append(array(point))
        self.assigned_point = array(point)

    def sendGoal(self, point):
        self.goal.target_pose.pose.position.x = point[0]
        self.goal.target_pose.pose.position.y = point[1]
        self.goal.target_pose.pose.orientation.w = 1.0
        # print('robot: %s  x: %f y: %f' %(self.name, point[0],point[1]))
        self.client.send_goal(self.goal)
        self.goal_history.append(array(point))
        self.assigned_point = array(point)

    def cancelGoal(self):
        point = self.getPosition()
        self.goal.target_pose.pose.position.x = point[0]
        self.goal.target_pose.pose.position.y = point[1]
        self.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.goal)
        self.assigned_point = array(point)

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        self.start.pose.position.x = start[0]
        self.start.pose.position.y = start[1]
        self.end.pose.position.x = end[0]
        self.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name+'/map', self.start)
        end = self.listener.transformPose(self.name+'/map', self.end)
        # plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        plan = self.make_plan(start=start, goal=end, tolerance=0.03)
        # tolerance should be defined in meter as according to the data, first try 0.04
        return plan.plan.poses

    def setGoalHistory(self, point):
        self.goal_history.append(point)
        
    def getGoalHistory(self):
        return self.goal_history

# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int((floor((Xp[1]-Xstarty)/resolution) *
                  width)+(floor((Xp[0]-Xstartx)/resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i/mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (float(i-(int(i/mapData.info.width)*(mapData.info.width)))*mapData.info.resolution)
    # modified for certain python version might mismatch the int and float covnersion
    return array([x, y])
# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0.0
    index = index_of_point(mapData, point)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                # print(array(point))
                # print(point_of_index(mapData, i))
                # print(norm(array(point)-point_of_index(mapData, i)))
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain += 1.0
    return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain
# ________________________________________________________________________________
def calculateLocationDistance(input_loc, dest_loc):
    return(np.sqrt(np.power(input_loc[0]-dest_loc[0],2) + np.power(input_loc[1]-dest_loc[1],2)))

# ________________________________________________________________________________
def calcDynamicTimeThreshold(curr_pos, goal_pos, time_per_meter):
    # this function use to dynamic determine the mission time threshold for the robot
    dist1 = np.sqrt(np.power(curr_pos[0]-goal_pos[0],2) + np.power(curr_pos[1]-goal_pos[1],2))
    if dist1 < 1.0:
        return time_per_meter
    elif dist1 >= 1.0 and dist1 <= 15.0:
        return int(time_per_meter * dist1)
    else:
        return int(time_per_meter * 15.0)


# ________________________________________________________________________________
def pathCost(path):
    if (len(path) > 0):
        i = len(path)/2
        p1 = array([path[i-1].pose.position.x, path[i-1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1-p2)*(len(path)-1)
    else:
        return inf
# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == 1):
                    return True
    return False
# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :]-x)
        if (n1 < n):
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i]-x)

        if (n1 < n):
            n = n1
    return i


def gridValueMergedMap(mapData, Xp, distance=2):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data  = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    outData = squareAreaCheck(Data, index, width, distance)
    # knownAreaPercentage = outData.count   (0)/(np.power((distance*2)+1,2))

    # if knownAreaPercentage <= 0.95:
    if max(outData,key=outData.count) != -1 and max(outData) != -1: 
        return max(outData,key=outData.count)
    elif max(outData,key=outData.count) == -1 and max(outData) == 100:
        return max(outData)
    elif max(outData,key=outData.count) == -1 and max(outData) != 100:
        return max(outData)
    elif max(outData,key=outData.count) == 0 and max(outData) == 100:
        return max(outData)
    elif max(outData,key=outData.count) == 0 and max(outData) == 0:
        return -1.0
    else:
        return -1.0
    # else:
    #     return -1.0
# ________________________________________________________________________________
def squareAreaCheck(data, index, width, distance=2):
    # now using the data to perform a square area check on the data for removing the invalid point
    dataOutList = []
    for j in range(-1*distance, distance+1):
        for i in range(int(index)+((width*j)-distance), int(index)+((width*j)+distance)):
            if i < len(data):
                dataOutList.append(data[int(i)])
    return dataOutList

# ________________________________________________________________________________
def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    outData = squareAreaCheck(Data, index, width, distance=2)
    if len(outData) > 1:
        return max(outData)
    else:
        return 100

# ________________________________________________________________________________
def checkSurroundingWall(mapData, Xp, dist_check):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data

    pointList = []
    pointList.append([Xp[0]-dist_check, Xp[1]])
    pointList.append([Xp[0]+dist_check, Xp[1]])
    pointList.append([Xp[0], Xp[1]-dist_check])
    pointList.append([Xp[0], Xp[1]+dist_check])
    pointList.append([Xp[0]-dist_check, Xp[1]-dist_check])
    pointList.append([Xp[0]-dist_check, Xp[1]+dist_check])
    pointList.append([Xp[0]+dist_check, Xp[1]-dist_check])
    pointList.append([Xp[0]+dist_check, Xp[1]+dist_check])
    pointList.append([Xp[0]-dist_check, Xp[1]-(dist_check/2.0)])
    pointList.append([Xp[0]-dist_check, Xp[1]+(dist_check/2.0)])
    pointList.append([Xp[0]+dist_check, Xp[1]-(dist_check/2.0)])
    pointList.append([Xp[0]+dist_check, Xp[1]+(dist_check/2.0)])
    pointList.append([Xp[0]-(dist_check/2.0), Xp[1]-dist_check])
    pointList.append([Xp[0]-(dist_check/2.0), Xp[1]+dist_check])
    pointList.append([Xp[0]+(dist_check/2.0), Xp[1]-dist_check])
    pointList.append([Xp[0]+(dist_check/2.0), Xp[1]+dist_check])


    indexList = []
    for i in range(0, len(pointList)):
        randomNumber1 = ((random.randint(-100, 100))/100.0)*dist_check
        randomNumber2 = ((random.randint(-100, 100))/100.0)*dist_check

        index = (floor((pointList[i][1]+randomNumber1-Xstarty)/resolution)*width) + \
            (floor((pointList[i][0]+randomNumber2-Xstartx)/resolution))
        if int(index) < len(Data):
            indexList.append(Data[int(index)])
        else:
            indexList.append(100)

    # now checking the pair list.list()
    sortedList = sorted(indexList, reverse=True)
    sumNum = sortedList[0] + sortedList[1] + sortedList[2] + sortedList[3]

    wallDetected = False
    if sumNum > 190:
        wallDetected = True
    # print('-------------------------------')
    # print(wallDetected)
    # print(sortedList)
    # print('the total number: %d' %sumNum)
    # print('-------------------------------')
    # print(Xp)
    # print(pointList)
    # print(indexList)
    # print(wallDetected)
    return wallDetected
