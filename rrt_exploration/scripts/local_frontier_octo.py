#!/usr/bin/env python
from copy import copy
import rospy

# call back data for the program

# node calling for the program
def node():
	global frontiers
	rospy.init_node('frontier_detection', anonymous=False)
    # determine the rosparam

    # now working on extracting the data from the octomap

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 