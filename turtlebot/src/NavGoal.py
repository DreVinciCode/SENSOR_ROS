#!/usr/bin/python

import actionlib
import argparse
import math
import numpy as np
import os
import rospy

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import *
from tf import transformations

class WayPoint():
    def __init__(self):
        self.goal_sent = False
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	    rospy.loginfo("Wait for the action server to come up")
	    self.move_base.wait_for_server(rospy.Duration(5))


if __name__ == "__main__":
    try:
        rospy.init_node("NavGoal", anonymous = False)
        navigator = WayPoint()

    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")
