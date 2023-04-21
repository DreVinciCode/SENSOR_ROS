#!/usr/bin/python

import actionlib
import argparse
import math
import numpy as np
import os
import rospy

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import *
from tf import transformations

class WayPoint():
    def __init__(self):
        self._amcl_pose = None
        self.goal_sent = False    
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        
        
        self.current_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.Pose_callback)

        # try:
        #     rospy.spin()
        # except:
        #     rospy.logerr("Failed to call ROS spin")


    def Pose_callback(self, data):
        self._amcl_pose = data.pose.pose

    def getPose(self):
        return self._amcl_pose 

if __name__ == "__main__":
    try:
        rospy.init_node("NavGoal", anonymous = False)
        navigator = WayPoint()

        # while not rospy.is_shutdown():
        #     amcl_pose = navigator.getPose()
        #     print(amcl_pose)
        #     rospy.sleep(1.0)

        while not rospy.is_shutdown():
            amcl_pose = None
            while amcl_pose is not None:
                amcl_pose = navigator.getPose()
                print(amcl_pose)

    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")