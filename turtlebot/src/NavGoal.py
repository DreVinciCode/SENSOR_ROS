#!/usr/bin/python

import actionlib
import argparse
import numpy as np
import os
import rospy
import keyboard

from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import *
from std_msgs.msg import *
from nav_msgs.srv import *
from nav_msgs.msg import *
from math import atan2
from tf import transformations

class WayPoint():
    def __init__(self):
        self._amcl_pose = None
        self.goal_sent = False    
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
        
        
        self.current_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.Pose_callback)

        execute_action_full_name = '/move_base/make_plan'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, GetPlan)

        self.mainPlan = rospy.Publisher('/customPlan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # try:
        #     rospy.spin()
        # except:
        #     rospy.logerr("Failed to call ROS spin")
        
    def path_callback(self, msg):
        # Loop through each point in the Path message and move the robot to that point
        for pose in msg.plan.poses:
            # Get the x and y coordinates of the goal position
            x_goal = pose.pose.position.x
            y_goal = pose.pose.position.y

            # Calculate the distance and angle to the goal point
            current_pose = rospy.wait_for_message('/odom', Odometry).pose.pose
            x_current = current_pose.position.x
            y_current = current_pose.position.y

            # Calculate the desired angle to turn towards the goal
            angle_to_goal = atan2(y_goal - y_current, x_goal - x_current)

            # Create a Twist message to rotate the Turtlebot
            rotate_twist = Twist()
            rotate_twist.angular.z = angle_to_goal

            # Publish the Twist message to rotate the Turtlebot
            self.cmd_vel_pub.publish(rotate_twist)

            # Create a Twist message to move the Turtlebot forward
            move_twist = Twist()
            move_twist.linear.x = 0.2

            # Publish the Twist message to move the Turtlebot forward
            self.cmd_vel_pub.publish(move_twist)

            # Wait for the Turtlebot to reach the goal position
            while abs(x_goal - x_current) > 0.1 or abs(y_goal - y_current) > 0.1:
                current_pose = rospy.wait_for_message('/odom', Odometry).pose.pose
                x_current = current_pose.position.x
                y_current = current_pose.position.y

        # Stop the Turtlebot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)


    def getPlan(self):

        start = PoseStamped()
        start.pose = self.getAmcl_Pose().pose.pose
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose = self.getAmcl_Pose().pose.pose
        goal.pose.position.x = goal.pose.position.x + 1
        tolerance = Float32()
        tolerance = 0.0

        # return plan.goal
        return self.execute_action(start, goal, tolerance)   

    def Pose_callback(self, data):
        self._amcl_pose = data.pose.pose

    def getAmcl_Pose(self):
        return rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)

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

            if keyboard.read_key() == 'n':
                amcl_pose = navigator.getAmcl_Pose()
                # print(amcl_pose.pose.pose)
                # amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
                # print(amcl_pose)
            elif keyboard.read_key() == 'b':
                # path = Path()
                path = navigator.getPlan()
                navigator.path_callback(path)
      
            # while amcl_pose is None:
            #     amcl_pose = navigator.getPose()
            #     print(amcl_pose)

    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")