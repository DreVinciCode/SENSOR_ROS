#!/usr/bin/python

import actionlib
import argparse
import numpy as np
import os
import rospy
import keyboard

from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction as MBA
from mbf_msgs.msg import MoveBaseActionGoal, ExePathAction, ExePathGoal
from std_srvs.srv import *
from std_msgs.msg import *
from nav_msgs.srv import *
from nav_msgs.msg import *
from math import atan2
from tf2_ros import *
# from tf2_ros import *

def generate_pointstamped():
    return (Point(0.59, -0.57, 0.0), # A
    		Point(0.62, 0.53, 0.0), # B
    		Point(0, 0.54, 0.0), # M
 			# set points from rebase_linkped room

            # points are in base_link frame
)

class WayPoint():
    def __init__(self):

        rospy.init_node("NavGoal", anonymous = False)


        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)


        self._amcl_pose = None
        self.goal_sent = False   

        self.WayPoints = []

        self.poseStampedArray = []
        self.paths = []
        self.combined_path = Path()
        self.combined_path.header.frame_id = "base_link" 

        self.createdPath = Path()
        self.createdPoseStampedArray = PoseStamped()

        self.targets = generate_pointstamped()
        
        self.move_base = actionlib.SimpleActionClient("move_base", MBA)

        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))
                
        self.current_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.Pose_callback)

        planPath_full_name = '/move_base/make_plan'
        rospy.wait_for_service(planPath_full_name)
        self.planPath = rospy.ServiceProxy(planPath_full_name, GetPlan)

        self.mainPlan = rospy.Publisher('customPlan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)

        self.move_base_flex = rospy.Publisher('/move_base_flex/move_base/goal', MoveBaseActionGoal, queue_size=10)

        self.click_point = rospy.Subscriber('clicked_point', PointStamped, self.clicked_point_callback )

        self.nav_waypoints = rospy.Subscriber('/NavWaypoints', Path, self.nav_waypoints_callback)


    def clearPaths(self):
        empty = Path()
        empty.header.frame_id = 'base_link'
        self.mainPlan.publish(empty)
        # self.createdPath = Path()


    def nav_waypoints_callback(self, data):
        tempPoseStampedArray = []


        for pose in data.poses:
            
            tempPoseStampedArray.append(pose)


        self.createdPath = data
        self.create_waypoints(data)
        # self.mainPlan.publish(self.createdPath)
        # self.combined_path = self.createdPath
        # print(len(data.poses))

    def clicked_point_callback(self, data):
        self.WayPoints.append(data)
        print(len(self.WayPoints))

    def move_base_flex_callback(self, path_msg):
        goal_msg = ExePathGoal()
        goal_msg.path = path_msg
        # goal_msg.controller = 'dwa'
        
        self.client.send_goal(goal_msg)

    def create_waypoints(self, data):
        print(len(data.poses))
        # self.poseStampedArray = []

        start = PoseStamped()
        start.pose = self.getAmcl_Pose().pose.pose
        self.poseStampedArray.append(start)

        for point in data.poses:
            if type(point) == type(PointStamped()):

                quat = Quaternion()
                quat.w = 1

                posestamped = PoseStamped()
                posestamped.header.frame_id = 'base_link'
                posestamped.header.stamp = rospy.Time.now()          
                posestamped.pose = Pose(point.point, quat)
                
                self.poseStampedArray.append(posestamped)

            else:
                print("poseStamped")
                self.poseStampedArray.append(point)

        self.create_paths_from_Waypoints(self.poseStampedArray)       

    def create_paths_from_Waypoints(self, waypoints):

        # Wait for the transformation to become available
        self.tfBuffer.can_transform("map", "base_link", rospy.Time(), rospy.Duration(1.0))

        for i in range(1, len(waypoints)):
            start = waypoints[i -1]
            goal = waypoints[i]

            start = self.tfBuffer.transformPose("map", start)
            start.header.frame_id = "map"
            goal = self.tfBuffer.transformPose("map", goal)
            goal.header.frame_id = "map"



            tolerance = 0.1
            sub_path =  self.planPath(start, goal, tolerance)
            
            self.path_append(sub_path.plan)

    def path_append(self, path):
        self.paths.append(path)
        self.combine_paths()

    def combine_paths(self):
        self.combined_path.poses = []
        for path in self.paths:
            self.combined_path.poses += path.poses

        self.mainPlan.publish(self.combined_path)

    def path_callback(self, msg):
        # Loop through each point in the Path message and move the robot to that point
        for pose in msg.poses:
            # Get the x and y coordinates of the goal position
            x_goal = pose.pose.position.x
            y_goal = pose.pose.position.y

            # Calculate the distance and angle to the goal point
            current_pose = rospy.wait_for_message("/odom", Odometry).pose.pose
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
                current_pose = rospy.wait_for_message("/odom", Odometry).pose.pose
                x_current = current_pose.position.x
                y_current = current_pose.position.y

        # Stop the Turtlebot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

    def getPlan(self):

        start = PoseStamped()
        start.pose = self.getAmcl_Pose().pose.pose
        goal = PoseStamped()
        goal.header.frame_id = 'base_link'
        goal.pose = self.getAmcl_Pose().pose.pose
        goal.pose.position.x = goal.pose.position.x + 1
        goal.pose.position.y = goal.pose.position.y + 1
        tolerance = Float32()
        tolerance = 0.5

        return self.planPath(start, goal, tolerance)   

    def Pose_callback(self, data):
        self._amcl_pose = data.pose.pose

    def getAmcl_Pose(self):
        return rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)

    def getPose(self):
        return self._amcl_pose 

if __name__ == "__main__":
    try:
        navigator = WayPoint()

        # while not rospy.is_shutdown():
        #     amcl_pose = navigator.getPose()
        #     print(amcl_pose)
        #     rospy.sleep(1.0)

        while not rospy.is_shutdown():

            if keyboard.read_key() == 'n':
                amcl_pose = navigator.getAmcl_Pose()

            elif keyboard.read_key() == 'b':
                plan = navigator.getPlan()
                plan.plan.header.frame_id = 'base_link'
                
                navigator.mainPlan.publish(plan.plan)

            elif keyboard.read_key() == 'p':
                # navigator.create_waypoints()
                # navigator.mainPlan.publish(navigator.combined_path)
                pass

            elif keyboard.read_key() == 'r':
                navigator.move_base_flex_callback(navigator.combined_path)

            elif keyboard.read_key() == 'c':
                navigator.clearPaths()
                

    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")