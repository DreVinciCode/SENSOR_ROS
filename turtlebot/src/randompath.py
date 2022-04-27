#!/usr/bin/env python

'''
sensar_ros_randomtargets.py
SENSAR
Andre Cleaver
Tufts University
April 27th, 2022
'''

import actionlib
import math
import rospy
import time
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf import transformations


def generate_pointstamped():
    return (Point(0.03, -0.04, 0.0), # A
    		Point(2.134, -2.12, 0.0), # B
    		Point(3.876, -0.03, 0.0), # M
    		Point(2.176, -0.03, 0.0), # N
 			# set points from remapped room

            # points are in map frame
)

class randomtargets:
    def __init__(self):
        self.pub = rospy.Publisher("/SENSAR/random_path", Path, queue_size = 1)
        self.targets = generate_pointstamped()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        self.next_path()

    def next_path(self):
        
        for point in self.targets:

            input("press enter to start...")

            poseStampedArray = []
            quat = Quaternion()
            quat.w = 1 

            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = rospy.Time.now()

    
            posestamped = PoseStamped()
            posestamped.header.frame_id = 'map'
            posestamped.header.stamp = rospy.Time.now()          
            posestamped.pose = Pose(point, quat)
            poseStampedArray.append(posestamped)

            path.poses = poseStampedArray
            
            print(path)
            self.pub.publish(path) 




    def next_target(self):
        rate = rospy.Rate(10)

        for point in self.targets:

            input("press enter to start.")
            
            quat = transformations.quaternion_from_euler(0, 0, math.radians(0))
            quaternion = {'r1' : quat[0], 'r2' : quat[1], 'r3' : quat[2], 'r4' : quat[3]}
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()    
            goal.target_pose.pose = Pose(point, quaternion) 
            
            print(goal)
            self.move_base.send_goal(goal) 



if __name__  == "__main__":

	try:
		rospy.init_node('random_points', anonymous=False)
		random_points = randomtargets()

	except rospy.ROSInterruptException:
		rospy.loginfo("Quiting")
