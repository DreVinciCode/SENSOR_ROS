#!/usr/bin/python

'''
lab_478_path.py
SENSAR
Andre Cleaver
Tufts University
March 22, 2022
'''

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


class GoToPose():
	def __init__(self):

	    self.counter = 0
	    self.goal_sent = False

	    rospy.on_shutdown(self.shutdown)

	    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	    rospy.loginfo("Wait for the action server to come up")
	    self.move_base.wait_for_server(rospy.Duration(5))


	def goto(self, pos, quat):

		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		self.move_base.send_goal(goal)

		success = self.move_base.wait_for_result(rospy.Duration(90))

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
		    # We made it!
		    result = True
		else:
		    self.move_base.cancel_goal()

		self.goal_sent = False
		return result

	def shutdown(self):
		if self.goal_sent:
		    self.move_base.cancel_goal()
		rospy.loginfo("Stop")
		rospy.sleep(1)

if __name__ == "__main__":
    
    key = ['y']

    point_A = [2.15, 1.21, -90]
    point_B = [2.15, -5.35, -90]

    try:
        rospy.init_node("lab_478_path", anonymous = False)
        navigator = GoToPose()

        start_index = 0
        goal_index = start_index
        locations_names = ['A', 'B']
        num_location = len(locations_names)

        location_coord = np.zeros([num_location,3])
        location_coord[0] = point_A
        location_coord[1] = point_B


        counter = 0

        while not rospy.is_shutdown():
            
            turtlebot_orientation_in_degrees = location_coord[goal_index][2]
            
            #convert euler to quaternion and save in new variable quat
            quat = transformations.quaternion_from_euler(0, 0, math.radians(turtlebot_orientation_in_degrees))
            
            # Customize the following values so they are appropriate for your location
            position = {'x': location_coord[goal_index][0], 'y' : location_coord[goal_index][1]}
            quaternion = {'r1' : quat[0], 'r2' : quat[1], 'r3' : quat[2], 'r4' : quat[3]}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Hooray, reached point " + locations_names[goal_index])

                # request user to input file name for rosbag
                if locations_names[goal_index] == 'A':
                    counter += 1

                    proceed = raw_input("To continue with path press 'y'...")

                    while proceed not in key:
                        proceed = raw_input("You must press 'y'...")


                elif locations_names[goal_index] == 'B':
                    rospy.wait_for_service('/move_base/clear_costmaps')
                    clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                    resp1 = clear_costmap()
                    print("cleared Costmap")

                else:
                    pass

            else:
                rospy.loginfo("The base failed to reach point " + locations_names[goal_index])
                rospy.wait_for_service('/move_base/clear_costmaps')
                clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                resp1 = clear_costmap()

            goal_index += 1

            # num_location
            if goal_index >= num_location:
                goal_index = 0

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)


    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")
