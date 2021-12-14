#!/usr/bin/python3

'''
roundtrip.py
SENSAR
Andre Cleaver
Tufts University
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
from tf import transformations

parser = argparse.ArgumentParser(description="A script to command the Turtlebot2 to make a round trip on selected points on a map.")
parser.add_argument('--n', metavar='NumPoints', type=int , help="Specify the number of points (2-10) for the robot to travel towards.")
args=parser.parse_args()

numPoints = 0
minPoints = 2
maxPoints = 10

waypointArray = []


class GoToWayPoints:

    def __init__(self):
        rospy.init_node("Roundtrip")
        self.goal_sent = False
        self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        if (self.goal_sent):
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def goto(self, waypoint):

        self.goal_sent = True
        turtlebot_orientation_in_degrees = 0
        rotation = transformations.quaternion_from_euler(0,0, math.radians(turtlebot_orientation_in_degrees))        
        rotation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(waypoint.point, rotation)

        self.move_base.send_goal(goal)
        success = self.move_base.wait_for_result()
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result



def NumberOfWayPoints():            
    userInput = minPoints
    
    try:
        if (args.n >= minPoints) and (args.n <= maxPoints) :
            print("Entered in %(value)s points." % {"value" : args.n})
            return int(args.n)        
        else:
            while True:
                userInput = input("Enter number of points between 2 and 10: ")
                try:
                    if(isinstance(int(userInput), int) and int(userInput) >= minPoints and int(userInput) < maxPoints):
                        print("Entered in %(value)s points." % {"value" : userInput})
                        return int(userInput)     
                except:
                    continue
    except:
        while True:
            userInput = input("Enter number of points between 2 and 10: ")
            try:
                if(isinstance(int(userInput), int) and int(userInput) >= minPoints and int(userInput) < maxPoints):
                    print("Entered in %(value)s points." % {"value" : userInput})

                    return int(userInput)     
            except:
                continue

def point_callback(data):
    waypointArray.append(data)

    if(len(waypointArray) == waypointTotal):
        rospy.signal_shutdown("Points Entered.")
    else:
        print("Select " + str(waypointTotal - len(waypointArray)) + " more points.")


def WaypointList():
    print("Select " + str(waypointTotal) + " points on the Rviz map using the Publish Point button.")

    while (len(waypointArray) < waypointTotal):
        msg = rospy.wait_for_message("/clicked_point", PointStamped, timeout=60)
        waypointArray.append(msg)
        if(len(waypointArray) == waypointTotal):
            pass
        else:
            print("Select " + str(waypointTotal - len(waypointArray)) + " more points.")

    return waypointArray


def StartRoundTrip(wayPoints):

    roundtrip = GoToWayPoints()

    for waypoint in wayPoints:
        success = roundtrip.goto(waypoint)  


if __name__ == "__main__":
    
    try:
        rospy.init_node("Roundtrip", anonymous = False)
        waypointTotal = NumberOfWayPoints()
        waypointArray = WaypointList()

        while True:
            StartRoundTrip(waypointArray)

    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")