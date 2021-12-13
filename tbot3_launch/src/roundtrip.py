#!/usr/bin/python3

'''
roundtrip.py
SENSAR
Andre Cleaver
Tufts University
'''

import argparse
import numpy as np
import os
import rospy

from geometry_msgs.msg import Pose, Point, PointStamped

parser = argparse.ArgumentParser(description="A script to command the Turtlebot2 to make a round trip on selected points on a map.")
parser.add_argument('--n', metavar='NumPoints', type=int , help="Specify the number of points (2-10) for the robot to travel towards.")
args=parser.parse_args()

numPoints = 0
minPoints = 2
maxPoints = 10



class GoToWayPoints:

    def __init__(self):
        rospy.init_node("roundtrip", anonymous=False)
        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback)

        self.pointArray[]
        rospy.spin()


    def point_callback(self, data):
        self.pointArray.append(data)





def NumberOfWayPoints():            
    userInput = 2
    
    try:
        if args.n >= 2 :
            print("Entered in %(value)s points." % {"value" : args.n})
            userInput = args.n        
    
    except:
        while True:
            userInput = input("Enter number of points between 2 and 10: ")
            try:
                if(isinstance(int(userInput), int) and int(userInput) >= minPoints and int(userInput) < maxPoints):
                    print("Entered in %(value)s points." % {"value" : userInput})

                    return int(userInput)     
            except:
                continue


def WaypointList():

    rospy.Subscriber("/clicked_point", PointStamped, point_callback)


def point_callback(data):
    print()


if __name__ == "__main__":
    

    try:
        waypointTotal = NumberOfWayPoints()

        waypointArray = WaypointList()

        # WayPointFollower = GoToWayPoints()

        # while()


    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")