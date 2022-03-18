#!/usr/bin/env python

'''
sensar_ros_randomtargets.py
SENSAR
Darren Tang
Tufts University
'''

import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

def generate_pointstamped():
    return (Point(2.157, 1.408, 0.0), # D
    		Point(2.134, -2.12, 0.0), # B
    		Point(3.876, -4.63, 0.0), # C
 			# set points from remapped room
)

class randomtargets:
    def __init__(self):
        self.pub = rospy.Publisher("/SENSAR/random_point", PointStamped, queue_size = 1)
        self.targets = generate_pointstamped()

    def next_target(self):

        for point in self.targets:
            raw_input("press enter to start...")			
            ptstmp = PointStamped()
            ptstmp.point = point
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "map"
            ptstmp.header = h

            print(ptstmp)
            self.pub.publish(ptstmp)           


if __name__  == "__main__":

	try:
		rospy.init_node('random_points', anonymous=False)
		random_points = randomtargets()
		random_points.next_target()

	except rospy.ROSInterruptException:
		rospy.loginfo("Quiting")
