#! usr/bin/python

'''
sensar_ros_randomtargets.py
SENSAR
Darren Tang
Tufts University
'''

import rospy
from geometry_msgs.msg import Point, PointStamped
frpm std_msgs.msg import Header

def generate_pointstamped():
    return (Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0),
            Point(0, 0, 0), # Last one is center (origin) of the room
)

class randomtargets:
    def __init__(self):
        self.pub = rospy.Publisher("/SENSAR/point", PointStamped, 5)
        self.targets = generate_pointstamped()
    def next_target():
        i = 0
        while i < len(self.targets):
            ptstmp = PointStamped()
            ptstmp.Point = self.targets[i]
            ptstmp.Header = Header
            yield ptstmp
        yield ptstmp
    def publish_latest():
        self.pub.publish(self.next_target)
