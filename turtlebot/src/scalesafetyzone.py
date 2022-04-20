#!/usr/bin/env python

'''
scalesafetyzone.py
SENSAR
Andre Cleaver
Tufts University
'''

import rospy
from std_msgs.msg import Header, Float32

def generate_scales():
    return 3.0, 1.0, 4.0, 2.0,

class randomscale:
    def __init__(self):
        self.pub = rospy.Publisher("/SENSAR/safetyzone_scale", Float32, queue_size = 1)
        self.scales = generate_scales()

    def next_scale(self):

        for scale in self.scales:
            raw_input("press enter to start...")			
            
            print(scale)
            self.pub.publish(scale)           


if __name__  == "__main__":

	try:
		rospy.init_node('scale_safetyzone', anonymous=False)
		random_points = randomscale()
		random_points.next_scale()

	except rospy.ROSInterruptException:
		rospy.loginfo("Quiting")
