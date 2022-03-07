#!/usr/bin/python

'''
sensar_ros_scalesafetyzone.py
SENSAR
Andre Cleaver
Tufts University
'''

import rospy

class ScaleSafetyZone:

    def __init__(self):
		self.var = True

if __name__ == "__main__":
    
    print("terdst")
    
    try:
        rospy.init_node("Scale_SafetyZone", anonymous = False)
        
        Scale = ScaleSafetyZone()

    except rospy.ROSInterruptException:
        rospy.loginfo("Quitting")
