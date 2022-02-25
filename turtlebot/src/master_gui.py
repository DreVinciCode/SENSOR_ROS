#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool

from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton


class KivyMD_GUI(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)


		
	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		return self.screen

	def inc_person_threshold(self, *args):

		# msg=True
		# pub.publish(msg)
		if rospy.has_param('/SENSAR/person_threshold'):
			threshold = rospy.get_param("/SENSAR/person_threshold")
			threshold = round(threshold  + 0.1, 1)
			rospy.set_param('/SENSAR/person_threshold', threshold)
			print(threshold)

	def dec_person_threshold(self, *args):

		if rospy.has_param('/SENSAR/person_threshold'):
			threshold = rospy.get_param("/SENSAR/person_threshold")
			threshold = round(threshold  - 0.1, 1)
			rospy.set_param('/SENSAR/person_threshold', threshold)
			print(threshold)


if __name__ == "__main__":
	
	# pub = rospy.Publisher('/SENSAR/person_threshold', Bool, queue_size=2)

	rospy.init_node('kivymd_gui', anonymous=True)	

	KivyMD_GUI().run()