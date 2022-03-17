#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton


class ARViz_GUI(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)

	
	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		return self.screen

	def inc_person_threshold(self, *args):

		if rospy.has_param('/SENSAR/person_threshold'):
			threshold = rospy.get_param("/SENSAR/person_threshold")
			threshold = round(threshold  + 0.1, 1)
			rospy.set_param('/SENSAR/person_threshold', threshold)
			print(threshold)

			self.screen.ids.person_threshold_value_text.text = str(threshold)

	def dec_person_threshold(self, *args):

		if rospy.has_param('/SENSAR/person_threshold'):
			threshold = rospy.get_param("/SENSAR/person_threshold")
			threshold = round(threshold  - 0.1, 1)
			rospy.set_param('/SENSAR/person_threshold', threshold)
			print(threshold)

			self.screen.ids.person_threshold_value_text.text = str(threshold)

	def slider_function(self, slider_value):
		# print(round(slider_value,0))
		# pub.publish(slider_value)
		roundvalue = round(slider_value,1)

		if rospy.has_param('/SENSAR/person_threshold'):
			threshold = rospy.get_param("/SENSAR/person_threshold")
			rospy.set_param('/SENSAR/person_threshold', roundvalue)
			self.screen.ids.person_threshold_value_text.text = str(roundvalue)


if __name__ == "__main__":
	
	pub = rospy.Publisher('/SENSAR/slider_value', Float64, queue_size=1)
	rospy.init_node('kivymd_gui', anonymous=True)	
	ARViz_GUI().run()

