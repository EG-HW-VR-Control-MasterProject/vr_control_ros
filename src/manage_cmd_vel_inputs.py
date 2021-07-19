#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Twist
#from itemPose.msg import itemPose
from master_project.msg import itemPoseList, itemPose
from math import *

class CmdVelManager():
	def __init__(self):
		rospy.init_node("cmd_vel_manager", anonymous=True)
		self.choice_index_sub = rospy.Subscriber('/choice_cmd_input', Int32, self.choice_cmd_input_cb)
		#self.explore_lite_cmd_sub = rospy.Subscriber('/explore_lite_cmd', Twist, self.explore_lite_cmd_cb)
		self.explore_lite_cmd_sub = rospy.Subscriber('/input_explore_lite', Twist, self.explore_lite_cmd_cb)
		self.teleop_cmd_sub = rospy.Subscriber('/teleop_cmd', Twist, self.teleop_cmd_cb)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.choice_index = 0
		self.choice_index_prev = 0

		self.teleop_speed = Twist()
		self.explore_lite_speed = Twist()


		"""
		while not rospy.is_shutdown():
			
			if self.choice_index == 0:
				print("Manual mode")
				self.cmd_vel_pub.publish(self.teleop_speed)

			if self.choice_index == 1:
				print("Autonomous mode")
				self.cmd_vel_pub.publish(self.explore_lite_speed)
		"""
	
	def choice_cmd_input_cb(self, msg):
		print("New input command received")
		if self.choice_index != msg.data:
			# Send a Empty velocity to stop the robot if the method change
			self.cmd_vel_pub.publish(Twist())
		self.choice_index = msg.data
	
	def teleop_cmd_cb(self, msg):
		self.teleop_speed = msg
		if self.choice_index == 0:
			print("Manual mode")
			self.cmd_vel_pub.publish(self.teleop_speed)
		
	def explore_lite_cmd_cb(self, msg):
		self.explore_lite_speed = msg
		if self.choice_index == 1:
			print("Autonomous mode")
			self.cmd_vel_pub.publish(self.explore_lite_speed)
		

	

def main():
	CmdVelManager()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
