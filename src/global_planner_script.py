#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String, Int16
from geometry_msgs.msg import PoseStamped
from math import *

class PathPlanner():
	def __init__(self):
		rospy.init_node("path_planner", anonymous=True)

		self.goal_pub = rospy.Publisher('/global_planner/goal', PoseStamped, queue_size=10)
		self.planner_costmap_pub = rospy.Publisher('/global_planner/costmap/costmap', OccupancyGrid, queue_size=10)

		self.choice_index = 0
		self.choice_index_prev = 0

		self.goalMsg = PoseStamped()
		self.goalMsg.header.frame_id = "map"
		self.goalMsg.pose.position.x = 2
		self.goalMsg.pose.position.y = -3
		self.goalMsg.pose.orientation.z = 0.0
		self.goalMsg.pose.orientation.w = 1.0
		self.goalMsg.header.stamp = rospy.Time.now()

		while not rospy.is_shutdown():

			self.goal_pub.publish(self.goalMsg)

def main():
	PathPlanner()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
