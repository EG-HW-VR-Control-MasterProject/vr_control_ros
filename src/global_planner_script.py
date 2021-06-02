#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry

from geometry_msgs.msg import PoseStamped
from itemPose.msg import itemPose
from itemPoseList.msg import itemPoseList
from math import *

class PathPlanner():
	def __init__(self):
		rospy.init_node("path_planner", anonymous=True)
		#self.sub_costmap = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_cb)
		self.index_list_sub = rospy.Subscriber('/item/item_list', itemPoseList, self.index_list_cb)
		self.choice_index_sub = rospy.Subscriber('/item/choice_item_index', Int, self.choice_index_cb)
		self.goal_pub = rospy.Publisher('/global_planner/goal', PoseStamped, queue_size=10)
		self.planner_costmap_pub = rospy.Publisher('/global_planner/costmap/costmap', OccupancyGrid, queue_size=10)

		self.choice_index = 0
		self.choice_index_prev = 0

		self.item_selected = itemPose()

		self.updatedItemList = itemPoseList()

		self.goalMsg = PoseStamped()
		self.goalMsg.header.frame_id = "map"
		self.goalMsg.pose.position.x = -2
		self.goalMsg.pose.position.y = -3
		self.goalMsg.pose.orientation.z = 0.0
		self.goalMsg.pose.orientation.w = 1.0
		self.goalMsg.header.stamp = rospy.Time.now()

		while not rospy.is_shutdown():
			self.item_selected = self.updatedItemList[self.choice_index]

			self.goalMsg.pose = item_selected.pose
			self.goal_pub.publish(self.goalMsg)
	
	def choice_index_cb(self, msg):
		self.choice_index = msg
		#if self.choice_index_prev != self.choice_index:
	
	def index_list_cb(self, msg):
		self.updatedItemList = msg


def main():
	PathPlanner()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
