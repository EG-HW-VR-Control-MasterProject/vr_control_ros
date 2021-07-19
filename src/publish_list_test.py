#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
#from itemPose.msg import itemPose
from master_project.msg import itemPoseList, itemPose
from math import *

class GoalSender():
	def __init__(self):
		rospy.init_node("goal_sender", anonymous=True)
		#self.choice_index_pub = rospy.Publisher('/item/choice_item_index', Int32, queue_size=10)
		self.index_list_pub = rospy.Publisher('/item/item_list', itemPoseList, queue_size=10)
		self.id_list_pub = rospy.Publisher('/item/id_list', String, queue_size=10)
		self.choice_index = 0
		self.choice_index_prev = 0

		self.counter = 0

		self.item_selected = itemPose()
		self.item_selected_second = itemPose()
		self.item_selected_third = itemPose()
		
		self.updatedItemList = itemPoseList()
		self.updatedIdList = String()
		
		self.item_selected.id = "1st"
		self.item_selected.pose.position.x = -2
		self.item_selected.pose.position.y = -3
		self.updatedIdList.data += self.item_selected.id

		self.updatedItemList.item_pose_list.append(self.item_selected)

		self.item_selected_second.id = "2nd"
		self.item_selected_second.pose.position.x = 4
		self.item_selected_second.pose.position.y = -2
		self.updatedIdList.data += "-" + self.item_selected_second.id
		self.updatedItemList.item_pose_list.append(self.item_selected_second)

		self.item_selected_third.id = "3rd"
		self.item_selected_third.pose.position.x = 2
		self.item_selected_third.pose.position.y = -1
		self.updatedIdList.data += "-" + self.item_selected_third.id
		#self.updatedIdList.data += "-" + self.item_selected_second.id
		self.updatedItemList.item_pose_list.append(self.item_selected_third)



		while not rospy.is_shutdown():
			#if self.counter < 3:
			print(self.updatedIdList)
			self.index_list_pub.publish(self.updatedItemList)
			self.id_list_pub.publish(self.updatedIdList)
			self.counter += 1
	
	

def main():
	GoalSender()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
