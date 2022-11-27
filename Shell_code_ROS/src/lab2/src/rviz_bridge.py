#!/usr/bin/env python3


import sys
import rospy

from threading import Lock, Thread

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import actionlib

from lab2.msg import NavTargetAction, NavTargetGoal


class RvizBridge:
	def __init__(self):
		self.action_client = actionlib.SimpleActionClient('nav_target', NavTargetAction)
		self.action_client.wait_for_server()

		self.point_publisher = rospy.Publisher('goal_points', MarkerArray, queue_size=10)
		self.click_subscriber = rospy.Subscriber('clicked_point', PointStamped, self._click_callback, queue_size=10)

		self.queue = []
		self.mutex = Lock()

		self.running = False
		self.dispatch_thread = Thread(target=self._dispatch)

		self._display()

	def _dispatch(self):
		while self.running:
			with self.mutex:
				if len(self.queue) > 0:
					goal = self.queue[0]
					self.queue = self.queue[1:]
				else:
					goal = None

			if goal:
				print(f'Goal: ({goal.point.x}, {goal.point.y})')
				target = NavTargetGoal()
				target.goal = goal

				rospy.loginfo('sending action')
				self.action_client.send_goal(target)
				self.action_client.wait_for_result()
				success = self.action_client.get_result()
				rospy.loginfo(f'Got response: {success.success}')
			else:
				rospy.sleep(0.01)

	def _display(self):
		array = MarkerArray()

		with self.mutex:
			marker = Marker()
			marker.header.frame_id = 'odom'
			marker.header.stamp = rospy.Time.now()
			marker.id = 0
			marker.type = Marker.LINE_STRIP
			marker.action = Marker.ADD
			marker.scale.x = 0.1
			marker.scale.y = 0.1
			marker.scale.z = 0.1
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
			marker.color.a = 1.0
			marker.points = self.queue

			array.markers.append(marker)

			for i, point in enumerate(self.queue):
				marker = Marker()
				marker.header.frame_id = point.header.frame_id
				marker.header.stamp = rospy.Time.now()
				marker.id = i + 1
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
#				marker.pose.position = point.point
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0		
				marker.pose.orientation.w = 1.0
				marker.scale.x = 0.2
				marker.scale.y = 0.2
				marker.scale.z = 0.2
				marker.color.r = 0.0
				marker.color.g = 0.0
				marker.color.b = 1.0
				marker.color.a = 1.0

				array.markers.append(marker)

		self.point_publisher.publish(array)

	def _click_callback(self, point):
		print('click')
		with self.mutex:
			self.queue.append(point)

		self._display()

	def start(self):
		self.running = True
		self.dispatch_thread.start()

	def stop(self):
		self.running = False
		self.dispatch_thread.join()


if __name__ == '__main__':
	rospy.init_node('rviz_interface', argv=sys.argv)

	bridge = RvizBridge()
	bridge.start()

	rospy.spin()

	bridge.stop()
