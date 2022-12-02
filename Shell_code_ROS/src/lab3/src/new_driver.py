#!/usr/bin/env python3


import rospy
import sys

from math import atan2, sqrt, tanh

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import actionlib
import tf

from lab2.msg import NavTargetAction, NavTargetResult, NavTargetFeedback


class Driver:
	def __init__(self, position_source, threshold=0.1):
		self._target_point = None
		self._threshold = threshold

		self.transform_listener = tf.TransformListener()

		self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.target_pub = rospy.Publisher('current_target', Marker, queue_size=1)

		self._lidar_sub = rospy.Subscriber('base_scan', LaserScan, self._lidar_callback, queue_size=10)

		# Action client
		self._action_server = actionlib.SimpleActionServer('nav_target', NavTargetAction, execute_cb=self._action_callback, auto_start=False)
		self._action_server.start()

	@classmethod
	def zero_twist(cls):
		t = Twist()
		t.linear.x = 0.0
		t.linear.y = 0.0
		t.linear.z = 0.0
		t.angular.x = 0.0
		t.angular.y = 0.0
		t.angular.z = 0.0

		return t

	# Respond to the action request.
	def _action_callback(self, goal):
		rospy.loginfo(f'Got an action request for ({goal.goal.point.x:.2f}, {goal.goal.point.y:.2f})')

		result = NavTargetResult()

		self._target_point = goal.goal

		# Build a marker for the goal point
		marker = Marker()
		marker.header.frame_id = goal.goal.header.frame_id
		marker.header.stamp = rospy.Time.now()
		marker.id = 0
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position = goal.goal.point
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0		
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		rate = rospy.Rate(100)
		while self._target_point:
			if self._action_server.is_preempt_requested():
				self._target_point = None
				result.success.data = False
				self._action_server.set_succeeded(result)

			self.target_pub.publish(marker)
			rate.sleep()

		result.success.data = True
		self._action_server.set_succeeded(result)

	def _lidar_callback(self, lidar):
		if self._target_point:
			self._target_point.header.stamp = rospy.Time.now()
			try:
				target = self.transform_listener.transformPoint('base_link', self._target_point)
				x = target.point.x
				y = target.point.y
				distance = sqrt(x ** 2 + y ** 2)

				feedback = NavTargetFeedback()
				feedback.distance.data = distance
				self._action_server.publish_feedback(feedback)

				# This is where it moves to the next goal position. f you want to have a "smarter" reached goal criteria, then change
				#  close_enough_to_waypoint to return True for that case
				if self.close_enough_to_waypoint(distance, (target.point.x, target.point.y), lidar):
					self._target_point = None
					command = Driver.zero_twist()
				else:
					command = self.get_twist((target.point.x, target.point.y), lidar)
			except:
				return
		else:
			command = Driver.zero_twist()

		self._cmd_pub.publish(command)

	def close_enough_to_waypoint(self, distance, target, lidar):
		""" This is a dummy class method that will be overwritten by the one in student_driver - change that
		 one, NOT this one"""
		raise NotImplemented('close_enough_to_waypoint() not implemented')

	def get_twist(self, target, lidar):
		""" This is a dummy class method that will be overwritten by the one in student_driver - change that
		 one, NOT this one"""
		raise NotImplemented('get_twist() not implemented')


if __name__ == '__main__':
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver('odom')

	rospy.spin()
