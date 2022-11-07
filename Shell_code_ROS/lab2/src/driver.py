#!/usr/bin/env python3


import rospy
import sys

from math import atan2, tanh, sqrt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion


class Driver:
	def __init__(self, position_source, goal=None, threshold=0.01):
		self.goal = goal
		self.threshold = threshold

		# Publisher before subscriber
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.sub = rospy.Subscriber(position_source, Odometry, self._odom_callback, queue_size=10)

	def _odom_callback(self, odom):
		# We're going to send a command every time, even if it is to stay still
		command = Twist()
		command.linear.x = 0
		command.linear.y = 0
		command.linear.z = 0
		command.angular.x = 0
		command.angular.y = 0
		command.angular.z = 0

		if self.goal:
			# Get the angle of the robot from the odometry quaternion
			q = odom.pose.pose.orientation
			_, _, robot_angle = euler_from_quaternion((q.x, q.y, q.z, q.w))

			# Get the angle and distance ot the goal
			pose = odom.pose.pose.position
			goal_angle = atan2(self.goal.y - pose.y, self.goal.x - pose.x)
			goal_distance = sqrt((pose.x - self.goal.x) ** 2 + (pose.y - self.goal.y) ** 2)

			rospy.loginfo(f'robot angle: {robot_angle} goal angle: {goal_angle}, goal distance: {goal_distance}')

			angle = goal_angle - robot_angle

			command = Twist()
			command.linear.x = tanh(goal_distance) if goal_distance > self.threshold else 0
			command.angular.z = tanh(angle * 5) if abs(goal_angle) > self.threshold else 0

			if goal_distance < self.threshold and goal_angle < self.threshold:
				self.goal = None
				rospy.loginfo('Reached goal')

			rospy.loginfo(f'angle: {angle}, rotation: {command.angular.z}')

		self.pub.publish(command)


	def set_goal(self, goal):
		self.goal = goal


if __name__ == '__main__':
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver('odom')

	target = Point()
	target.x = 5
	target.y = 0
	target.z = 0

	driver.set_goal(target)

	rospy.spin()
