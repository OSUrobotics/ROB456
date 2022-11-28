#!/usr/bin/env python3


import sys
import rospy

import numpy as np

from threading import Lock

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import actionlib
from lab2.msg import NavTargetAction, NavTargetActionGoal


class SendPoints:
	def __init__(self, points):
		# A mutex to keep us safe during the list deletions.
		self.mutex = Lock()

		# An action server to send the requests to.
		self.action_client = actionlib.SimpleActionClient('nav_target', NavTargetAction)
		self.action_client.wait_for_server()

		# Publisher for the visualization
		self.marker_pub = rospy.Publisher('goal_points', MarkerArray, queue_size=10)

		# Make a list of stamped points. This is the list of points used in send_points below (the way points)
		# Each point has to be of type "PointStamped" beause that is what the action server expects
		self.points = []

		for p in points:
			goal = PointStamped()
			goal.header.frame_id = 'odom'
			goal.header.stamp = rospy.Time.now()

			goal.point.x = p[0]
			goal.point.y = p[1]
			goal.point.z = 0

			self.points.append(goal)

		self.marker_timer = rospy.Timer(rospy.Duration(0.1), self._marker_callback)

	def _marker_callback(self, event):
		"""Publishes the points in the list and links them up so they'll show up in RViz"""
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
			marker.points = [p.point for p in self.points]

			array.markers.append(marker)

			for i, point in enumerate(self.points):
				marker = Marker()
				marker.header.frame_id = point.header.frame_id
				marker.header.stamp = rospy.Time.now()
				marker.id = i + 1
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position = point.point
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

		self.marker_pub.publish(array)

	def _feedback_callback(self, feedback):
		"""Every time we get a laser scan (that triggers the robot moving), send back the distance to the target back
		@param feedback - data created by the action server - this has the distance in it (as a float)"""
		rospy.loginfo(f'Distance: {feedback.distance.data:.2f}')
		
	def send_points(self):
		"""This is called to kick things off (see below). This calls an action server and sends it the points
		that were created in __init__(). """

		# While we have any points left in self.points
		while len(self.points) > 0:
			# Create a new target goal
			rospy.loginfo(f'Sending target: ({self.points[0].point.x:.2f}, {self.points[0].point.y:.2f})')

			goal = NavTargetActionGoal()
			goal.goal = self.points[0]

			# This sends the goal to the action server...
			self.action_client.send_goal(goal, feedback_cb=self._feedback_callback)
			#  ... and this waits for the action server to let you know it got to the goal
			self.action_client.wait_for_result()

			result = self.action_client.get_result()

			with self.mutex:
				self.points = self.points[1:]


if __name__ == '__main__':
	rospy.init_node('send_points', argv=sys.argv)

	# Create a list of points on a circle. In __init__, this is turned into a kist of PointStamped
	points = [(2 * np.cos(theta), 2 * np.sin(theta)) for theta in np.linspace(0.0, 2 * np.pi, 15)]

	# Create the class above, initialized with the set of points
	sender = SendPoints(points)

	# Now start sending the points to the action server
	sender.send_points()

	# When all the points have been visited, we just exit
	rospy.log_info("All done!")
