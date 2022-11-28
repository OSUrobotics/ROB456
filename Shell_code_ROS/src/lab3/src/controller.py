#!/usr/bin/env python3


import sys
import rospy

import signal
from threading import Lock

import message_filters
import tf

from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import actionlib
from lab2.msg import NavTargetAction, NavTargetActionGoal


class RobotController:
	def __init__(self):
		# Since we're doing stuff with threads, then we should be good and use a mutex.
		self.mutex = Lock()

		self._waypoints = None

		self._odom = None
		self._map_data = None

		# We're going to use TF, so we'll need a transform listener.
		self.transform_listener = tf.TransformListener()

		# An action server to send the requests to.
		self.action_client = actionlib.SimpleActionClient('nav_target', NavTargetAction)
		self.action_client.wait_for_server()
		
		# Visualize the goal points.
		self.marker_pub = rospy.Publisher('goal_points', MarkerArray, queue_size=10)

		# Subscribe to the map and the map metadata.
		self.map_sub = rospy.Subscriber('map', OccupancyGrid, self._map_callback, queue_size=10)
		self.map_data_sub = rospy.Subscriber('map_metadata', MapMetaData, self._map_data_callback, queue_size=10)

		self.odom_pub = rospy.Subscriber('odom', Odometry, self._odom_callback, queue_size=1)

		# We're going to publish the markers at 10Hz using this timer.
		self.marker_timer = rospy.Timer(rospy.Duration(0.1), self._marker_callback)

		# Set up a signal handler to deal with ctrl-c so that we close down gracefully.
		signal.signal(signal.SIGINT, self._shutdown)

	@classmethod
	def _generate_point(cls, p):
		'''This function takes an (x, y) tuple, and returns a PointStamped in the map frame.'''
		map_point = PointStamped()
		map_point.header.frame_id = 'map'
		map_point.header.stamp = rospy.Time.now()
		map_point.point.x = p[0]
		map_point.point.y = p[1]
		map_point.point.z = 0.0

		return map_point

	def _shutdown(self, sig, frame):
		self.set_waypoints([])
		sys.exit()

	def _odom_callback(self, odom):
		self._odom = odom

	def _map_callback(self, map):
		point = PointStamped()
		point.header = self._odom.header
		point.point = self._odom.pose.pose.position

		try:
			point = self.transform_listener.transformPoint('map', point)
		except:
			point = None

		self.map_update(point, map, self._map_data)

	def _map_data_callback(self, data):
		self._map_data = data

	def _marker_callback(self, _):
		if not self._waypoints:
			return

		array = MarkerArray()

		with self.mutex:
			marker = Marker()
			marker.header.frame_id = 'map'
			marker.header.stamp = rospy.Time.now()
			marker.id = 0
			marker.type = Marker.LINE_STRIP
			marker.action = Marker.ADD
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0		
			marker.pose.orientation.w = 1.0
			marker.scale.x = 0.1
			marker.scale.y = 0.1
			marker.scale.z = 0.1
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
			marker.color.a = 1.0
			marker.points = [p.point for p in self._waypoints]

			array.markers.append(marker)

			for i, point in enumerate(self._waypoints):
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
		self.distance_update(feedback.distance.data)

	def set_waypoints(self, points):
		self.action_client.cancel_goal()

		with self.mutex:
			self._waypoints = [RobotController._generate_point(p) for p in points]

	def send_points(self):
		rate = rospy.Rate(10)
		while True:
			while self._waypoints and len(self._waypoints) > 0:
				rospy.loginfo(f'Sending target: ({self._waypoints[0].point.x:.2f}, {self._waypoints[0].point.y:.2f})')

				goal = NavTargetActionGoal()
				goal.goal = self._waypoints[0]
				goal.goal.header.stamp = rospy.Time.now()

				self.action_client.send_goal(goal, feedback_cb=self._feedback_callback)
				self.action_client.wait_for_result()

				result = self.action_client.get_result()

				with self.mutex:
					self._waypoints = self._waypoints[1:]

			rate.sleep()

	def distance_update(self, distance):
		""" This is a dummy class method that will be overwritten by the one in student_controller - change that
		 one, NOT this one"""
		raise NotImplemented('distance_update() not implemented')

	def map_update(self, point, map, map_data):
		""" This is a dummy class method that will be overwritten by the one in student_controller - change that
		 one, NOT this one"""
		raise NotImplemented('map_update() not implemented')


if __name__ == '__main__':
	rospy.init_node('robot_controller', argv=sys.argv)

	# This creates the controller then sends some starting way points to get the robot moving
	controller = RobotController()
	controller.set_waypoints(((-4, -3), (-4, 0), (5, 0)))
	controller.send_points()

	rospy.spin()

