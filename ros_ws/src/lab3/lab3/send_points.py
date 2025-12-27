#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# send_points.py
# Send navigation targets to the robot


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

import numpy as np

from threading import Lock

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav_targets.action import NavTarget
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from nav_msgs.msg import OccupancyGrid

# These are for transforming points/targets in the world into a point in the robot's coordinate space
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

# Your path planning
from lab3.path_planning import dijkstra, convert_image, is_free
from lab3.exploring import find_all_possible_goals, find_best_point


class SendPoints(Node):
	def __init__(self, points):
		""" Initialize way points
		@param - points, an iterable list of x,y tuples"""
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('send_points')

		# A mutex to keep us safe during the list deletions.
		self.mutex = Lock()

		# An action server to send the requests to.
		self.action_client = ActionClient(node=self, action_type=NavTarget, action_name='nav_target')

		# Save the points for when we startup
		self.current_point = 0
		self.points = [p for p in points]

		# Parameters that hold the current state of the action client
		self._send_goal_future = None
		self._goal_handle = None
		self._result_future = None

		# Subscriber after publisher; this is the map
		self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',  # topic name
            self.map_callback,
            10
        )

		# Create a buffer to put the transform data in
		self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
		self.transform_listener = TransformListener(self.tf_buffer, self)

		
		# Timer to make sure we publish the target marker and start the goal sending
		self.start_timer = self.create_timer(1.0, self._start_action_client)

		# Two sets of markers - one for the goal points, one for reachable points, one for path points
		self.goal_markers = None
		self.path_markers = None
		self.reachable_markers = None

		# Publishers for the RViz visualization
		self.goal_marker_pub = self.create_publisher(MarkerArray, 'goal_points', 1)
		self.path_marker_pub = self.create_publisher(MarkerArray, 'path_points', 1)
		self.reachable_marker_pub = self.create_publisher(MarkerArray, 'reachable_points', 1)

	def _start_action_client(self):
		""" Call this to start sending goal, or send the next goal"""

		# Cancel the timer - we're starting
		self.start_timer.cancel()

		if self.current_point == 0:
			# Wait for driver to start
			self.get_logger().info("Start driver.py to get started")
			self.action_client.wait_for_server()
		
		if self.current_point >= len(self.points):
			self.get_logger().info("No more points to send")
			return
			
		if self.current_point == 0:
			# First time through - make the marker points and publish them
			self._set_goal_markers()

		# send the current goal
		pt = self.points[self.current_point]
		self.current_point += 1

		# Create the goal point in the world coordinate frame
		goal = NavTarget.Goal()
		goal.goal.header.frame_id = 'odom'
		goal.goal.header.stamp = self.get_clock().now().to_msg()

		goal.goal.point.x = float(pt[0])
		goal.goal.point.y = float(pt[1])
		goal.goal.point.z = 0.0

		self.get_logger().info(f'Sending goal request... {self.current_point-1} of {len(self.points)} {pt[0], pt[1]}')

		# Send the driver the message that we're ready to send a goal point
		self._send_goal_future: Future = self.action_client.send_goal_async(goal=goal, 
														                    feedback_callback=self._feedback_callback)
		# This sets the call back for when the driver says it got the goal request 
		self._send_goal_future.add_done_callback(self._goal_sent_callback)

	def _goal_sent_callback(self, future : Future):
		""" This gets called when the server says I got the goal
		@param future - communicate with the server"""

		self._goal_handle: ClientGoalHandle = future.result()
		if not self._goal_handle.accepted:
			self.warn(f"{self.get_name()}: Action server not available; did you kill driver.py?")
		else:
			self.get_logger().info(f"Goal accepted")
			# Add a callback for the actual driver executing the goal
			self._result_future: Future = self._goal_handle.get_result_async()
			self._result_future.add_done_callback(self._goal_done_callback)

	def _goal_done_callback(self, future : Future):
		""" This gets called when the server says I finished the goal"""
		result: NavTarget.Result = future.result().result
		if result:
			self.get_logger().info(f"Got to goal {self.current_point}, moving to next")
			self.start_timer.reset() # Increment to the next goal	
		else:
			# GUIDE: This is where you should flag if you want to bail on the current set of goals
			# entirely or just skip to the next one
			self.get_logger().info(f"Did not get to goal, stopping {self.current_point}")

	def _feedback_callback(self, feedback):
		"""Every time driver loops in the action callback it send back the distance to the target as feedbackack
		@param feedback - data created by the action server - this has the distance in it (as a float)"""
		
		# Right now not doing anything but publishing the current distance
		
		self.last_distance = feedback.feedback.distance.data
		self.get_logger().info(f'Feedback: Distance: {feedback.feedback.distance.data}')

	def _set_goal_markers(self):
		""" Update the goal markers whenever the goals change"""
		if self.goal_markers == None:
			self.goal_markers = MarkerArray()

		# Lock while we make the Marker Array
		with self.mutex:
			line_marker = Marker()
			line_marker.header.frame_id = 'odom'
			line_marker.header.stamp = self.get_clock().now().to_msg()
			line_marker.type = Marker.LINE_STRIP
			line_marker.action = Marker.ADD
			line_marker.id = 0
			line_marker.scale.x = 0.1
			line_marker.scale.y = 0.1
			line_marker.scale.z = 0.1
			line_marker.color.r = 0.0
			line_marker.color.g = 0.0
			line_marker.color.b = 1.0
			line_marker.color.a = 1.0
			line_marker.points = []
			for p in self.points:
				pt = Point()
				pt.x = p[0]
				pt.y = p[1]
				pt.z = 0.0
				line_marker.points.append(pt)
			
			# Make the line(s) between the markers
			self.goal_markers.markers = []
			self.goal_markers.markers.append(line_marker)

			# Make the dots for the markers
			for indx, point in enumerate(self.points):
				marker = Marker()
				marker.header.frame_id = 'odom'
				marker.header.stamp = self.get_clock().now().to_msg()
				marker.id = line_marker.id + indx + 1
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = point[0]
				marker.pose.position.y = point[1]
				marker.pose.position.z = 0.0
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

				self.goal_markers.markers.append(marker)

		# Actually publish the list
		self.goal_marker_pub.publish(self.goal_markers)

	def _set_path_markers(self, path_list, skip=5):
		"""Update the path markers. Assumes path_list is a list of tuple x,y locations
		@param path_list - param list of tuples with x,y locations in map coordinate frame
		@param skip draw ever nth one"""
		if self.path_markers == None:
			self.path_markers = MarkerArray()

		# Lock while we make the Marker Array
		with self.mutex:
			line_marker = Marker()
			line_marker.header.frame_id = 'odom'
			line_marker.header.stamp = self.get_clock().now().to_msg()
			line_marker.type = Marker.LINE_STRIP
			line_marker.action = Marker.ADD
			line_marker.id = 10000
			line_marker.scale.x = 0.1
			line_marker.scale.y = 0.1
			line_marker.scale.z = 0.1
			line_marker.color.r = 1.0
			line_marker.color.g = 1.0
			line_marker.color.b = 0.0
			line_marker.color.a = 1.0
			line_marker.points = []
			for p in path_list[0::skip]:
				pt = Point()
				pt.x = p[0]
				pt.y = p[1]
				pt.z = 0.0
				line_marker.points.append(pt)
			
			# Make the line(s) between the markers
			self.path_markers.markers = []
			self.path_markers.markers.append(line_marker)

			# Make the dots for the markers
			for indx, point in enumerate(path_list[0::skip]):
				marker = Marker()
				marker.header.frame_id = 'odom'
				marker.header.stamp = self.get_clock().now().to_msg()
				marker.id = line_marker.id + indx + 1
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = point[0]
				marker.pose.position.y = point[1]
				marker.pose.position.z = 0.0
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0		
				marker.pose.orientation.w = 1.0
				marker.scale.x = 0.2
				marker.scale.y = 0.2
				marker.scale.z = 0.2
				marker.color.r = 1.0
				marker.color.g = 1.0
				marker.color.b = 0.0
				marker.color.a = 1.0

				self.path_markers.markers.append(marker)
				
		# Actually publish the list
		self.path_marker_pub.publish(self.path_markers)

	def _set_reachable_markers(self, points):
		""" Put markers on the reachable points
		@param points - list of x,y tuples in map space"""

		if self.reachable_markers == None:
			self.reachable_markers = MarkerArray()

		# Lock while we make the Marker Array
		with self.mutex:
			self.reachable_markers.markers = []

			# Make the dots for the markers
			for indx, point in enumerate(points):
				marker = Marker()
				marker.header.frame_id = 'odom'
				marker.header.stamp = self.get_clock().now().to_msg()
				marker.id = 10000 + indx + 1
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = point[0]
				marker.pose.position.y = point[1]
				marker.pose.position.z = 0.0
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0		
				marker.pose.orientation.w = 1.0
				marker.scale.x = 0.05
				marker.scale.y = 0.05
				marker.scale.z = 0.05
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 0.5
				marker.color.a = 1.0

				self.reachable_markers.markers.append(marker)
				
		# Actually publish the list
		self.reachable_marker_pub.publish(self.reachable_markers)

	def set_marker_points(self):
		"""Publishes the points in the list and links them up so they'll show up in RViz"""
		self._set_goal_markers()

	def from_map_to_image(self, map_msg : OccupancyGrid, pt_xy = (0.0, 0.0)):
		""" Convert from a point in the image to a point in the world
		@param map_msg - the map
		@param pt_xy - a tuple with an x,y in it
		@return pt_uv - point in the image"""
		info = map_msg.info

		im_u = 0
		im_v = 0

		# GUIDE: Subtract the origin position of the map and then divide by the resolution
		#   Don't forget to cast to an int
  # YOUR CODE HERE
		
		# self.get_logger().info(f"before {pt_xy} after {im_u}, {im_v}")
		return (im_u, im_v)
			
	def from_image_to_map(self, map_msg : OccupancyGrid, pt_uv = (0, 0)):
		""" Convert from a point in the world to a point in the image
		@param map_msg - the map
		@param pt_uv - a tuple with a u,v in width/height in it
		@return pt_xy - point in the world"""
		info = map_msg.info

		pt_x = 0.0
		pt_y = 0.0
		# GUIDE: Multiply by the resolution then add the origin position of the map 
  # YOUR CODE HERE
		# self.get_logger().info(f"before {pt_uv} after {pt_x}, {pt_y}")
		return (pt_x, pt_y)

	def map_callback(self, map_msg : OccupancyGrid):
		""" Called when the map gets updated. Size etc of the map is in the message"""
		self.get_logger().info(f"Got map size {(map_msg.info.width, map_msg.info.height)}, resolution {map_msg.info.resolution}")
		self.get_logger().info(f" Origin origin {map_msg.info.origin.position}")

	    # msg.data is a flat list of int8 values (-1 for unknown, 0 free, 100 occupied)
		im = np.array(map_msg.data, dtype=np.int8)

		# Reshape to (height, width)
		im = im.reshape((map_msg.info.height, map_msg.info.width))

		im_thresh = np.zeros(im.shape, dtype=np.uint8)

		# Threshold image
		im_thresh[im < 10] = 255    # Free
		im_thresh[im >= 100] = 0    # Wall
		im_thresh[im == -1] = 128   # Unknown

		self.get_logger().info(f"N free {np.count_nonzero(im_thresh == 255)}, N walls {np.count_nonzero(im_thresh == 0)}, N {np.count_nonzero(im_thresh == 128)}")

		# Location of robot
		transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
		robot_current_loc_in_map = (transform.transform.translation.x, transform.transform.translation.y)
		robot_current_loc_in_image = self.from_map_to_image(map_msg=map_msg, pt_xy=robot_current_loc_in_map)
		self.get_logger().info(f"Robot current location {robot_current_loc_in_map}")

		# GUIDE: Decide if you need to find new goal points/find a new path
		# This just looks for the last viable goal (that is free) - will grab a goal
		#  that's already been seen
		goal_loc_in_image = (map_msg.info.width // 2, map_msg.info.height // 2)
		if self.points:
			for p in self.points:
				try_goal_loc_in_image = self.from_map_to_image(map_msg=map_msg, pt_xy=p)
				if try_goal_loc_in_image[0] < map_msg.info.width and try_goal_loc_in_image[1] < map_msg.info.height:
					if is_free(im_thresh, try_goal_loc_in_image):
						goal_loc_in_image = try_goal_loc_in_image

		try:
			path = dijkstra(im_thresh, robot_current_loc_in_image, goal_loc_in_image)

			path_pts = []
			for p in path:
				map_xy = self.from_image_to_map(map_msg=map_msg, pt_uv=p)
				path_pts.append(map_xy)
			self._set_path_markers(path_pts, 5)
		except ValueError:
			if is_free(im_thresh, robot_current_loc_in_image):
				if is_free(im_thresh, goal_loc_in_image):
					self.get_logger().info(f"No valid path {robot_current_loc_in_image} to {goal_loc_in_image}")
				else:
					self.get_logger().info(f"Goal not free {robot_current_loc_in_image} to {goal_loc_in_image}")
			else:
				self.get_logger().info(f"Robot starting location not free {robot_current_loc_in_image}")

		# GUIDE: Change this to get just the points you might consider looking at
		all_unseen = find_all_possible_goals(im_thresh)
		reachable_pts = []
		for k, _ in all_unseen.items():
			map_xy = self.from_image_to_map(map_msg=map_msg, pt_uv=k)
			reachable_pts.append(map_xy)

		self._set_reachable_markers(reachable_pts)


# Unlike all the previous code, here we'll start up with a list of points to go to
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Create a list of points that will take the robot through the map
	points = [(-4.5, -3.0), (-4.5, 0.0), (1.0, 0.0)]
	send_points = SendPoints(points)

	# Multi-threaded execution
	executor = MultiThreadedExecutor()
	executor.add_node(send_points)
	executor.spin()
	#rclpy.spin(send_points)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()