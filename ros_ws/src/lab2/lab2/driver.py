#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# driver.py
# Drive the robot towards a goal, going around an object


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped

# math stuff
from math import atan2, tanh, sqrt, pi, fabs, cos, sin
import numpy as np

# Header for the twist message
from std_msgs.msg import Header

# The twist command and the goal
from geometry_msgs.msg import TwistStamped, PointStamped

# For publishing markers to rviz
from visualization_msgs.msg import Marker

# The laser scan message type
from sensor_msgs.msg import LaserScan

# These are all for setting up the action server/client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

# This is the format of the message sent by the client - it is another node under lab 2
from nav_targets.action import NavTarget

# These are for transforming points/targets in the world into a point in the robot's coordinate space
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point

# This sets up multi-threading so the laser scan can happen at the same time we're processing the target goal
from rclpy.executors import MultiThreadedExecutor


class Lab3Driver(Node):
	def __init__(self, threshold=0.2):
		""" We have parameters this time
		@param threshold - how close do you have to be before saying you're at the goal? Set to width of robot
		"""
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('driver')

		# Goal will be set later. The action server will set the goal; you don't set it directly
		self.goal = None
		# A controllable parameter for how close you have to be to the goal to say "I'm there"
		self.threshold = threshold

		# Make a Marker to put in RViz to show the current goal/target the robot is aiming for
		self.target_marker = None

		# Publisher before subscriber
		self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
		# Publish the current target as a marker (so RViz can show it)
		self.target_pub = self.create_publisher(Marker, 'current_target', 1)

		# Subscriber after publisher; this is the laser scan
		self.sub = self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)

		# Create a buffer to put the transform data in
		self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
		self.transform_listener = TransformListener(self.tf_buffer, self)

		# Action client for passing "target" messages/state around
		# An action has a goal, feedback, and a result. This class (the driver) will have the action server side, and be
		#   responsible for sending feed back and result
		# The SendPoints class will have the action client - it will send the goals and cancel the goal and send another when 
		#    the server says it has completed the goal
		# There is an initial call and response (are you ready for a target?) followed by the target itself
		#   goal_accept_callback handles accepting the goal
		#   cancel_callback is called if the goal is actually canceled by the action client
		#   execute_callback actually starts moving toward the goal
		self.action_server = ActionServer(node=self,
									action_type=NavTarget,
									action_name="nav_target",
									callback_group=ReentrantCallbackGroup(),
									goal_callback=self.goal_accept_callback,
									cancel_callback=self.cancel_callback,
									execute_callback=self.action_callback)

		# This is the goal in the robot's coordinate system, calculated in set_target
		self.target = PointStamped()
		self.target.point.x = 0.0
		self.target.point.y = 0.0

		# GUIDE: Declare any variables here
  # YOUR CODE HERE

		# Timer to make sure we publish the target marker (once we get a goal)
		self.marker_timer = self.create_timer(1.0, self._marker_callback)

		self.count_since_last_scan = 0
		self.print_twist_messages = False
		self.print_distance_messages = False

	def zero_twist(self):
		"""This is a helper class method to create and zero-out a twist"""
		# Don't really need to do this - the default values are zero - but can't hurt
		t = TwistStamped()
		t.header.frame_id = 'base_link'
		t.header.stamp = self.get_clock().now().to_msg()
		t.twist.linear.x = 0.0
		t.twist.linear.y = 0.0
		t.twist.linear.z = 0.0
		t.twist.angular.x = 0.0
		t.twist.angular.y = 0.0
		t.twist.angular.z = 0.0

		return t

	def _marker_callback(self):
		"""Publishes the target so it shows up in RViz"""
		if not self.goal:
			# No goal, get rid of marker if there is one
			if self.target_marker:
				self.target_marker.action = Marker.DELETE
				self.target_pub.publish(self.target_marker)
				self.target_marker = None
				self.get_logger().info(f"Driver: Had an existing target marker; removing")
			return
		
		# If we do not currently have a marker, make one
		if not self.target_marker:
			self.target_marker = Marker()
			self.target_marker.header.frame_id = self.goal.header.frame_id
			self.target_marker.id = 0
		
			self.get_logger().info(f"Driver: Creating Marker")

		# Build a marker for the target point
		#   - this prints out the green dot in RViz (the current target)
		self.target_marker.header.stamp = self.get_clock().now().to_msg()
		self.target_marker.type = Marker.SPHERE
		self.target_marker.action = Marker.ADD
		self.target_marker.pose.position = self.goal.point
		self.target_marker.scale.x = 0.3
		self.target_marker.scale.y = 0.3
		self.target_marker.scale.z = 0.3
		self.target_marker.color.r = 0.0
		self.target_marker.color.g = 1.0
		self.target_marker.color.b = 0.0
		self.target_marker.color.a = 1.0

		# Publish the marker
		self.target_pub.publish(self.target_marker)

		# Turn off the timer so we don't just keep making and deleting the target Marker
		#   Will get turned back on when we get an goal request
		self.marker_timer.cancel()

	def goal_accept_callback(self, goal_request : ServerGoalHandle):
		"""Accept a request for a new goal"""
		self.get_logger().info("Received a goal request")

		# Timer to make sure we publish the new target
		self.marker_timer.reset()

		# Accept all goals. You can use this (in the future) to NOT accept a goal if you want
		return GoalResponse.ACCEPT
	
	def cancel_callback(self, goal_handle : ServerGoalHandle):
		"""Accept or reject a client request to cancel an action."""
		self.get_logger().info('Received a cancel request')

		# Make sure our goal is removed
		self.goal = None

		# ...and robot stops
		t = self.zero_twist()
		self.cmd_pub.publish(t)
				
		# Timer to make sure we remove the current target (if there is one)
		self.marker_timer.reset()

		return CancelResponse.ACCEPT
	
	def close_enough(self):
		""" Return true if close enough to goal. This will be used in action_callback to stop moving toward the goal
		@ return true/false """

  # YOUR CODE HERE
		return False

	def distance_to_target(self):
		""" Communicate with send points - set to distance to target"""
		return np.sqrt(self.target.point.x ** 2 + self.target.point.y ** 2)
	
	# Respond to the action request.
	def action_callback(self, goal_handle : ServerGoalHandle):
		""" This gets called when the new goal is sent by SendPoints
		@param goal_handle - this has the new goal
		@return a NavTarget return when done """

		self.get_logger().info(f'Received an execute goal request... {goal_handle.request.goal.point}')
	
		# Save the new goal as a stamped point
		self.goal = PointStamped()
		self.goal.header = goal_handle.request.goal.header
		self.goal.point = goal_handle.request.goal.point
		
		# Build a result to send back
		result = NavTarget.Result()
		result.success = False

		# Reset target
		self.set_target()

		# Keep publishing feedback, then sleeping (so the laser scan can happen)
		# GUIDE: If you aren't making progress, stop the while loop and mark the goal as failed
		rate = self.create_rate(0.5)
		while not self.close_enough():
			if not self.goal:
				self.get_logger().info(f"Goal was canceled")

				return result
			
			feedback = NavTarget.Feedback()
			feedback.distance.data = self.distance_to_target()
			
			# Publish feedback - this gets sent back to send_points
			goal_handle.publish_feedback(feedback)

			# sleep so we can process the next scan
			rate.sleep()
			
		# Timer to make sure we remove the current target
		self.marker_timer.reset()

		# Don't keep processing goals
		self.goal = None 

		# Publish the zero twist
		t = self.zero_twist()
		self.cmd_pub.publish(t)

		self.get_logger().info(f"Completed goal")

		# Set the succeed value on the handle
		goal_handle.succeed()

		# Set the result to True and return
		result.success = True
		return result

	def set_target(self):
		""" Convert the goal into an x,y position (target) in the ROBOT's coordinate space
		@return the new target as a Point """

		if self.goal:
			# Transforms for all coordinate frames in the robot are stored in a transform tree
			#  odom is the coordinate frame of the "world", base_link is the base link of the robot
			# A transform stores a rotation/translation to go from one coordinate system to the other
			transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))

			# This applies the transform to the Stamped Point
			#    Note: This does not work, for reasons that are unclear to me
			self.target = do_transform_point(self.goal, transform)
			
			# This does the transform manually, by calculating the theta rotation from the quaternion
			euler_ang = -atan2(2 * transform.transform.rotation.z * transform.transform.rotation.w,
			                   1.0 - 2 * transform.transform.rotation.z * transform.transform.rotation.z)
			
			# Translate to the base link's origin
			x = self.goal.point.x - transform.transform.translation.x
			y = self.goal.point.y - transform.transform.translation.y

			# Do the rotation
			rot_x = x * cos(euler_ang) - y * sin(euler_ang)
			rot_y = x * sin(euler_ang) + y * cos(euler_ang)

			self.target.point.x = rot_x
			self.target.point.y = rot_y
			if self.print_distance_messages:
				self.get_logger().info(f'Target relative to robot: ({self.target.point.x:.2f}, {self.target.point.y:.2f}), orig ({self.goal.point.x, self.goal.point.y})')
			
		else:
			if self.print_distance_messages:
				self.get_logger().info(f'No target to get distance to')
			self.target = None		
		
		# GUIDE: Calculate any additional variables here
		#  Remember that the target's location is in its own coordinate frame at 0,0, angle 0 (x-axis)
  # YOUR CODE HERE

		return self.target

	def scan_callback(self, scan):
		""" Lidar scan callback
		@param scan - has information about the scan, and the distances (see stopper.py in lab1)"""
	
		if self.print_twist_messages:
			self.get_logger().info("In scan callback")
		# Got a scan - set back to zero
		self.count_since_last_scan = 0

		# If we have a goal, then act on it, otherwise stay still
		if self.goal:
			# Recalculate the target point (assumes we've moved)
			self.set_target()

			# Call the method to actually calculate the twist
			t = self.get_twist(scan)
		else:
			t = self.zero_twist()
			#t.twist.linear.x = 0.1
			if self.print_twist_messages:
				self.get_logger().info(f"No goal, sitting still")

		# Publish the new twist
		self.cmd_pub.publish(t)

	def get_obstacle(self, scan):
		""" check if an obstacle
		@param scan - the lidar scan
		@return Currently True/False and speed, angular turn"""

		if not self.target:
			return False, 0.0, 0.0
		
		# GUIDE: Use this method to collect obstacle information - is something in front of, to the left, or to 
		# the right of the robot? Start with your stopper code from Lab1
  # YOUR CODE HERE
		return False, 0.0, 0.0

	def get_twist(self, scan):
		"""This is the method that calculate the twist
		@param scan - a LaserScan message with the current data from the LiDAR.  Use this for obstacle avoidance. 
		    This is the same as your lab1 go and stop code
		@return a twist command"""
		t = self.zero_twist()

		# GUIDE:
		#  Step 1) Calculate the angle the robot has to turn to in order to point at the target
		#  Step 2) Set your speed based on how far away you are from the target, as before
		#  Step 3) Add code that veers left (or right) to avoid an obstacle in front of it
		# Reminder: t.linear.x = 0.1    sets the forward speed to 0.1
		#           t.angular.z = pi/2   sets the angular speed to 90 degrees per sec
		# Reminder 2: target is in self.target 
		#  Note: If the target is behind you, might turn first before moving
		#  Note: 0.4 is a good speed if nothing is in front of the robot

		min_speed = 0.05
		max_speed = 0.2         # This moves about 0.01 m between scans
		max_turn = np.pi * 0.1  # This turns about 2 degrees between scans

  # YOUR CODE HERE

		# t.twist.linear.x = max_speed
		# t.twist.angular.z = 0.0
		if self.print_twist_messages:
			self.get_logger().info(f"Setting twist forward {t.twist.linear.x} angle {t.twist.angular.z}")
		return t			


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	driver = Lab3Driver()

	# Multi-threaded execution
	executor = MultiThreadedExecutor()
	executor.add_node(driver)
	executor.spin()
	
	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
