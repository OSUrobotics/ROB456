#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# dumb_stopper.py
# This example gives the robot callback-based stopping (see something in the laser, stop).


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node


# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import TwistStamped

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


class DumbStopper(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('dumb_stopper')

		# Set up a publisher.  The default topic for Twist messages is cmd_vel.
		self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

		# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
		self.sub = self.create_subscription(LaserScan, 'base_scan', self.callback, 10)

		# No timer this time because we will get regular callbacks for the laser scanner
		#  So that callback will handle changing the twist message

		# Start off not moving
		self.b_is_stopped = True

	def callback(self, scan):
		# This is a callback that will fire every time a laser scan message comes in.
		# Note that there's no timer for this one - this code only gets called when 
		# Every time we get a laser scan, calculate the shortest scan distance, and set
		# the speed accordingly.

		# scan.ranges is a numpy array with distances
		shortest = min(scan.ranges)

		# Should the robot stop y/n?
		if shortest < 1.0:
			self.b_is_stopped = True
		else:
			self.b_is_stopped = False

		# Create a Twist and fill in the information.  Note that we fill in values
		# even for the elements we're not going to use.  We don't have to do this,
		# but it's good practice.
		t = TwistStamped()
		# if something is close, don't move. Otherwise, go forward
		t.twist.linear.x = 0.0 if self.b_is_stopped else 0.2
		t.twist.linear.y = 0.0
		t.twist.linear.z = 0.0
		t.twist.angular.x = 0.0
		t.twist.angular.y = 0.0
		t.twist.angular.z = 0.0

		# Publish the velocity command.
		self.pub.publish(t)

		# Print out a log message to the INFO channel to let us know it's working.
		self.get_logger().info(f'Shortest {shortest}, Published {t.twist.linear.x}, stop y/n {self.b_is_stopped}')


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	dumb_stopper = DumbStopper()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(dumb_stopper)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()
	

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()