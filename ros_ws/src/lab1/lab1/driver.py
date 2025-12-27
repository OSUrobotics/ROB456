#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# driver.py
# This example gives the basic code for driving a robot around.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node


# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import TwistStamped


class BasicDriver(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('driver')

		# Set up a publisher.  The default topic for Twist messages is cmd_vel.
		self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

		# Rather than setting up a Rate-controller loop, the idiom in ROS2 is to use timers.
		# Timers are available in the Node interface, and take a period (in seconds), and a
		# callback.  Timers are repeating by default.
		# Generating new twist commands at 10Hz
		self.timer = self.create_timer(0.1, self.timer_callback)

	# This callback will be called every time the timer fires.
	def timer_callback(self):
		# Create a Twist and fill in the information.  Note that we fill in values
		# even for the elements we're not going to use.  We don't have to do this,
		# but it's good practice.
		t = TwistStamped()
		t.header.frame_id = 'odom'
		t.header.stamp = self.get_clock().now().to_msg()
		t.twist.linear.x = 0.2
		t.twist.linear.y = 0.0
		t.twist.linear.z = 0.0
		t.twist.angular.x = 0.0
		t.twist.angular.y = 0.0
		t.twist.angular.z = 0.0

		# Publish the velocity command.
		self.pub.publish(t)

		# Print out a log message to the INFO channel to let us know it's working.
		self.get_logger().info(f'Published {t.twist.linear.x}')


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	driver = BasicDriver()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(driver)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()