#!/usr/bin/env python3


# Example of publishing in ROS 2
#
# publisher.py
#
# Bill Smart
#
# This is an example of a simple publisher in ROS 2.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# We're going to publish an Int64.  This is the same as ROS.
from std_msgs.msg import Int64


# The idiom in ROS2 is to encapsulate everything to do with the node in a class that
# is derived from the Node class in rclpy.node.
class BasicPublisher(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('publisher')

		# Create a publisher, and assign it to a member variable.  There is a more
		# commone interface to the creation of communication mechanisms, which is
		# part of the Node interface.  Since we derive from Node, we can get to these
		# through the self reference.  The call takes a type, topic name, and queue size.
		self.pub = self.create_publisher(Int64, 'counter', 10)

		# Rather than setting up a Rate-controller loop, the idiom in ROS2 is to use timers.
		# Timers are available in the Node interface, and take a period (in seconds), and a
		# callback.  Timers are repeating by default.
		self.timer = self.create_timer(1, self.timer_callback)

		# Set up a counter that we can increment.
		self.counter = 0

	# This callback will be called every time the timer fires.
	def timer_callback(self):
		# Make an Int64 message, and fill in the information.
		msg = Int64()
		msg.data = self.counter

		# Publish the message, just like we do in ROS.
		self.pub.publish(msg)

		# Log that we published something.  In ROS2, loggers are associated with nodes, and
		# the idiom is to use the get_logger() call to get the logger.  This has functions
		# for each of the logging levels.
		self.get_logger().info(f'Published {self.counter}')
		self.counter += 1


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	publisher = BasicPublisher()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(publisher)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()