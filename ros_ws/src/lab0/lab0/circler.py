#!/usr/bin/env python3

# Publish a point that moves in a circle, so that we can visualize it with rviz.
#
# circler.py
# Bill Smart, smartw@oregonstate.edu


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# We're going to need to do some math
from math import sin, cos

# This time we're going to be using PointStamped, which we get from geometry_msgs.
from geometry_msgs.msg import PointStamped


class BasicCircler(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('circler')

		# Set up a publisher.  This will publish on a topic called "dot", with a
		# message type of Point.
		self.pub = self.create_publisher(PointStamped, 'dot', 10)

		# We're going to move the point around a circle.  To do this, we're going
		# to keep track of how far around the circle it is with an angle.  We're also
		# going to define how far it moves in each step.
		self.theta = 0.0
		self.theta_inc = 0.05


		# Rather than setting up a Rate-controller loop, the idiom in ROS2 is to use timers.
		# Timers are available in the Node interface, and take a period (in seconds), and a
		# callback.  Timers are repeating by default.
		# Sampling at 10Hz
		self.timer = self.create_timer(0.1, self.timer_callback)

	# This callback will be called every time the timer fires.
	def timer_callback(self):
		# Make a point instance, and fill in the information.
		p = PointStamped()
		p.header.stamp = self.get_clock().now().to_msg()
		p.header.frame_id = 'map'
		p.point.x = cos(self.theta)
		p.point.y = sin(self.theta)
		p.point.z = 0.0

		# Publish the point, just like we do in ROS.
		self.pub.publish(p)

		# Log that we published something.  In ROS2, loggers are associated with nodes, and
		# the idiom is to use the get_logger() call to get the logger.  This has functions
		# for each of the logging levels.
		self.get_logger().info(f'Published point at ({p.point.x}, {p.point.y})')

		# Increment theta.  This will grow without bound, which is bad if we run
		# the node for long enough, but we're not going to worry about it for this
		# toy example.
		self.theta += self.theta_inc


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	circler = BasicCircler()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(circler)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()