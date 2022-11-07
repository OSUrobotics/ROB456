#!/usr/bin/env python3

# Publish a point that moves in a circle, so that we can visualize it with rviz.
#
# circle.py
#
# Bill Smart, smartw@oregonstate.edu


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to need to do some math
from math import sin, cos

# This time we're going to be using PointStamped, which we get from geometry_msgs.
from geometry_msgs.msg import PointStamped


if __name__ == '__main__':
	# Initialize the node, and call it "circler".
	rospy.init_node('circler', argv=sys.argv)

	# Set up a publisher.  This will publish on a topic called "dot", with a
	# message type of Point.
	publisher = rospy.Publisher('dot', PointStamped, queue_size=10)

	# We're going to move the point around a circle.  To do this, we're going
	# to keep track of how far around the circle it is with an angle.  We're also
	# going to define how far it moves in each step.
	theta = 0.0
	theta_inc = 0.05

	# Rate allows us to control the (approximate) rate at which we publish things.
	# For this example, we want to publish at 10Hz.
	rate = rospy.Rate(10)

	# This will loop until ROS shuts down the node.  This can be done on the
	# command line with a ctrl-C, or automatically from roslaunch.
	while not rospy.is_shutdown():
		# Make a point instance, and fill in the information.
		p = PointStamped()
		p.header.stamp = rospy.Time.now()
		p.header.frame_id = 'map'
		p.point.x = cos(theta)
		p.point.y = sin(theta)
		p.point.z = 0.0

		# Publish the value of the counter.
		publisher.publish(p)

		# Print out a log message to the INFO channel to let us know it's working.
		rospy.loginfo(f'Published point at ({p.point.x}, {p.point.y})')

		# Increment theta.  This will grow without bound, which is bad if we run
		# the node for long enough, but we're not going to worry about it for this
		# toy example.
		theta += theta_inc

		# Do an idle wait to control the publish rate.  If we don't control the
		# rate, the node will publish as fast as it can, and consume all of the
		# available CPU resources.  This will also add a lot of network traffic,
		# possibly slowing down other things.
		rate.sleep()