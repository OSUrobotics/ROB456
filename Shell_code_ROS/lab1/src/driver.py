#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the basic code for driving a robot around.


# Import ROS Python basic API and sys
import rospy
import sys


# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('driver', argv=sys.argv)

	# Set up a publisher.  The default topic for Twist messages is cmd_vel.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# 10 Hz is a good rate to control a slow-moving robot.
	rate = rospy.Rate(10)

	# This will loop until ROS shuts down the node.  This can be done on the
	# command line with a ctrl-C, or automatically from roslaunch.
	while not rospy.is_shutdown():
		# Create a Twist and fill in the information.  Note that we fill in values
		# even for the elements we're not going to use.  We don't have to do this,
		# but it's good practice.
		t = Twist()
		t.linear.x = 0.2
		t.linear.y = 0.0
		t.linear.z = 0.0
		t.angular.x = 0.0
		t.angular.y = 0.0
		t.angular.z = 0.0

		# Publish the velocity command.
		publisher.publish(t)

		# Print out a log message to the INFO channel to let us know it's working.
		rospy.loginfo(f'Published {t.linear.x}')

		# Make sure we're publishing a tthe right rate.
		rate.sleep()