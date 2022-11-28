#!/usr/bin/env python3


import sys
import rospy

from new_driver import Driver

from math import atan2, sqrt


class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self):
		super().__init__('odom')

	def get_twist(self, target, lidar):
		'''
		This function is called whenever there a current target is set and there is a lidar data
		available.  This is where you should put your code for moving the robot.  The target point
		is in the robot's coordinate frame.  The x-axis is positive-forwards, and the y-axis is
		positive to the left.

		The example sets constant velocities, which is clearly the wrong thing to do.  Replace this
		code with something that moves the robot more intelligently.

		Parameters:
			target:		The current target point, in the coordinate frame of the robot (base_link) as
						an (x, y) tuple.
			lidar:		A LaserScan containing the new lidar data.

		Returns:
			A Twist message, containing the commanded robot velocities.
		'''
		angle = atan2(target[1], target[0])
		distance = sqrt(target[0] ** 2 + target[1] **2)
		rospy.loginfo(f'Distance: {distance:.2f}, angle: {angle:.2f}')

		# This builds a Twist message with all elements set to zero.
		command = Driver.zero_twist()

		# Forwards velocity goes here, in meters per second.
		command.linear.x = 0.1

		# Rotational velocity goes here, in radians per second.  Positive is counter-clockwise.
		command.angular.z = 0.1

		return command


if __name__ == '__main__':
	rospy.init_node('student_driver', argv=sys.argv)

	driver = StudentDriver()

	rospy.spin()
