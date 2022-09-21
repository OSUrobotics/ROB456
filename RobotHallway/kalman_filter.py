#!/usr/bin/env python3

# This is the code you need to do the Kalman filter for the door localization

import numpy as np

from world_ground_truth import WorldGroundTruth
from robot_sensors import RobotSensors
from robot_ground_truth import RobotGroundTruth


# State estimation using Kalman filter
#   Stores the belief about the robot's location as a Gaussian
#     See the probability_sampling assignment, gaussian, for how to implement store probability as a Gaussian
class KalmanFilter:
    def __init__(self):
        # Kalman (Gaussian) probabilities
        self.mu = 0.5
        self.sigma = 0.4

        self.reset_kalman()

    # Put robot in the middle with a really broad standard deviation
    def reset_kalman(self):
        self.mu = 0.5
        self.sigma = 0.4

    # Sensor reading, distance to wall
    def update_gauss_sensor_reading(self, robot_sensors, dist_reading):
        """ Update state estimation based on sensor reading
        :param robot_sensors - for standard deviation of wall sensor
        :param dist_reading - distance reading returned"""

# YOUR CODE HERE
        return self.mu, self.sigma

    # Given a movement, update Gaussian
    def update_continuous_move(self, robot_ground_truth, amount):
        """ Kalman filter update mu/standard deviation with move (the prediction step)
        @param robot_ground_truth : robot state - has the standard deviation error for moving
        @param amount : The requested amount to move
        @return : mu and standard deviation of my current estimated location """

# YOUR CODE HERE
        return self.mu, self.sigma

