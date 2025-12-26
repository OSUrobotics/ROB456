#!/usr/bin/env python3

import numpy as np
from world_ground_truth import WorldGroundTruth
from robot_ground_truth import RobotGroundTruth


# This class handles the robot's sensors
#  - In front of door, y/n
#  - Distance to wall
class RobotSensors:
    def __init__(self):

        # Default sensor probabilities (see probabilities homework for how to store)
        #   - is if the robot is in front of the door, return True/False
        #   - distance_wall - returns a distance (with noise)
        # Bayes filter and Particle filter:
        #  GUIDE: Store and sample from random variables for the two door conditions (robot is in front of door or not)
        # Kalman filter and Particle filter:
        #  GUIDE: Store a Gaussian random variable for the distance to the wall sensor noise
        # Note: The actual values will be set in the calls to set_* below
        # Second note: all variables should be referenced with self. or they will disappear

        # GUIDE: Create the variable to store the probabilities
        # YOUR CODE HERE

        # In the GUI version, these will be called with values from the GUI after the RobotSensors instance
        #   has been created
        # Actually SET the values for the dictionaries
        self.set_door_sensor_probabilites()
        self.set_distance_wall_sensor_probabilities()

    def set_door_sensor_probabilites(self, in_prob_see_door_if_door:float=0.8, in_prob_see_door_if_not_door:float=0.1):
        """ Set the two door probabilities.
        @param in_prob_see_door_if_door - probability of seeing a door if there is one
        @param in_prob_see_door_if_not_door - probability of seeing a door if there is NOT one
        """
        # Bayes and particle filter assignment
        # GUIDE: Store the input values for the TWO random variables (one for the door there, one for no door)
        #  Reminder: You should have created the variable to hold these values in the __init__ method above
        #  Second note: all variables should be referenced with self.
        # YOUR CODE HERE

    def set_distance_wall_sensor_probabilities(self, sigma=0.1):
        """ Setup the wall sensor probabilities (store them in the dictionary)
        Note: Mean is zero for this assignment
        @param sigma - sigma of noise"""

        # Kalman and particle filter assignment
        # GUIDE: Store the Gaussian (reminder, mean for location is zero)
        # YOUR CODE HERE

    def query_door(self, robot_gt:RobotGroundTruth, world_gt:WorldGroundTruth):
        """ Query the door sensor
        @param robot_gt - ground truth of robot location is in here
        @param world_gt - ground truth of where the doors are
        @return True (thinks open) or False (thinks closed)
        """

        # Bayes assignment and particle filter
        # I've handled the checking if the robot is in front of the door (y/n) part for you
        # This is the ground truth - True if robot is actually in front of the door, False otherwise
        #  Use this to determine which random variable to use
        # is_in_front_of_door is a Boolean, which is True if the actual robot is in front of an actual door
        is_in_front_of_door = world_gt.is_location_in_front_of_door(robot_gt.robot_loc)

        # GUIDE:
        #  This is the place where you need a 4-way if statement
        #   First if statement: Is the robot in front of the door?
        # STEP 1 - generate a random number between 0 and 1
        # STEP 2 - use the random number (and your first if statement) to determine if you should return True or False
        # Note: Step 2 is just the sample_boolean code from your probabilities assignment
        
        # YOUR CODE HERE

    def query_distance_to_wall(self, robot_gt: RobotGroundTruth):
        """ Return a distance reading (with correct noise) of the robot's location
        This returns the estimated robot's location by measuring the distance to the left/right walls - i.e., it
        is a direct measure of the robot's location
        @param robot_gt - ground truth of robot location is in here
        @return distance + noise (float) """

        # Kalman assignment and particle filter assignment
        # GUIDE: Return the distance to the wall (with noise)
        #  This is the Gaussian assignment from your probabilities homework
        # YOUR CODE HERE


def test_discrete_sensors(b_print=True):
    """ Test that the door sensor is working correctly
    @param b_print - do print statements, yes/no"""
    np.random.seed(3)

    robot_gt = RobotGroundTruth()
    world_gt = WorldGroundTruth()
    robot_sensor = RobotSensors()

    probs_see_door = (0.7, 0.2)
    robot_sensor.set_door_sensor_probabilites(probs_see_door[0], probs_see_door[1])
    n_samples = 1000

    # Test if the robot is actually in front of the door versus if it is NOT in front of the door
    #   Doing this as a for loop to avoid duplicating the counting code. p will have the probability of returning
    #   True, given the first case (robot is in front of door), versus the second case (not in front of the door)
    for loc, p in zip((world_gt._location_in_front_of_door(), world_gt._location_not_in_front_of_door()), probs_see_door):
        robot_gt.robot_loc = loc

        is_in_front_of_door_gt = world_gt.is_location_in_front_of_door(robot_gt.robot_loc)
        if b_print:
            print(f"Testing case for robot in front of door: {is_in_front_of_door_gt}")

        # Check that we get our probabilities back (mostly)
        count_returned_true = 0
        for i in range(0, n_samples):
            if robot_sensor.query_door(robot_gt=robot_gt, world_gt=world_gt):
                count_returned_true += 1

        prob_count_true = count_returned_true / n_samples
        # If the robot is in front of the door, then we get the first set of probabilities - the robot is in
        #  front of the door and we should return True 70% of the time
        # If the robot is NOT in front of the door, then we get the second set of probabilities - the robot is NOT in
        #  front of the door and we should return True 20% of the time
        if not np.isclose(prob_count_true, p, atol=0.1):
            raise ValueError(f"Probability should be close to {p:0.4f}, is {prob_count_true:0.4f}, robot loc {is_in_front_of_door_gt}")

    if b_print:
        print("Passed tests")
    return True


def test_continuous_sensor(b_print=True):
    """ Test that the distance to wall sensor is working properly
    @param b_print - do print statements, yes/no"""
    if b_print:
        print("Checking query wall with normal distribution probabilities")

    robot_gt = RobotGroundTruth()
    robot_sensor = RobotSensors()

    sigma = 0.01

    # Doing this as a for loop with pre-allocating the data (np.zeros instead of a list with an append) because
    #   we are going to place the robot randomly at each iteration, then measure the distance the sensor returns,
    #   rather than keeping the robot in one place all the time
    # This is the exactly how one would measure sensor noise in the first place, btw
    n_samples = 10000
    dist_measured = np.zeros(n_samples)
    robot_sensor.set_distance_wall_sensor_probabilities(sigma)
    for i in range(0, n_samples):
        # Put the robot in a random location
        robot_gt.place_random()

        # How much is the measurement off by?
        dist_measured[i] = robot_sensor.query_distance_to_wall(robot_gt) - robot_gt.robot_loc

    # What is the mu and sigma of the sensor noise?
    mu_query = np.mean(dist_measured)
    sigma_query = np.std(dist_measured)
    if not np.isclose(mu_query, 0.0, atol=0.01):
        raise ValueError(f"Mean should be close to 0.0, is {mu_query}")
    if not np.isclose(sigma_query, sigma, atol=0.01):
        raise ValueError(f"Mean should be close to {sigma}, is {sigma_query}")

    if b_print:
        print("Passed continuous tests")
    return True


if __name__ == '__main__':
    b_print = True

    # ----------------------- Bayes filter -----------------
    # Syntax check
    robot_gt = RobotGroundTruth()
    world_gt = WorldGroundTruth()
    robot_sensor = RobotSensors()

    probs_see_door = (0.7, 0.2)
    robot_sensor.set_door_sensor_probabilites(probs_see_door[0], probs_see_door[1])
    ret_value = robot_sensor.query_door(robot_gt, world_gt)
    if ret_value is True or ret_value is False:
        print("Passed robot sensor syntax check")

    test_discrete_sensors(b_print)

    # ----------------------- Kalman filter -----------------
    # Syntax check
    robot_gt = RobotGroundTruth()
    robot_sensor = RobotSensors()
    robot_sensor.set_distance_wall_sensor_probabilities(sigma=0.01)
    dist_with_noise = robot_sensor.query_distance_to_wall(robot_gt)
    if 0.0 < dist_with_noise < 1.0:
        print("Dist wall sensor, passed syntax test")

    test_continuous_sensor(b_print)
