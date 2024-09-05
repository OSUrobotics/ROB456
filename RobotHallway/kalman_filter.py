#!/usr/bin/env python3

# This is the code you need to do the Kalman filter for the door localization

import numpy as np

from robot_sensors import RobotSensors
from robot_ground_truth import RobotGroundTruth


# State estimation using Kalman filter
#   Stores the belief about the robot's location as a Gaussian
#     See the probability_sampling assignment, gaussian, for how to implement store probability as a Gaussian
# Slides for this assignment: https://docs.google.com/presentation/d/1Q6w-vczvWHanGDqbuz6H1qhTOkSrX54kf1g8NgTcipQ/edit?usp=sharing
class KalmanFilter:
    def __init__(self):
        # Kalman (Gaussian) probabilities
        self.mu = 0.5
        self.sigma = 0.4

        self.reset_kalman()

    # Put robot in the middle with a really broad standard deviation
    #  NOTE - don't change this
    def reset_kalman(self):
        self.mu = 0.5
        self.sigma = 0.4

    # Sensor reading, distance to wall
    def update_belief_distance_sensor(self, robot_sensors, dist_reading):
        """ Update state estimation based on sensor reading
        Lec 3.1 Kalman filters
        Slides: https://docs.google.com/presentation/d/1a9FGeAQKxtAlIeDMykePcmJYF1wBNOwSUg-ts4L0N_U/edit#slide=id.p23
          Steps 5-8 (correction steps)
        @param robot_sensors - for mu/sigma of wall sensor
        @param dist_reading - distance reading returned by sensor"""

        # TODO: Calculate C and K, then update self.mu and self.sigma
        # YOUR CODE HERE
        return self.mu, self.sigma

    # Given a movement, update Gaussian
    def update_continuous_move(self, robot_ground_truth, amount):
        """ Kalman filter update mu/standard deviation with move (the prediction step)
        Lec 3.1 Kalman filters
        Slides: https://docs.google.com/presentation/d/1a9FGeAQKxtAlIeDMykePcmJYF1wBNOwSUg-ts4L0N_U/edit#slide=id.p23
          Steps 2-3 (prediction steps)
        @param robot_ground_truth : robot state - has the mu/sigma for moving
        @param amount : The control signal (the amount the robot was requested to move
        @return : mu and sigma of new current estimated location """

        # TODO: Update mu and sigma by Ax + Bu equation
        # YOUR CODE HERE
        return self.mu, self.sigma

    def one_full_update(self, robot_ground_truth, robot_sensor, u: float, z: float):
        """This is the full update loop that takes in one action, followed by a sensor reading
        Lec 3.1 Kalman filters
        Slides: https://docs.google.com/presentation/d/1a9FGeAQKxtAlIeDMykePcmJYF1wBNOwSUg-ts4L0N_U/edit#slide=id.p23
          Prediction followed by zensor reading
        Assumes the robot has been moved by the action u, then a sensor reading was taken (see test function below)
        @
        @param robot_sensor - has the robot sensor probabilities
        @param robot_ground_truth - robot location, has the probabilities for actually moving left if move_left called
        @param u will be the amount moved
        @param z will be the wall distance sensor reading
        """
        # TODO:
        #  Step 1 predict: update your belief by the action (move the Gaussian)
        #  Step 2 correct: do the correction step (move the Gaussian to be between the current mean and the sensor reading)
        # YOUR CODE HERE




def test_kalman_update(b_print=True):
    """ Check against the saved results
    @param b_print - print the results, y/n
    Beware that this requires only calling random.uniform when doing the sensor/move - any additional
     calls will throw the random numbers off"""

    if b_print:
        print("Testing Kalman")
    # Generate some move sequences and compare to the correct answer
    import json
    with open("Data/check_kalman_filter.json", "r") as f:
        answers = json.load(f)

    kalman_filter = KalmanFilter()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Set mu/sigmas
    robot_ground_truth.set_move_continuos_probabilities(answers["move_error"]["sigma"])
    robot_sensor.set_distance_wall_sensor_probabilities(answers["sensor_noise"]["sigma"])

    # This SHOULD insure that you get the same answer as the solutions, provided you're only calling uniform within
    #  robot_ground_truth_syntax.move*
    np.random.seed(3)

    # Try different sequences
    for seq in answers["results"]:
        # Reset to uniform
        kalman_filter.reset_kalman()
        robot_ground_truth.reset_location()
        for i, s in enumerate(seq["seq"]):
            if s == "Sensor":
                dist = robot_sensor.query_distance_to_wall(robot_ground_truth)
                kalman_filter.update_belief_distance_sensor(robot_sensor, dist)
                if not np.isclose(dist, seq["sensor_reading"][i]):
                    print(f"Warning, sensor reading should be {seq['sensor_reading'][i]}, got {dist}")
            elif s == "Move":
                actual_move = robot_ground_truth.move_continuous(seq['move_amount'][i])
                kalman_filter.update_continuous_move(robot_ground_truth, seq['move_amount'][i])
                if not np.isclose(actual_move, seq["actual_move"][i]):
                    print(f"Warning, move should be {seq['actual_move'][i]}, got {actual_move}")
            elif s == "Both":
                actual_move = robot_ground_truth.move_continuous(seq['move_amount'][i])
                dist = robot_sensor.query_distance_to_wall(robot_ground_truth)
                kalman_filter.one_full_update(robot_ground_truth, robot_sensor, seq['move_amount'][i], dist)
                if not np.isclose(actual_move, seq["actual_move"][i]):
                    print(f"Warning, move should be {seq['actual_move'][i]}, got {actual_move}")
                if not np.isclose(dist, seq["sensor_reading"][i]):
                    print(f"Warning, sensor reading should be {seq['sensor_reading'][i]}, got {dist}")
            else:
                raise ValueError(f"Should be one of Move, Sensor, or Both, got {s}")

        if not np.isclose(seq["mu"], kalman_filter.mu, atol=0.01):
            raise ValueError(f"Failed sequence {seq['seq']}, got mu {kalman_filter.mu}, expected {seq['mu']}")
        if not np.isclose(seq["sigma"], kalman_filter.sigma, atol=0.01):
            raise ValueError(f"Failed sequence {seq['seq']}, got sigma {kalman_filter.sigma}, expected {seq['sigma']}")

    if b_print:
        print("Passed")
    return True


if __name__ == '__main__':

    # Syntax checks
    kalman_filter_syntax = KalmanFilter()
    robot_ground_truth_syntax = RobotGroundTruth()
    robot_sensor_syntax = RobotSensors()

    # Set mu/sigmas
    sensor_noise_syntax = {"mu": 0.0, "sigma": 0.1}
    move_error_syntax = {"mu": 0.0, "sigma": 0.05}
    robot_ground_truth_syntax.set_move_continuos_probabilities(move_error_syntax["sigma"])
    robot_sensor_syntax.set_distance_wall_sensor_probabilities(sensor_noise_syntax["sigma"])

    kalman_filter_syntax.update_belief_distance_sensor(robot_sensor_syntax, 0.1)
    kalman_filter_syntax.update_continuous_move(robot_ground_truth_syntax, 0.0)

    # Do both at the same time
    u_syntax = 0.1
    robot_ground_truth_syntax.move_continuous(u_syntax)
    z_syntax = robot_sensor_syntax.query_distance_to_wall(robot_ground_truth_syntax)
    kalman_filter_syntax.one_full_update(robot_ground_truth_syntax, robot_sensor_syntax, u_syntax, z_syntax)

    # Generate some move sequences and compare to the correct answer
    test_kalman_update()

    print("Done")
