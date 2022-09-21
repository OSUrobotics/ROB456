#!/usr/bin/env python3

import numpy as np

from RobotHallway.world_ground_truth import WorldGroundTruth
from RobotHallway.door_sensor import DoorSensor


# Belief about world/robot state
class RobotStateEstimation:
    def __init__(self):

        # Probability representation (discrete)
        self.probabilities = []
        self.reset_probabilities(10)

        # Kalman (Gaussian) probabilities
        self.mean = 0.5
        self.standard_deviation = 0.4
        self.reset_kalman()

    def reset_probabilities(self, n_probability):
        """ Initialize discrete probability resolution with uniform distribution """
        div = 1.0 / n_probability
        self.probabilities = np.ones(n_probability) * div

    def update_belief_sensor_reading(self, ws, ds, sensor_reading_has_door):
        """ Update your probabilities based on the sensor reading being true (door) or false (no door)
        :param ws World state - has where the doors are
        :param ds Door Sensor - has probabilities for door sensor readings
        :param sensor_reading_has_door - contains true/false from door sensor
        """
        new_probs = np.zeros(len(self.probabilities))
        # begin homework 2 : problem 3
        div = 1.0 / len(self.probabilities)
        for i, p in enumerate(self.probabilities):
            x_pos = (i+0.5) * div
            x_pos_has_door = ws.is_in_front_of_door(x_pos)

            if x_pos_has_door:
                if sensor_reading_has_door:
                    new_probs[i] = ds.prob_see_door_if_door * p
                else:
                    new_probs[i] = (1-ds.prob_see_door_if_door) * p
            else:
                if sensor_reading_has_door:
                    new_probs[i] = ds.prob_see_door_if_no_door * p
                else:
                    new_probs[i] = (1 - ds.prob_see_door_if_no_door) * p

        # Normalize - all the denominators are the same because they're the sum of all cases
        prob_sum = sum(new_probs)
        for i in range(0, len(self.probabilities)):
            self.probabilities[i] = new_probs[i] / prob_sum
        # end homework 2 : problem 3

    # Distance to wall sensor (state estimation)
    def update_dist_sensor(self, ws, dist_reading):
        """ Update state estimation based on sensor reading
        :param ws - for standard deviation of wall sensor
        :param dist_reading - distance reading returned from the sensor, in range 0,1 (essentially, robot location) """

        # Standard deviation of error
        standard_deviation = ws.wall_standard_deviation
        # begin homework 2 : Extra credit
        div = 1.0 / len(self.probabilities)
        new_probs = np.zeros(len(self.probabilities))

        for i, p in enumerate(self.probabilities):
            x_pos = (i+0.5) * div
            # Sample from probability
            dy = np.exp(-np.power(dist_reading - x_pos, 2.0) / (2 * np.power(standard_deviation, 2.0)))

            new_probs[i] = dy * p

        # Normalize - all the denominators are the same
        prob_sum = sum(new_probs)
        for i in range(0, len(self.probabilities)):
            self.probabilities[i] = new_probs[i] / prob_sum
        # end homework 2 : Extra credit
        return self.mean, self.standard_deviation

    def update_belief_move_left(self, rs):
        """ Update the probabilities assuming a move left.
        :param rs - robot state, has the probabilities"""

        # begin homework 2 problem 4
        new_probs = np.zeros(len(self.probabilities))

        # Check probability of left, no, right sum to one
        sum_probs = rs.prob_move_left_if_left + rs.prob_no_move_if_left + rs.prob_move_right_if_left
        if abs(sum_probs - 1) > 0.0001:
            raise ValueError("Move left, values should sum to 1 {}".format(sum_probs))

        # Left edge - put move left probability into zero square along with stay-put probability
        new_probs[0] += self.probabilities[0] * (rs.prob_move_left_if_left +
                                                 rs.prob_no_move_if_left)
        new_probs[1] += self.probabilities[0] * rs.prob_move_right_if_left

        for i, p in zip(range(1, len(new_probs)-1), self.probabilities[1:-1]):
            new_probs[i-1] += p * rs.prob_move_left_if_left
            new_probs[i] += p * rs.prob_no_move_if_left
            new_probs[i+1] += p * rs.prob_move_right_if_left

        # Right edge - put move right probability into last square
        new_probs[-2] += self.probabilities[-1] * rs.prob_move_left_if_left
        new_probs[-1] += self.probabilities[-1] * rs.prob_no_move_if_left
        new_probs[-1] += self.probabilities[-1] * rs.prob_move_right_if_left

        # Normalize - sum should be one, except for numerical rounding
        prob_sum = sum(new_probs)
        for i in range(0, len(self.probabilities)):
            self.probabilities[i] = new_probs[i] / prob_sum
        if abs(prob_sum - 1) > 0.0001:
            raise ValueError("Sum should be 1, was {}".format(prob_sum))
        # end homework 2 problem 4

    def update_belief_move_right(self, rs):
        """ Update the probabilities assuming a move right.
        :param rs - robot state, has the probabilities"""

        # begin homework 2 problem 4
        new_probs = np.zeros(len(self.probabilities))

        # Check probability of left, no, right sum to one
        sum_probs = rs.prob_move_left_if_right + rs.prob_no_move_if_right + rs.prob_move_right_if_right
        if abs(sum_probs - 1) > 0.0001:
            raise ValueError("Move right, values should sum to 1 {}".format(sum_probs))

        # Left edge - put move left probability into zero square along with stay-put probability
        new_probs[0] += self.probabilities[0] * (rs.prob_move_left_if_right +
                                                 rs.prob_no_move_if_right)
        new_probs[1] += self.probabilities[0] * rs.prob_move_right_if_right

        for i, p in zip(range(1, len(new_probs)-1), self.probabilities[1:-1]):
            new_probs[i-1] += p * rs.prob_move_left_if_right
            new_probs[i] += p * rs.prob_no_move_if_right
            new_probs[i+1] += p * rs.prob_move_right_if_right

        # Right edge - put move right probability into last square
        new_probs[-2] += self.probabilities[-1] * rs.prob_move_left_if_right
        new_probs[-1] += self.probabilities[-1] * rs.prob_no_move_if_right
        new_probs[-1] += self.probabilities[-1] * rs.prob_move_right_if_right

        # Normalize - sum should be one, except for numerical rounding
        prob_sum = sum(new_probs)
        for i in range(0, len(self.probabilities)):
            self.probabilities[i] = new_probs[i] / prob_sum
        if abs(prob_sum - 1) > 0.0001:
            raise ValueError("Sum should be 1, was {}".format(prob_sum))
        # end homework 2 problem 4

    # Put robot in the middle with a really broad standard deviation
    def reset_kalman(self):
        self.mean = 0.5
        self.standard_deviation = 0.4

    # Given a movement, update Gaussian
    def update_kalman_move(self, rs, amount):
        """ Kalman filter update mu/standard deviation with move (the prediction step)
        :param rs : robot state - has the standard deviation error for moving
        :param amount : The requested amount to move
        :return : mu and standard deviation of my current estimated location """

        # begin homework 3 : Problem 2
        self.mean = self.mean + amount
        self.standard_deviation = self.standard_deviation + rs.robot_move_standard_deviation_err
        # end homework 3 : Problem 2
        return self.mean, self.standard_deviation

    # Sensor reading, distance to wall (Kalman filtering)
    def update_gauss_sensor_reading(self, ws, dist_reading):
        """ Update state estimation based on sensor reading
        :param ws - for standard deviation of wall sensor
        :param dist_reading - distance reading returned"""

        # begin homework 3 : Problem 1
        K = self.standard_deviation / (self.standard_deviation + ws.wall_standard_deviation)
        self.mean = self.mean + K * (dist_reading - self.mean)
        self.standard_deviation = (1-K) * self.standard_deviation
        # end homework 3 : Problem 1
        return self.mean, self.standard_deviation


if __name__ == '__main__':
    ws_global = WorldGroundTruth()

    ds_global = DoorSensor()

    rse_global = RobotStateEstimation()

    # Check out these cases
    # We have two possibilities - either in front of door, or not - cross two sensor readings
    #   saw door versus not saw door
    uniform_prob = rse_global.probabilities[0]

    # begin homework 2 problem 4
    # Four cases - based on default door probabilities of
    # DoorSensor.prob_see_door_if_door = 0.8
    # DoorSensor.prob_see_door_if_no_door = 0.2
    #  and 10 probability divisions. Three doors visible.

    # probability saw door if door, saw door if no door, etc
    check_prob_saw_door_door = ds_global.prob_see_door_if_door
    check_prob_saw_door_no_door = ds_global.prob_see_door_if_no_door

    check_prob_no_saw_door_door = (1-ds_global.prob_see_door_if_door)
    check_prob_no_saw_door_no_door = (1-ds_global.prob_see_door_if_no_door)

    # Resulting probabilities, assuming 3 doors
    denom_saw_door = 3 * check_prob_saw_door_door + 7 * check_prob_saw_door_no_door
    denom_no_saw_door = 3 * check_prob_no_saw_door_door + 7 * check_prob_no_saw_door_no_door

    check_prob_saw_door_door /= denom_saw_door
    check_prob_saw_door_no_door /= denom_saw_door
    check_prob_no_saw_door_door /= denom_no_saw_door
    check_prob_no_saw_door_no_door /= denom_no_saw_door

    # Check that our probabilities are updated correctly
    n_bins = 10
    rse_global.reset_probabilities(n_bins)
    # Spacing of bins
    div_bins = 1.0 / n_bins
    rse_global.update_belief_sensor_reading(ws_global, ds_global, True)
    for i_bin, prob in enumerate(rse_global.probabilities):
        if ws_global.is_in_front_of_door((i_bin+0.5) * div_bins):
            if abs(prob - check_prob_saw_door_door) > 1e-6:
                raise ValueError("TT New probability should be {}, was{}", check_prob_saw_door_door, prob)
        else:
            if abs(prob - check_prob_saw_door_no_door) > 1e-6:
                raise ValueError("FT New probability should be {}, was{}", check_prob_saw_door_no_door, prob)

    rse_global.reset_probabilities(n_bins)
    rse_global.update_belief_sensor_reading(ws_global, ds_global, False)
    for i_bin, prob in enumerate(rse_global.probabilities):
        if ws_global.is_in_front_of_door((i_bin+0.5) * div_bins):
            if abs(prob - check_prob_no_saw_door_door) > 1e-6:
                raise ValueError("TF New probability should be {}, was{}", check_prob_no_saw_door_door, prob)
        else:
            if abs(prob - check_prob_no_saw_door_no_door) > 1e-6:
                raise ValueError("FF New probability should be {}, was{}", check_prob_no_saw_door_no_door, prob)

    # end homework 2 problem 4

    print("Passed tests")
