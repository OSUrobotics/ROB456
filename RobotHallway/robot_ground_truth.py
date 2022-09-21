#!/usr/bin/env python3

import numpy as np


# This class keeps track of the "ground truth" of the robot's location
#  - where the robot actually is
#  - moves the robot in the direction asked, with some noise
#
# There are two types of movement; either discrete movement (i.e., from one bin to the next) or continuous movement
#   (move by some amount). The discrete motion is used for the bayes filter, the other two for the particle/kalman
#   filter movement.
#
# Homework instructions:
#   You'll be filling in the code to "add noise" to the robot's movement. There's two parts to this; the first is to
#     set up the dictionaries that hold the probabilities, the second is to sample from those probability distributions.
#   There are THREE probability distributions you need to set up:
#     move_left - what happens in the discrete/bin world when you move left one bin
#     move_right - same thing as move_left, but now you're moving right one bin
#     move_continuous - move by some distance (negative is left, positive is right)
#
#  move_left/move_right are versions of the discrete probability dictionaries you set up in probability assignment
#   (probability_sampling). This is used for the Bayes filter part of the assignment
#  move_continuous is a version of the gaussian sampling. This is used for the Kalman filter/particle filter part of
#   the assignment
#
class RobotGroundTruth:
    def __init__(self):
        # Where the robot actually is
        self.robot_loc = 0.5

        # Default move probabilities, stored as dictionaries (see probabilities homework)
        #    move_left and move_right are discrete probabilities, with three cases - move left, move right, or hold still
        #    move_continuous is a Gaussian - store the standard deviation
        self.move_probabilities = {"move_left": {}, "move_right": {}, "move_continuous": {}}

        # In the GUI version, these will be called with values from the GUI
        self.set_move_left_probabilities()
        self.set_move_right_probabilities()
        self.set_move_continuos_probabilities()

    def reset(self):
        """ Put robot in the middle"""
        self.robot_loc = 0.5

    def place_random(self):
        """ Put robot in a random location"""
        self.robot_loc = np.random.uniform()

    def set_move_left_probabilities(self, move_left=0.8, move_right=0.05):
        """ Set the three discrete probabilities for moving left (should sum to one and all be positive)
        @param move_left - the probability of actually moving left
        @param move_right - the probability of actually moving right """

        # Bayes assignment
        # Set self.move_probabilities["move_left"] dictionary
        # Check that the probabilities sum to one and are between 0 and 1

# YOUR CODE HERE

    def set_move_right_probabilities(self, move_left=0.05, move_right=0.8):
        """ Set the three discrete probabilities for moving right (should sum to one and all be positive)
        @param move_left - the probability of actually moving left
        @param move_right - the probability of actually moving right """

        # Bayes assignment
        # Set self.move_probabilities["move_right"] dictionary
        # Check that the probabilities sum to one and are between 0 and 1

# YOUR CODE HERE

    def set_move_continuos_probabilities(self, mu=0.0, sigma=0.1):
        """ Set the noise for continuous movement
        @param mu - mu of noise
        @param sigma - standard deviation of noise"""

        # Kalman assignment
        # Set self.move_probabilities["move_continuous"] dictionary
        # Check that sigma is positive

# YOUR CODE HERE

    # Just a helper function to place robot in middle of bin
    def _adjust_middle_of_bin(self, n_divs):
        """ Helper function to place robot in middle of bin
        @param n_divs - number of bins"""
        div = 1.0 / n_divs
        bin_id = min(n_divs-1, max(0, np.round(self.robot_loc / div)))
        self.robot_loc = (bin_id + 0.5) * div

    def _move_clamped_discrete(self, amount):
        """ Move the amount given - but clamp value between zero and one so don't fall out of hallway
        @param amount - requested amount to move
        @return amount - amount actually moved"""
        if 0 <= self.robot_loc + amount <= 1:
            self.robot_loc += amount
            return amount

        # At limit, either left or right, so don't move
        return 0.0

    def _move_clamped_continuous(self, amount):
        """ Move the amount given - but clamp value between zero and one so don't fall out of hallway
        @param amount - requested amount to move
        @return amount - amount actually moved"""
        if 0 <= self.robot_loc + amount <= 1:
            self.robot_loc += amount
            return amount

        if self.robot_loc + amount < 0:
            ret_amount = self.robot_loc
            self.robot_loc = 0.0
            return ret_amount

        ret_amount = (self.robot_loc + amount) - 1.0
        self.robot_loc = 1.0
        return ret_amount

    def move_left(self, step_size):
        """ Move to the next bin to the left (probably)
        @param step_size - the size of the step/distance between bins
        @return The amount actually moved """

        # Bayes assignment
        # Set step_dir to -1 (left), 0 (stay put) or 1 (right) based on sampling the move_left variable
        step_dir = 0

# YOUR CODE HERE

        return self._move_clamped_discrete(step_dir * step_size)

    def move_right(self, step_size):
        """ Move to the next bin to the right (probably)
        @param step_size - the size of the step/distance between bins
        @return The amount actually moved """

        # Bayes assignment
        # Set step_dir to -1 (left), 0 (stay put) or 1 (right) based on sampling the move_right variable
        step_dir = 0

# YOUR CODE HERE

        return self._move_clamped_discrete(step_dir * step_size)

    def move_continuous(self, amount):
        """ Move by the amount given (may be positive or negative) with noise added
        @param amount - amount to move (negative is left, positive right)
        @return the amount actually moved """

        # Kalman assignment
        # Set noisy_amount to be the amount to move, plus noise
        noisy_amount = amount

# YOUR CODE HERE

        # Actually move (don't run off of end)
        return self._move_clamped_continuous(noisy_amount)

def test_discrete_move_functions():
    """ Check that moving all the way left (or right) pushes the robot to the left (or right) """
    np.random.seed(5)

    rgt = RobotGroundTruth()
    n_bins = 10
    step_size = 1.0 / n_bins

    # Number of moves to try
    n_moves = 40

    # Move far enough to the left/right and you should stop moving
    count_left_walls = 0
    count_right_walls = 0
    # First time through the loop, move left. Second time through the loop, move right
    for mf, dir_move in zip([rgt.move_left, rgt.move_right], ["left", "right"]):
        print(f"Checking move_{dir_move} function")
        for i in range(0, n_moves):
            mf(step_size)
            if rgt.robot_loc < 0:
                raise ValueError(f"Robot went off end of left wall {rgt.robot_loc}")
            if rgt.robot_loc > 1:
                raise ValueError(f"Robot went off end of right wall {rgt.robot_loc}")
            if rgt.robot_loc < step_size:
                count_left_walls += 1
            if rgt.robot_loc > 1.0 - step_size:
                count_right_walls += 1

    # Should take 5 moves to get to left/right walls, plus some amount of moving off of wall randomly.
    #   So this is a reasonable check
    print(f"On left wall {count_left_walls}, on right wall {count_right_walls}, of {2 * n_moves} moves")
    if count_left_walls < n_moves // 4:
        raise ValueError(f"Failed: Potential problem of not reaching left wall {count_left_walls}")
    if count_right_walls < n_moves // 4:
        raise ValueError(f"Failed: Potential problem of not reaching right wall {count_right_walls}")

    # Check that we get our probabilities back (mostly)
    n_samples = 10000
    for mf, dir_move in zip([rgt.move_left, rgt.move_right], ["left", "right"]):
        print(f"Checking move {dir_move} probabilities")
        count_moved_left = 0
        count_moved_right = 0
        for i in range(0, n_samples):
            rgt.robot_loc = 0.5
            mf(step_size)
            if np.isclose(rgt.robot_loc, 0.5 - step_size):
                count_moved_left += 1
            elif np.isclose(rgt.robot_loc, 0.5 + step_size):
                count_moved_right += 1

        prob_count_left = count_moved_left / n_samples
        prob_count_right = count_moved_right / n_samples
        if not np.isclose(rgt.move_probabilities["move_" + dir_move]["left"], prob_count_left, atol=0.1):
            raise ValueError(f"Probability should be close to {rgt.move_probabilities['move_' + dir_move]['left']}, is {prob_count_left}")
        if not np.isclose(rgt.move_probabilities["move_" + dir_move]["right"], prob_count_right, atol=0.1):
            raise ValueError(f"Probability should be close to {rgt.move_probabilities['move_' + dir_move]['right']}, is {prob_count_right}")

    print("Passed discrete tests")
    return True


def test_continuous_move_functions():
    rgt = RobotGroundTruth()

    print("Checking move with normal distribution probabilities")
    dist_moved = []
    mu = 0.1
    sigma = 0.1
    n_samples = 10000
    rgt.set_move_continuos_probabilities(mu, sigma)
    for i in range(0, n_samples):
        rgt.robot_loc = 0.5

        dist_moved.append(rgt.move_continuous(0.0))

    mu_moved = np.mean(dist_moved)
    sigma_moved = np.std(dist_moved)
    if not np.isclose(mu_moved, mu, atol=0.01):
        raise ValueError(f"Mean should be close to {mu}, is {mu_moved}")
    if not np.isclose(sigma_moved, sigma, atol=0.01):
        raise ValueError(f"Mean should be close to {sigma}, is {sigma_moved}")

    print("Passed continuous tests")
    return True


if __name__ == '__main__':
    test_discrete_move_functions()
    test_continuous_move_functions()

    print("Done")
