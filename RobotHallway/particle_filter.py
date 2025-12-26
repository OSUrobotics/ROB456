#!/usr/bin/env python3

# This is just the code you need to do the particle filter for the door localization

import numpy as np

from world_ground_truth import WorldGroundTruth
from robot_sensors import RobotSensors
from robot_ground_truth import RobotGroundTruth


# State estimation using a particle filter
#   Stores the belief about the robot's location as a set of samples of the state
# Slides for this assignment: https://docs.google.com/presentation/d/1GVCAWUSUhJiHP6VBi-HusDyZmunQG1mnVlopLhtRGLs/edit?usp=sharing
class ParticleFilter:
    def __init__(self):

        # GUIDE: Probability representation
        #   Create variable(s) to store the particles and the weights
        #.  Make sure to update get_number_particles and get_particle_weight
        # YOUR CODE HERE

        # Note that in the GUI version, this will be called with the desired number of samples to save
        self.reset_particles()

    def get_number_particles(self):
        """ Return the number of particles
        @return number of particles (int)"""
        # GUIDE: Return number of particles (int)
        # YOUR CODE HERE

    def get_particle_weight(self, indx_particle : int):
        """ Return the weight of the particle at index
        @param indx_particle - index of the particle
        @return weight of particle (float) """
        # GUIDE: Return weight for the given particle
        # YOUR CODE HERE
    
    def get_particle_location(self, indx_particle : int):
        """ Return the x location of the particle
        @param indx_particle - index of the particle
        @return location of particle (float) """
        # GUILDE: Return the location stored in particle indx_particle
        # YOUR CODE HERE

    def reset_particles(self, n_samples=1000):
        """ Initialize particle filter with uniform samples and weights
        @param n_samples - the number of state samples to keep """

        # GUIDE
        #  Step 1: create n_samples of the state space, uniformly distributed
        #  Step 2: create n_samples of uniform weights
        # YOUR CODE HERE

    def update_particles_move_continuous(self, robot_ground_truth, amount):
        """ Update state estimation based on sensor reading
        See Assignment slides for a pointer to the lecture notes
        @param robot_sensors - for mu/sigma of wall sensor
        @param dist_reading - distance reading returned by sensor"""

        # GUIDE
        #   For each particle, move it by the given amount PLUS some noise, drawn from the robot_ground_truth_syntax noise model
        #       If you don't add noise, you will quickly have all of the particles at the same location..
        #   If it runs into a wall, offset it from the wall by a (small) random amount
        # YOUR CODE HERE
        # print(f"CL {count_off_left_wall} CR {count_off_right_wall}")

    def calculate_weights_door_sensor_reading(self, world_ground_truth, robot_sensor, sensor_reading):
        """ Update your weights based on the sensor reading being true (door) or false (no door)
        Lec 4.1 particle filters
        Slide https://docs.google.com/presentation/d/1yddr6QwnUNHfW4GqLkC5Ds6tk8ezO56-WI2-B4ninWU/edit#slide=id.p11
        The weight calculation - line 5
        @param world_ground_truth - has where the doors actually are
        @param robot_sensor - has the robot sensor probabilities
        @param sensor_reading - the actual sensor reading - either True or False
        """

        # GUIDE
        #  You'll need a for loop to loop over the particles
        #  For each particle, calculate an importance weight using p(y|x) p(x) (the numerator of the Bayes' sensor update)
        #     p(x) is the probability of being at the point x for this sample (so... what is this value?)
        #     p(y|x) is the probability of the sensor returning T/F given the location x
        #     The location of each particle... is the value stored in the particle.
        # You might find enumerate useful
        #  for i, p in enumerate(self.particles):
        #      w[i] =
        #
        # Note that the particles themselves shouldn't change.
        # Note that w, p = zip(self.weights, self.particles):
        #    w = 3
        # will NOT set the weight in self.weights to the value to 3

        # YOUR CODE HERE

    def calculate_weights_distance_wall(self, robot_sensors, dist_reading):
        """ Calculate weights based on distance reading
        @param robot_sensors - for mu/sigma of wall sensor
        @param dist_reading - distance reading returned by sensor"""

        # GUIDE
        #  See calculate_weights above - only this time, set the weight for the particle based on how likely it
        #   was that the distance sensor was correct, given the location in the particle.

        # Yes, you can put function definitions in functions.
        def gaussian(x, mu, sigma):
            """Gaussian with given mu, sigma
            @param x - the input x value
            @param mu - the mu
            @param sigma - the standard deviation
            @return y = gauss(x) """
            return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(- (x - mu) ** 2 / (2 * sigma ** 2))

        # YOUR CODE HERE

    def resample_particles(self):
        """ Importance sampling - take the current set of particles and weights and make a new set
        """

        # GUIDE:
        #   Part 1: make a new numpy array that is a running sum of the weights (normalized)
        #       This is to speed up the computation
        #   Part 2: for n_samples (current number of particles) grab one of the particles based on the weights
        #       Like discrete_sampling, only with n_particles
        #       Generate a value between 0 and 1, use that to pick one of the particles
        #       Put that particle in the new list
        #         Note that np.where can be used to substantially speed up finding which particle
        #   Part 3: Set the weights back to uniform (just to be neat and clean)
        # YOUR CODE HERE

    def one_full_update_door(self, world_ground_truth, robot_ground_truth, robot_sensor, u: float, z: bool):
        """This is the full update loop that takes in one action, followed by a door sensor reading
        See Assignment slides for links to the lecture slides
        Assumes the robot has been moved by the amount u, then a door sensor reading was taken
        ONLY door sensor

        @param world_ground_truth - has where the doors actually are
        @param robot_sensor - has the robot sensor probabilities
        @param robot_ground_truth - robot location, has the probabilities for actually moving left if move_left called
        @param u will be the amount moved
        @param z will be one of True or False (door y/n)
        """
        # GUIDE:
        #  Step 1 Move the particles (with moise added)
        #  Step 2 Calculate the weights using the door sensor return value
        #  Step 3 Resample/importance weight
        # YOUR CODE HERE

    def one_full_update_distance(self, robot_ground_truth, robot_sensor, u: float, z: float):
        """This is the full update loop that takes in one action, followed by a door sensor reading
        See Assignment slides for links to the lecture slides
        Assumes the robot has been moved by the amount u, then a wall distance reading was taken
        ONLY door sensor

        @param robot_sensor - has the robot sensor probabilities
        @param robot_ground_truth - robot location, has the probabilities for actually moving left if move_left called
        @param u will be the amount moved
        @param z will be the distance from the sensor
        """
        # GUIDE:
        #  Step 1 Move the particles (with moise added)
        #  Step 2 Calculate the weights using the distance sensor return value
        #  Step 3 Resample/importance weight
        # YOUR CODE HERE

    def plot_particles_with_weights(self, axs, world_ground_truth, robot_ground_truth):
        """Plot the particles (scaled by weights) and the doors and where the robot actually is
        @param axs - window to draw in
        @param world_ground_truth - for the doors
        @param robot_ground_truth - for the robot's location"""

        # Plot "walls"
        height = 0.75
        axs.plot([0.0, 1.0, 1.0, 0.0, 0.0], [0.0, 0.0, height, height, 0.0], '-k')
        # Plot "doors"
        door_width = 0.95 * world_ground_truth.door_width / 2.0
        for d in world_ground_truth.doors:
            axs.plot([d - door_width, d - door_width, d + door_width, d + door_width], [0.0, 0.5 * height, 0.5 * height, 0.0], '-r')

        min_ws = np.min(self.weights)
        max_ws = np.max(self.weights)
        if np.isclose(max_ws, min_ws):
            max_ws = min_ws + 0.01

        for w, p in zip(self.weights, self.particles):
            h = 0.2 * (w - min_ws) / (max_ws - min_ws) + 0.05
            axs.plot([p, p], [0.01, 0.01 + h], '-g')

        # Robot
        axs. plot(robot_ground_truth.robot_loc, 0.05, 'xb', markersize=10)



def convert_histogram(pf, n_bins):
    """ Convert the particle filter to an (approximate) pmf in order to compare results
    @param pf - the particle filter
    @param n_bins - number of bins
    @returns a numpy array with (normalized) probabilities"""

    bins = np.zeros(n_bins)
    for p in pf.particles:
        bin_index = int(np.floor(p * n_bins))
        try:
            bins[bin_index] += 1.0
        except IndexError:
            if p < 0.0 or p > 1.0:
                raise ValueError(f"Convert histogram: particle location not in zero to 1 {p}")
            bins[-1] += 1.0

    bins /= np.sum(bins)
    return bins


def test_particle_filter_syntax(b_print=True):
    """ Test the sequence of calls for syntax and basic errors
    @param b_print - do print statements, yes/no"""

    # Read in some move sequences and compare your result to the correct answer
    from make_tests import _get_path_name
    import json
    with open(_get_path_name("Data/check_particle_filter.json"), "r") as f:
        answers = json.load(f)

    if b_print:
        print("Testing particle filter (syntax)")

    particle_filter = ParticleFilter()
    world_ground_truth = WorldGroundTruth()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Generate some move sequences and compare to the correct answer
    n_doors = answers["n_doors"]
    n_bins = answers["n_bins"]

    seed = 3
    np.random.seed(seed)
    world_ground_truth.random_door_placement(n_doors, n_bins)

    # Set mu/sigmas
    robot_ground_truth.set_move_continuos_probabilities(answers["move_error"]["sigma"])
    robot_sensor.set_distance_wall_sensor_probabilities(answers["sensor_noise"]["sigma"])

    # This SHOULD insure that you get the same answer as the solutions, provided you're only calling uniform within
    #  the door sensor reading, one call to random.normal() for the move, and one call to uniform for each particle
    #  in the importance sampling*
    np.random.seed(3)

    # Try different sequences
    for seq in answers["results"]:
        # Reset to uniform
        particle_filter.reset_particles()
        robot_ground_truth.reset_location()

        for i, s in enumerate(seq["seq"]):
            if s == "Door":
                saw_door = robot_sensor.query_door(robot_ground_truth, world_ground_truth)
                particle_filter.calculate_weights_door_sensor_reading(world_ground_truth, robot_sensor, saw_door)
                particle_filter.resample_particles()
                if saw_door != seq["sensor_reading"][i]:
                    print(f"Warning: expected {seq['sensor_reading'][i]} got {saw_door}")
            elif s == "Dist":
                dist = robot_sensor.query_distance_to_wall(robot_ground_truth)
                particle_filter.calculate_weights_distance_wall(robot_sensor, dist)
                particle_filter.resample_particles()
                if not np.isclose(dist, seq["sensor_reading"][i]):
                    print(f"Warning: expected {seq['sensor_reading'][i]} got {dist}")
            elif s == "Move":
                actual_move = robot_ground_truth.move_continuous(seq["move_amount"][i])
                particle_filter.update_particles_move_continuous(robot_ground_truth, seq["move_amount"][i])
            elif s == "Move_dist":
                actual_move = robot_ground_truth.move_continuous(seq["move_amount"][i])
                dist = robot_sensor.query_distance_to_wall(robot_ground_truth)
                particle_filter.one_full_update_distance(robot_ground_truth, robot_sensor, u=seq["move_amount"][i], z=dist)
            elif s == "Move_door":
                actual_move = robot_ground_truth.move_continuous(seq["move_amount"][i])
                saw_door = robot_sensor.query_door(robot_ground_truth, world_ground_truth)
                particle_filter.one_full_update_door(world_ground_truth, robot_ground_truth, robot_sensor, u=seq["move_amount"][i], z=saw_door)
            else:
                raise ValueError(f"Should be one of Move, Sensor, or Both, got {s}")
            
    if b_print:
        print("Passed syntax check")
    return True


def test_doors(b_print=True):
    if b_print:
        print("Testing particle filter (doors)")

    particle_filter = ParticleFilter()
    world_ground_truth = WorldGroundTruth()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Set the doors
    seed = 3
    np.random.seed(seed)
    world_ground_truth.random_door_placement()

    # Set the probability of seeing a door if there is one
    robot_sensor.set_door_sensor_probabilites(in_prob_see_door_if_door=0.9, in_prob_see_door_if_not_door=0.2)

    # Do the door query
    robot_sensor.query_door(world_gt=world_ground_truth, robot_gt=robot_ground_truth)

    for test_door in {True, False}:
        # Scatter some particles
        n_samples = 200
        particle_filter.reset_particles(n_samples=n_samples)

        if particle_filter.get_number_particles() != n_samples:
            print(f"Failed: Expected {n_samples} got {particle_filter.get_number_particles()}")
        else:
            if b_print:
                print("Passed get_number_of_particles check")

        # Update particle weights with True
        particle_filter.calculate_weights_door_sensor_reading(world_ground_truth, robot_sensor, test_door)

        # Check that the values in front (and not in front) of the door are correct
        weight_in_front_of_door = -1.0
        weight_not_in_front_of_door = -1.0
        b_particle_values_same_in_front_of_door = True
        b_particle_values_same_not_in_front_of_door = True
        for indx in range(0, particle_filter.get_number_particles()):
            if world_ground_truth.is_location_in_front_of_door(particle_filter.get_particle_location(indx)):
                if weight_in_front_of_door == -1.0:
                    weight_in_front_of_door = particle_filter.get_particle_weight(indx)
                else:
                    if not np.isclose(weight_in_front_of_door, particle_filter.get_particle_weight(indx)):
                        b_particle_values_same_in_front_of_door = False
            else:
                if weight_not_in_front_of_door == -1.0:
                    weight_not_in_front_of_door = particle_filter.get_particle_weight(indx)
                else:
                    if not np.isclose(weight_not_in_front_of_door, particle_filter.get_particle_weight(indx)):
                        b_particle_values_same_not_in_front_of_door = False

        if not b_particle_values_same_in_front_of_door:
            print("Failed test: All particle values for particles in front of door should be the same")
            return False
        else:
            if b_print:
                print("Passed: All particle values for particles in front of door should be the same")

        if not b_particle_values_same_not_in_front_of_door:
            print("Failed test: All particle values for particles NOT in front of door should be the same")
            return False
        else:
            if b_print:
                print("Passed: All particle values for particles NOT in front of door should be the same")

        if test_door:
            if not weight_in_front_of_door > weight_not_in_front_of_door:
                print(f"Failed sensor True test: probability for in front of door should be bigger")
                return False
            else:
                if b_print:
                    print("Passed test_door {test_door}")
        else:
            if not weight_in_front_of_door < weight_not_in_front_of_door:
                print(f"Failed sensor False test: probability for NOT in front of door should be bigger")
                return False
            else:
                if b_print:
                    print("Passed test_door {test_door}")
    return True


def test_distance(b_print=True):
    if b_print:
        print("Testing particle filter (distance)")

    particle_filter = ParticleFilter()
    world_ground_truth = WorldGroundTruth()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Set the doors
    seed = 3
    np.random.seed(seed)
    world_ground_truth.random_door_placement()

    # Set the probability of seeing a door if there is one
    robot_sensor.set_distance_wall_sensor_probabilities(sigma=0.1)

    n_particles = 50
    for loc in np.linspace(0.1, 0.9, 3):
        robot_ground_truth.robot_loc = loc
        
        # Do the distance query
        particle_filter.reset_particles(n_particles)
        particle_filter.calculate_weights_distance_wall(robot_sensor, loc)

        loc_weight_pairs = []
        for indx in range(0, particle_filter.get_number_particles()):
            loc_weight_pairs.append((particle_filter.weights[indx], abs(particle_filter.get_particle_location(indx) - loc)))
        loc_weight_pairs.sort()

        b_in_descending = True
        last_dist = loc_weight_pairs[0][1]
        for indx in range(1, particle_filter.get_number_particles()):
            # As the weight goes up, the distance should go down
            if loc_weight_pairs[indx][1] > last_dist:
                print(f"Distance: weight is not monotonically decreasing, robot location {loc}")
                return False
            last_dist = loc_weight_pairs[indx][1]

    return True

def test_reweighting(b_print=True):
    if b_print:
        print("Testing particle filter (reweighting)")

    particle_filter = ParticleFilter()
    world_ground_truth = WorldGroundTruth()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Set the doors
    seed = 3
    np.random.seed(seed)
    world_ground_truth.random_door_placement()

    # Set the distance wall sensor prob
    robot_sensor.set_distance_wall_sensor_probabilities(sigma=0.1)

    n_particles = 50
    for loc in np.linspace(0.1, 0.9, 3):
        # Sensor reading first, to calculate weights
        robot_ground_truth.robot_loc = loc
        particle_filter.reset_particles(n_particles)
        particle_filter.calculate_weights_distance_wall(robot_sensor, loc)

        # Now resample
        particle_filter.resample_particles()

        # Check that weights are back to the same
        weight_all = particle_filter.get_particle_weight(0)
        # Check that more particles in bin with loc than not
        bins = [0, 0, 0]
        for indx in range(0, particle_filter.get_number_particles()):
            if not np.isclose(weight_all, particle_filter.get_particle_weight(indx)):
                print(f"Failure: Expected all weights to be back to the same value {weight_all}, got other value at {indx}")
                return False
            bin_indx = int(np.floor(particle_filter.get_particle_location(indx) * 2.99))
            bins[bin_indx] += 1

        cur_bin = int(np.floor(loc * 2.999))
        for j in range (0, 2):
            not_cur_bin = (cur_bin + 1) % 3
            if bins[cur_bin] < bins[not_cur_bin]:
                print(f"Failed: Bin at loc {loc} did not have more particles than bin at {0.1 * not_cur_bin}, bins {bins}")
    if b_print:
        print("Passed test: All particles have equal weight")
        print("Passed test: most particles close to robot loc after distance sensor read")
    return True
            

def test_particle_move(b_print=True):
    """ Test the move and reset"""
    if b_print:
        print("Testing particle filter (move)")

    particle_filter = ParticleFilter()
    world_ground_truth = WorldGroundTruth()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Set the probability of seeing a door if there is one
    robot_sensor.set_distance_wall_sensor_probabilities(sigma=0.1)

    n_particles = 1000
    for t in [("left", 1), ("left", 2), ("right", 1), ("right", 2)]:
        robot_ground_truth.robot_loc = 0.5        
        particle_filter.reset_particles(n_samples=n_particles)

        # Do a wall query to make the particle distribution gaussian shaped around 0.5
        # NOTE: Assumes your re-weight and sensor reading code is correct
        particle_filter.calculate_weights_distance_wall(robot_sensors=robot_sensor, dist_reading=robot_ground_truth.robot_loc)
        particle_filter.resample_particles()
        hist_initial = convert_histogram(particle_filter, n_bins=9)
        count_bad = 0
        for b in hist_initial:
            if b > hist_initial[4]:
                if b_print:
                    print(f"Bin 4 should be the biggest after sensor reading")
                    count_bad += 1

        bin_check = 4
        sum_slope = hist_initial[3] + hist_initial[4] + hist_initial[5]
        for _ in range(0, t[1]):
            amt = 1.0 / 9.0
            if t[0] == "left":
                amt = amt * -1.0
                bin_check -= 1
            else:
                bin_check += 1

            robot_ground_truth.move_continuous(amount=amt)
            particle_filter.update_particles_move_continuous(robot_ground_truth=robot_ground_truth, amount=amt)
            hist_move = convert_histogram(particle_filter, 9)
            for b in hist_move:
                if b > hist_move[bin_check]:
                    if b_print:
                        print(f"Center of histogram should move {t} as robot moves")
                        count_bad += 1
            new_sum_slope = hist_move[bin_check-1] + hist_move[bin_check] + hist_move[bin_check+1]
            if new_sum_slope > sum_slope:
                if b_print:
                    print(f"Histogram should get wider after move")
                count_bad += 10
            sum_slope = new_sum_slope

    if count_bad < 5:
        return True
    if b_print == True:
        print(f"Passed move test")
    return False


if __name__ == '__main__':
    b_print = True

    # Syntax checks
    n_doors_syntax = 2
    n_bins_syntax = 10
    n_samples_syntax = 100
    world_ground_truth_syntax = WorldGroundTruth()
    world_ground_truth_syntax.random_door_placement(n_doors_syntax, n_bins_syntax)
    robot_ground_truth_syntax = RobotGroundTruth()
    robot_sensor_syntax = RobotSensors()
    particle_filter_syntax = ParticleFilter()

    # Syntax check 1, reset probabilities
    particle_filter_syntax.reset_particles(n_samples_syntax)

    # Syntax check 2, update move
    particle_filter_syntax.update_particles_move_continuous(robot_ground_truth_syntax, 0.1)

    # Syntax checks 3 and 4 - the two different sensor readings
    particle_filter_syntax.calculate_weights_door_sensor_reading(world_ground_truth_syntax, robot_sensor_syntax, True)
    if np.isclose(np.max(particle_filter_syntax.weights), np.min(particle_filter_syntax.weights)):
        print(f"Possible error: The weights should not all be the same")

    particle_filter_syntax.reset_particles(n_samples_syntax)
    particle_filter_syntax.calculate_weights_distance_wall(robot_sensor_syntax, 0.1)
    if np.isclose(np.max(particle_filter_syntax.weights), np.min(particle_filter_syntax.weights)):
        print(f"Possible error: The weights should not all be the same")

    # Syntax check 5 - importance sampling
    particle_filter_syntax.resample_particles()
    if not np.isclose(np.max(particle_filter_syntax.weights), np.min(particle_filter_syntax.weights)):
        print(f"Possible error: The weights should be set back to all the same")
    if np.unique(particle_filter_syntax.particles, return_counts=True) == n_samples_syntax:
        print(f"Possible error: There probably should be duplicate particles {np.unique(particle_filter_syntax.particles, return_counts=True)} {n_samples_syntax}")

    # Syntax checks 6 and 7 - the two full updates
    particle_filter_syntax.one_full_update_door(world_ground_truth_syntax, robot_ground_truth_syntax, robot_sensor_syntax, u=0.1, z=True)
    particle_filter_syntax.one_full_update_distance(robot_ground_truth_syntax, robot_sensor_syntax, u=0.1, z=0.6)

    # Check the door weighting
    test_doors(b_print=b_print)

    # Check the distance center weighting
    test_distance(b_print=b_print)

    # Check the importance sampling
    test_reweighting(b_print=b_print)

    # Check the movement
    test_particle_move(b_print=True)

    # Check the full system
    from make_tests import test_particle_filter_update
    test_particle_filter_update(b_check_res=True, b_print=b_print)

    print("Done")
