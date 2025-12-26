#!/usr/bin/env python3

# This is code for making (and doing) the full update tests

import numpy as np
import json


from world_ground_truth import WorldGroundTruth
from robot_sensors import RobotSensors
from robot_ground_truth import RobotGroundTruth
from particle_filter import ParticleFilter, convert_histogram

def _get_path_name(fname: str):
    """ Just checks the directory is correct, and adds RobotHallway if needed"""
    from os import getcwd
    cur_path = getcwd().split("/")
    if cur_path[-1] != "RobotHallway":
        fname = "RobotHallway/" + fname
    return fname



def test_bayes_move_update(b_print=True):
    """ Test the move update. This test is done by comparing your probability values to some pre-calculated/saved values
    @param b_print - do print statements, yes/no"""

    from bayes_filter import BayesFilter
    bayes_filter = BayesFilter()
    world_ground_truth = WorldGroundTruth()
    robot_ground_truth = RobotGroundTruth()
    robot_sensor = RobotSensors()

    # Read in some move sequences and compare your result to the correct answer
    import json
    fname = _get_path_name("Data/check_bayes_filter.json")
    with open(fname, "r") as f:
        answers = json.load(f)

    n_doors = answers["n_doors"]
    n_bins = answers["n_bins"]
    step_size = 1.0 / n_bins
    world_ground_truth.doors = answers["world"]

    if b_print:
        print("Testing move update")

    # This SHOULD insure that you get the same answer as the solutions, provided you're only calling uniform within
    #  robot_ground_truth.move and robot_sensor.query door*
    seed = 3
 
    # Try different probability values
    for ia, answer in enumerate(answers["answers"]):
        # This SHOULD insure that you get the same answer as the solutions, provided you're only calling uniform within
        #  robot_ground_truth.move and robot_sensor.query_door*
        np.random.seed(seed)
        seed = seed + 1

        seq = answer["seq"]
        zs = answer["Sensors"]

        # Reset to uniform
        bayes_filter.reset_probabilities(n_bins)
        robot_ground_truth.reset_location()
        for z_check, s in zip(zs, seq):
            if s == "left":
                robot_ground_truth.move_left(step_size)
            elif s == "right":
                robot_ground_truth.move_right(step_size)
            else:
                raise ValueError(f"Expected left or right, got {s}")

            z = robot_sensor.query_door(robot_ground_truth, world_ground_truth)
            if b_print:
                print(f"Loc {robot_ground_truth.robot_loc}, Doors {world_ground_truth.doors}, Sensor {z}")
            if z is not z_check:
                print(f"Warning: Sensor reading is different than check {z} versus {z_check}")

            bayes_filter.one_full_update(world_ground_truth, robot_ground_truth, robot_sensor, "move_" + s, z)

        check_seed = np.random.uniform()
        if not np.isclose(check_seed, answer["check_seed"]):
            print("Warning: random number generator is off, may report incorrect result, answer loop {ia}")

        prob_list = [bayes_filter.probability(indx) for indx in range(0, bayes_filter.n_bins())]
        if not np.any(np.isclose(answer["result"], prob_list, atol=0.01)):
            ValueError(f"Probabilities are different \n{answer['result']} \n{prob_list}")

    if b_print:
        print("Passed")
    return True


def test_kalman_update(b_print:bool = True):
    """ Check against the saved results
    @param b_print - print the results, y/n
    Beware that this requires only calling random.uniform when doing the sensor/move - any additional
     calls will throw the random numbers off"""

    if b_print:
        print("Testing Kalman")
    # Generate some move sequences and compare to the correct answer
    import json
    with open(_get_path_name("Data/check_kalman_filter.json"), "r") as f:
        answers = json.load(f)

    from kalman_filter import KalmanFilter
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

        if not np.isclose(seq["mu"], kalman_filter.location_mean(), atol=0.01):
            raise ValueError(f"Failed sequence {seq['seq']}, got mu {kalman_filter.location_mean()}, expected {seq['mu']}")
        if not np.isclose(seq["sigma"], kalman_filter.location_sigma(), atol=0.01):
            raise ValueError(f"Failed sequence {seq['seq']}, got sigma {kalman_filter.location_sigma()}, expected {seq['sigma']}")

    if b_print:
        print("Passed")
    return True


def test_particle_filter_update(b_check_res, b_print):
    """ Test the sequence of calls for syntax and basic errors
    @param b_check_res - check histogram result as well
    @param b_print - do print statements, yes/no"""

    # Read in some move sequences and compare your result to the correct answer
    import json
    from particle_filter import ParticleFilter
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

        if b_check_res:
            h = convert_histogram(particle_filter, n_bins)
            h_expected = np.array(seq["histogram"])
            res = np.isclose(h, h_expected, 0.1)
            if b_print:
                print(f"Should be approximately equal, seq: {seq['seq']}")
                print(f"{res}")
                print(f"Your h: {h}")
                print(f"Approximate h: {h_expected}\n")

    if b_print:
        if b_check_res:
            print("Passed update check")
        else:
            print("Passed syntax check")
    return True



if __name__ == '__main__':

    test_bayes_move_update(b_print=True)
    test_kalman_update(b_print=True)
    test_particle_filter_update(b_check_res=True, b_print=True)
