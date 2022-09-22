#!/usr/bin/env python3

import numpy as np


# This class keeps track of the "ground truth" of the world
#  - where the doors are
#  - width of the doors (for determining if robot in front of door)
class WorldGroundTruth:
    def __init__(self):
        """Real initialization will happen when random_door_placement called"""

        # Door configurations
        self.door_width = 1
        self.doors = []

        # Default door placement and door width
        self.random_door_placement()

    def random_door_placement(self, n_doors=3, n_bins=20):
        """ Randomly place the doors
        Door locations are always in the middle of the bin
        @param n_doors - number of doors to place. Should be less than number of bins
        @param n_bins - number of bins the world is divided up into (for determining door width)"""

        if n_bins <= n_doors:
            raise ValueError(f"Error, number of bins {n_bins} less than number of doors {n_doors}")

        div = 1.0 / n_bins
        door_loc = np.zeros(int(n_bins))
        # Randomly pick a door location until sufficient door locations filled
        while sum(door_loc) < n_doors:
            # Pick a random location in 0,1
            loc = np.random.uniform(0, 1)
            # Convert to a bin
            i_loc = min(int(n_bins), max(0, int(np.floor(loc / div))))
            # Set the closest location to 1
            while door_loc[i_loc] == 1:
                i_loc = (i_loc + 1) % n_bins
            door_loc[i_loc] = 1

        # A little extra padding
        self.door_width = div * 1.1
        self.doors = []
        for i in range(0, len(door_loc)):
            if door_loc[i]:
                self.doors.append((i + 0.5) * div)

    def location_in_front_of_door(self):
        """Return a location in front of the first door (testing routine)"""
        return self.doors[0]

    def location_not_in_front_of_door(self):
        """Return a location NOT in front of a door (testing routine)"""
        div = self.door_width / 1.1

        b_found_no_door = False
        location = div / 2
        while not b_found_no_door:
            if not self.is_location_in_front_of_door(location):
                b_found_no_door = True
            else:
                location += div

        return location

    # Determine if location is in front of door
    def is_location_in_front_of_door(self, location):
        """
        @param location - location in 0,1 to check
        @return True if location overlaps door
        """

        # Based on width of door overlap of robot and door
        door_width = self.door_width / 2

        # Determine percentage in front of door
        inside_door = [abs(location - d) < door_width for d in self.doors]
        if True in inside_door:
            return True
        return False


def test_world_ground_truth(b_print=True):
    """ Checking the world ground truth code (placing doors)
    @param b_print - do print statements, yes/no"""
    world = WorldGroundTruth()

    if b_print:
        print("Beginning world ground truth test")
    world.random_door_placement(3, 20)

    for doors in world.doors:
        for dw in np.linspace(doors - 0.49 * world.door_width, doors + 0.49 * world.door_width):
            if not world.is_location_in_front_of_door(dw):
                raise ValueError("Should be in front of door {}".format(dw))

    if b_print:
        print("Passed tests")
    return True


if __name__ == '__main__':
    b_print = True
    test_world_ground_truth(b_print)
