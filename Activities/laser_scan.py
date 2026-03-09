#!/usr/bin/env python3

# These are the reading and drawing routines for the scans

import numpy as np
import matplotlib.pyplot as plt

class LaserScan:
    def __init__(self, fname):
        self.angle_min = -np.pi/2
        self.angle_max = np.pi/2
        self.ranges = [0.0] * 16
        self.angle_increment = (self.angle_max - self.angle_min) / len(self.ranges)
        self.range_min = 0.0
        self.range_max = 5.0

        self.read_scan(fname)

    def read_scan(self, fname):
        var_names = vars(self)
        self.ranges = []
        with open(fname, "r") as f:
            lines = f.read().split('\n')
            b_found_ranges = False
            b_found_end_ranges = False
            for l in lines:
                if not b_found_ranges:
                    words = l.split(" ")
                    first_word = words[0]
                    for w in words:
                        if ":" in w:
                            first_word = w
                    my_key = first_word[:-1]
                    if my_key == "ranges":
                        b_found_ranges = True
                    elif my_key in var_names:
                        var_names[my_key] = float(words[-1])
                elif not b_found_end_ranges:
                    ss = l.split("-")
                    if len(ss) > 2 or "..." in ss[-1]:
                        b_found_end_ranges = True
                    else:
                        self.ranges.append(float(ss[-1]))


def convert_scan_to_xy(scan: LaserScan):
    """ Return xs, ys - the end points of the scan
    @param scan - the laser scan
    @return a tuple 
    """
    xs = [] 
    ys = []
    # GUIDE
    #. convert range scan to x,y location
    # YOUR CODE HERE
    return xs, ys


def label_scan(scan: LaserScan, robot_width = .38):
    """ Label each scan end-point with where it is wrt the robot
    @param scan - the laser scan class above (similar to the real laser scan in ROS)
    @param robot_width = the width of the robot in m (reminder that scans are in cm)
    @return a list the length of the number of scans, with the strings 'Left', 'Right', or 'Front' """
    # GUIDE: Label each scan with "Left", "Front", or "Right". 
    #. Front means the scan ends with a y value that is in front of the robot
    # Fancy way of making an array of labels the same size as the number of scans
    # Set all labels that are to the left of the robot with "Left", etc
    labels = ["Front"] * len(scan.ranges)
    # YOUR CODE HERE
    return labels


def plot_scan(axs, scan : LaserScan, optional_labels = []):
    cols = {"Left": "g-", "Front": "b-", "Right": "c-"}
    if optional_labels == []:
        labels = ["Front"] * len(scan.ranges)
    else:
        labels = optional_labels

    xs, ys = convert_scan_to_xy(scan)
    for x, y, lab in zip(xs, ys, labels):
        axs.plot([0, x], [0, y], cols[lab])
    axs.set_aspect("equal")


def plot_robot(axs, robot_width=0.38, stopping_distance=1.5):
    """ Plot the robot in the window, with the stopping distance as a line
    @param robot_width - the width of the robot in m (reminder that scans are in m)
    @param stopping_distance - What distance from the wall/obstacle to stop at"""

    box_xs = [robot_width * 0.5] * 5
    box_ys = [robot_width * 0.5] * 5
    box_xs[0] *= -1
    box_xs[3] *= -1
    box_xs[4] *= -1

    box_ys[0] *= -1
    box_ys[1] *= -1
    box_ys[4] *= -1

    axs.plot(box_xs, box_ys, '-k')
    axs.plot([stopping_distance, stopping_distance], [-robot_width*2.0, robot_width*2.0], '-k')

    axs.plot([0, 5.0], [robot_width * 0.5, robot_width * 0.5], '--k')
    axs.plot([0, 5.0], [-robot_width * 0.5, -robot_width * 0.5], '--k')


def plot_twist(axs, twist, robot_width):
    """ Show the twist 
    @param twist - the linear move and the angular spin as a tuple"""
    x = (twist[0] * 10.0 + robot_width * 0.5) * np.cos(twist[1])
    y = (twist[0] * 10.0 + robot_width * 0.5) * np.sin(twist[1])
    axs.plot(x, y, 'Xk')


def get_twist_values(scan: LaserScan, robot_width = 0.38, stopping_distance = 1.5, max_speed = 0.2):
    """ Label each scan end-point with where it is wrt the robot
    @param scan - the laser scan class above (similar to the real laser scan in ROS)
    @param robot_width - the width of the robot in m (reminder that scans are in m)
    @param stopping_distance - What distance from the wall/obstacle to stop at
    @param max_speed - fastest speed to travel at (assuming no obstacles)
    @return a tuple with the linear x (speed) and angular z (turn) values """

    linear_x = 0.0
    angular_z = 0.0

    # GUIDE
    # All labs
    # Use angle min, max, and number of readings to calculate the theta value for each scan
    # This should be a numpy array of length num_readings, that starts at angle_min and ends at angle_max
    #.  Reminder: These are variables in 
    # YOUR CODE HERE

    # GUIDE: Determine what the closest obstacle/reading is for scans in front of the robot
    #  Step 1: Determine which of the range readings correspond to being "in front of" the robot (see comment at top)
    #    Remember that robot scans are in the robot's coordinate system - theta = 0 means straight ahead
    #  Step 2: Get the minimum distance to the closest object (use only scans "in front of" the robot)
    #  Step 3: Use the closest distance from above to decide when to stop
    #  Step 4: Scale how fast you move by the distance to the closet object (tanh is handy here...)
    #  Step 5: Make sure to actually stop if close to 1 m
    # Finally, set t.linear.x to be your desired speed (0 if stop)
    # Suggestion: Do this with a for loop before being fancy with numpy (which is substantially faster)
    # DO NOT hard-wire in the number of readings, or the min/max angle. You CAN hardwire in the size of the robot

    # Instead of just stopping, we're going to set the speed based on the distance.
    # As we get closer to the required distance, we slow down.  If we're too close, we back up.  
    # To smooth things out, we're going to use a tanh function, which
    # looks like a smoothed-out step function, which asymptotes to -1 and +1 at x = -1 to 1.
    #.  (Ask Google to draw tanh for you)
    # There's nothing magical about this function, but it is used a lot for this sort of thing
    # Tricky part: You have to scale both the input AND the output of tanh to get what you want
    #.  Input: Need to scale the input to tanh to be between -1 and 1. The input is the distance
    #.   at which you want to start slowing down; eg, if you wanted to start slowing down at a
    #.   distance of d and stop at a distance of d_stop, you would want to set the input to atanh to
    #.   be -1 when distance is d, and 0 when distance is d_top
    #.  Output: tanh always outputs a value between -1 and 1. You need to scale the output so that
    #.   when tanh returns 1, you are going at the maximum allowable speed
    #. Since it's also hard to get exactly to zero, if the robot is "close" to the correct distance, just stop

    # Return max_speed
    shortest = 0
    max_speed = 0.2

    # You can use convert_scan_to_xy here if you want
    # YOUR CODE HERE
    return (linear_x, angular_z)

if __name__ == '__main__':
    import matplotlib.pyplot as pyplt

    robot_width = 0.39
    stopping_distance = 1.0

    names = ["hallway_left", "hallway_right_stop", "doorway",
             "corner_left", "corner_right", "right_wall", 
             "over_one_meter", "obstacle_close", "left_wall",
             "stop_corner_on_left", "stop_corner_on_right", "stop_wall"]

    nrows = 3
    ncols = 4
    fig, axs = pyplt.subplots(nrows, ncols, figsize=(ncols*6, 6))
    
    for indx, n in enumerate(names):
        r = indx // ncols
        c = indx % ncols
        scan = LaserScan("Data/" + n + ".txt")
        labels = label_scan(scan=scan)
        twist = get_twist_values(scan=scan, robot_width=robot_width, stopping_distance=stopping_distance)

        ax = axs[r, c]
        plot_scan(axs=ax, scan=scan, optional_labels=labels)
        plot_robot(axs=ax, robot_width=robot_width, stopping_distance=stopping_distance)
        plot_twist(axs=ax, twist=twist, robot_width=robot_width)
        ax.set_title(n)

    fig.tight_layout()
    pyplt.show()