#!/usr/bin/env python3

# This assignment implements Dijkstra's shortest path on a graph, finding an unvisited node in a graph,
#   picking which one to visit, and taking a path in the map and generating waypoints along that path
#
# Given to you:
#   Priority queue
#   Image handling
#   Four and Eight connected neighbors
#
# Slides https://docs.google.com/presentation/d/1XBPw2B2Bac-LcXH5kYN4hQLLLl_AMIgoowlrmPpTinA/edit?usp=sharing

# The ever-present numpy
import numpy as np

# Our priority queue
import heapq


# -------------- Showing start and end and path ---------------
def plot_with_path(im_threshhold, zoom=1.0, robot_loc=None, goal_loc=None, path=None):
    """Show the map plus, optionally, the robot location and goal location and proposed path
    @param im - the image of the SLAM map (numpy array)
    @param im_threshhold - the image of the SLAM map, threshholded
    @param zoom - how much to zoom into the map (value between 0 and 1)
    @param robot_loc - the location of the robot in pixel coordinates
    @param goal_loc - the location of the goal in pixel coordinates
    @param path - the proposed path in pixel coordinates"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 1)
    axs.imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs.set_title("threshold image")

    # Double checking lower left corner
    axs.plot(10, 5, 'xy', markersize=5)

    # Show original and thresholded image
    if robot_loc is not None:
        axs.plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
    if goal_loc is not None:
        axs.plot(goal_loc[0], goal_loc[1], '*g', markersize=10)
    if path is not None:
        for p, q in zip(path[0:-1], path[1:]):
            axs.plot([p[0], q[0]], [p[1], q[1]], '-y', markersize=2)
            axs.plot(p[0], p[1], '.y', markersize=2)
    axs.axis('equal')

    # Implements a zoom - set zoom to 1.0 if no zoom
    width = im_threshhold.shape[1]
    height = im_threshhold.shape[0]

    axs.set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
    axs.set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- Thresholded image True/False ---------------
def is_wall(im, pix=(0, 0)):
    """ Is the pixel a wall pixel?
    @param im - the image
    @param pix - the pixel i,j
    @return True if pixel value is zero"""
    if im[pix[1], pix[0]] == 0:
        return True
    return False


def is_unseen(im, pix=(0, 0)):
    """ Is the pixel one we've seen?
    @param im - the image
    @param pix - the pixel i,j
    @return True if pixel value 128 (the unseen color value)"""
    if im[pix[1], pix[0]] == 128:
        return True
    return False


def is_free(im, pix=(0,0)):
    """ Is the pixel empty?
    @param im - the image
    @param pix - the pixel i,j
    return True if 255 """
    if im[pix[1], pix[0]] == 255:
        return True
    return False


def convert_image(im, wall_threshold, free_threshold):
    """ Convert the image to a thresholded image with 'not seen' pixels marked
    @param im - width by height image as numpy (depends on input)
    @param wall_threshold - number between 0 and 1 to indicate wall threshold value
    @param free_threshold - number between 0 and 1 to indicate free space threshold value
    @return an image of the same WXH but with 0 (free) 255 (wall) 128 (unseen)"""

    # Assume all is unseen - fill the image with 128
    im_ret = np.zeros((im.shape[0], im.shape[1]), dtype='uint8') + 128

    im_avg = im
    if len(im.shape) == 3:
        # RGB image - convert to gray scale
        im_avg = np.mean(im, axis=2)
    # Force into 0,1
    im_avg = im_avg / np.max(im_avg)
    # threshold
    #   in our example image, black is walls, white is free
    im_ret[im_avg < wall_threshold] = 0
    im_ret[im_avg > free_threshold] = 255
    return im_ret


# -------------- Getting 4 or 8 neighbors ---------------
def four_connected(pix=(0, 0)):
    """ Generator function for 4 neighbors
    @param im - the image
    @param pix - the i, j location to iterate around"""
    for indx in [-1, 1]:
        ret = pix[0] + indx, pix[1]
        yield ret
    for indx in [-1, 1]:
        ret = pix[0], pix[1] + indx
        yield ret


def eight_connected(pix=(0, 0)):
    """ Generator function for 8 neighbors
    @param im - the image
    @param pix - the i, j location to iterate around"""
    for indx in range(-1, 2):
        for j in range(-1, 2):
            # Skip the middle pixel
            if indx == 0 and j == 0:
                pass
            ret = pix[0] + indx, pix[1] + j
            yield ret


def dijkstra(im, robot_loc=(0, 0), goal_loc=(0, 0)):
    """ Occupancy grid image, with robot and goal loc as pixels
    @param im - the thresholded image - use is_free(i, j) to determine if in reachable node
    @param robot_loc - where the robot is (i,j)
    @param goal_loc - where to go to (i,j)
    @returns a list of tuples"""

    # Sanity checks for ROS 2 assignment - these should properly trigger errors, but
    #. that makes it hard to debug the ROS 2 assignment
    try:
        if not is_free(im, robot_loc):
            print(f"ERROR: Start location {robot_loc} is not in the free space of the map")
            return []

        if not is_free(im, goal_loc):
            print(f"ERROR: Goal location {goal_loc} is not in the free space of the map")
            return []
    except IndexError:
            print(f"ERROR: robot {robot_loc} or {goal_loc} are not in the map {im.shape}")
            return []

    # The priority queue itself is just a list, with elements of the form (weight, (i,j))
    #    - i.e., a tuple with the first element the weight/score, the second element a tuple with the pixel location
    priority_queue = []
    # Push the start node onto the queue
    #   push takes the queue itself, then a tuple with the first element the priority value and the second
    #   being whatever data you want to keep - in this case, the robot location, which is a tuple
    heapq.heappush(priority_queue, (0, robot_loc))

    # The power of dictionaries - we're going to use a dictionary to store every node we've visited, along
    #   with the node we came from and the current distance
    # This is easier than trying to get the distance from the heap
    visited = {}
    # Use the (i,j) tuple to index the dictionary
    #   Store the best distance found so far, the parent node, and if it is closed y/n
    # Push the first node onto the heap - distance is zero, it has no parent, and it is NOT closed
    visited[robot_loc] = (0, None, False)   # For every other node this will be the current_node, distance, False

    # While the list is not empty 
    # Use a break statement to end the while loop if you encounter the goal node before the queue empties
    while priority_queue:
        # Get the current best node off of the list (pop the node off the queue)
        current_node = heapq.heappop(priority_queue)
        # Pop returns the value and the i, j
        distance_to_current_node = current_node[0]
        current_node_ij = current_node[1]  # i,j index of current node

        # Showing how to get this data back out of visited
        visited_triplet = visited[current_node_ij]  # This is a tuple with three values
        visited_distance = visited_triplet[0]       # First value is the current distance stored for that node
        visited_parent = visited_triplet[1]         # Second value is the parent node of this one
        visited_closed_yn = visited_triplet[2]      # Third value is if this node is closed y/n

        # GUIDE
        #  Step 1: Break out of the loop if current_node_ij is the goal node
        #  Step 2: If this node is closed, skip it
        #  Step 3: Set the node to closed
        #    Now do the instructions from the slide (the actual algorithm)
        #  See also lecture slides
        # YOUR CODE HERE

    # Now check that we actually found the goal node
    if not goal_loc in visited:
        print(f"Goal {goal_loc} not reached, taking closest")

        # GUIDE: Deal with not being able to get to the goal loc
        #   If the goal location is not reachable, find the node closest to the goal 
        #.  and return the path to it - you'll want this for the ROS 2 assignment
        # YOUR CODE HERE

    path = []
    path.append(goal_loc)
    # GUIDE: Build the path by starting at the goal node and working backwards
    # YOUR CODE HERE

    return path


def open_image(im_name):
    """ A helper function to open up the image and the yaml file and threshold
    @param im_name - name of image in Data directory
    @returns image anbd thresholded image"""

    # Using imageio to read in the image
    import imageio.v2 as imageio
    # yaml for file format
    import yaml as yaml

    # Needed for reading in map info
    from os import open, path
    fname = "Data/" + im_name
    im = imageio.imread(fname)
    
    wall_threshold = 0.7
    free_threshold = 0.9
    try:
        yaml_name = "Data/" + im_name[0:-3] + "yaml"
        with open(yaml_name, "r") as f:
            dict = yaml.load_all(f)
            wall_threshold = dict["occupied_thresh"]
            free_threshold = dict["free_thresh"]
    except:
        pass

    im_thresh = convert_image(im, wall_threshold, free_threshold)
    return im, im_thresh


def check_path_continuous(im, path, expected_len_four, expected_len_eight):
    """ Checks that the path is continuous and in free space"""
    b_is_eight = False
    pass_connected_test = True
    for p1, p2 in zip(path[0:-1], path[1:]):
        if abs(p1[0] - p2[0]) > 1:
            pass_connected_test = False
        if abs(p1[1] - p2[1]) > 1:
            pass_connected_test = False
        if abs(p1[0] - p2[0]) > 0 and abs(p1[1] - p2[1]):
            b_is_eight = True

    pass_len_test = False
    expected_len = expected_len_eight if b_is_eight else expected_len_four
    if abs(len(path) - expected_len) < 3:
        pass_len_test = True
    
    pass_free_test = True
    for pt in path:
        if not is_free(im, pt):
            pass_free_test = False

    if not pass_connected_test:
        print(f"Failed connected test")
        return False
    if not pass_free_test:
        print(f"Failed all path points must be free test")
        return False
    if not pass_len_test:
        print(f"Failed length test")
        if b_is_eight:
            print(f" Assumed 8 connected")
        else:
            print(f" Assumed 4 connected")
        return False
    return True


if __name__ == '__main__':
    
    # Use one of these
    robot_start_loc = (40, 60)
    robot_goal_loc_close = (60, 80)
    robot_goal_hallway = (80, 175)
    robot_goal_next_room = (130, 50)
    loc_not_reachable = (115, 145)

    # Opens and threshold the SLAM map image
    _, im_thresh = open_image("map.pgm")
    zoom = 1.0

    robot_goal_loc = robot_goal_loc_close
    path = dijkstra(im_thresh, robot_start_loc, robot_goal_loc)
    plot_with_path(im_thresh, zoom=zoom, robot_loc=robot_start_loc, goal_loc=robot_goal_loc, path=path)

    path_straight = dijkstra(im_thresh, robot_loc=(40, 60), goal_loc=robot_goal_loc_close)
    assert check_path_continuous(im_thresh, path_straight, 41, 21)

    path_hallway = dijkstra(im_thresh, robot_loc=(40, 60), goal_loc=robot_goal_hallway)
    assert check_path_continuous(im_thresh, path_hallway, 156, 116)

    path_next_room = dijkstra(im_thresh, robot_loc=(40, 60), goal_loc=robot_goal_next_room)
    assert check_path_continuous(im_thresh, path_next_room, 315, 241)

    # This one will be SLOW
    path_not_reachable = dijkstra(im_thresh, robot_loc=(40, 60), goal_loc=loc_not_reachable)
    assert len(path_not_reachable) > 20

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt
    plt.show()

    print("Done")
