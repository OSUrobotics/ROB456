#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import json as json

# Using your code from lab 1/hwk 1, read in the pick data, the pick data description, and plot the Wrist force z
#  channel for the first row. See the Lab lecture slides for this week for the correct answer.
#   https://docs.google.com/presentation/d/1IiGGUNet-4Nj07x2cTXU6IOYXy9TSdAF5OUWCCKIYEM/edit?usp=sharing
# I recommend using the subplots approach (rather than just plot) because eventually we'll be plotting in more than
#  one plotting area

# Read the data in
#   Note the ../Week_1_arrays/ part in front - this tells the file manager to go up one directory, then
#    into the Week_1_arrays directory, which is where the data is
pick_data = np.loadtxt("../Data/proxy_pick_data.csv", dtype="float", delimiter=",")

# Reminder: You need to change the directory when reading in the pick data description, too
# BEGIN SOLUTION
try:
    with open("../Data/week1_check_results.json", "r") as fp:
        pick_data_description = json.load(fp)
except FileNotFoundError:
    print(f"The file was not found; check that the data directory is in the current one and the file is in that directory")

# END SOLUTION


# Plot the Wrist force z channel for the first row.
#
# For the t values, assume the data is sampled at 30 Hz, i.e., the time sampling is 1/30th of a second
#  Step 1: How big does the t array have to be? (hint: How many data samples are there for the wrist force data?)
#  Step 2: How do you make an array of that size with that spacing (hint: np.arange)
#

# BEGIN SOLUTION NO PROMPT
time_step = 1.0 / 30   # Seconds
# Create a numpy array that starts at 0, ends at number of time steps * time_step, and has step size time_step
#   See np.arange
ts = np.arange(start=0, stop=pick_data_description["n_time_steps"] * time_step, step=time_step)

# I'd recommend getting the dictionary that has the wrist offset information here
wrist_torque_channel_data = pick_data_description["Data channels"][1]

# Now get out the actual data - it might be handy here to get the data out and assign it to a variable, just to make
#  sure it's the right thing
start_index = wrist_torque_channel_data["index_offset"]
y_data = pick_data[0, start_index::pick_data_description["n_total_dims"]]

# END SOLUTION

""" # BEGIN PROMPT
time_step = ...
# Array holding the time values
ts = ...
wrist_torque_channel_data = ...
y_data = ...
# END PROMPT """

# Create the plotting window
nrows = 1
ncols = 1
# BEGIN SOLUTION NO PROMPT
fig, axs = plt.subplots(nrows, ncols, figsize=(6, 3))

# Now actually plot the data
#   Notice the label - this is a look-ahead to when we want a more general plot command. This takes the name that's in
#   the data channel information and adds z to it
axs.plot(ts, y_data, '-b', label=wrist_torque_channel_data["name"] + ", z")

# Now label the figure (x label, y label, title).
#   Note: The units for each data channel are in proxy_data_description.json
axs.set_xlabel('Time (seconds)')
axs.set_ylabel(wrist_torque_channel_data["units"])
axs.set_title(wrist_torque_channel_data["name"])
axs.legend()

# END SOLUTION

""" # BEGIN PROMPT
fig, axs = ...
# END PROMPT """

# Put a break point here to stop the code from exiting
plt.close(fig)

