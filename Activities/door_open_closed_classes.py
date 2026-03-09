#!/usr/bin/env python3

import numpy as np


# This file has three classes, one for the "world", one for the "sensors", and one for belief (the actual Bayes equation)
#    This may be a bit overkill for this assignment, but we'll be using this general structure over
#. and over again, so might as well become familiar with it now.
#  World class keeps track of the "ground truth" of the world
#      - if the door is open or closed
#  it also handles the robot "opening" and "closing the door"
#.     - probabilities of the robot successfully opening/closing the door
#
#. And sensor queries: When you ask the sensor if the door is "open", it has to check if the door is open
# 
#  Sensor class handles generating "True/False" queries. The sensor class has access to the World class
#   so it can know the "ground truth" (if the door is ACTUALLY open or closed)
#
# The BeliefAboutDoor class handles tracking if you believe the door is open or closed (probabilities)

# GUIDE: The GUIDES are labeled with part 1 through part 4; you don't need to do it all at once
#. Also make sure to look for GUIDES/part in all of the classes
#

# SLIDES: https://docs.google.com/presentation/d/10joxdTeM7WvGhVDsHldWbn3c-eW4ea0pdXaAmFn9bWE/edit?usp=sharing


class DoorGroundTruth:
    # Define the actions the robot can take
    #   Defined both as a text string and as a number
    #. You can access this variable as DoorGroundTruth.actions
    actions = {"Opening": 0, "Closing": 1}

    def __init__(self, door_open_state: bool):
        """Create a door that can be opened and closed
        @param door_open_state - set to True if door is open, otherwise, false"""

        # GUIDE  Part 1: Store door state
        # YOUR CODE HERE

        # GUIDE  Part 3: Store probabilities of door being opened if open action taken, etc
        #           (the transition table)
        #.         - set to uniform probabilities initially
        # YOUR CODE HERE
        ...

    def set_probability(self, door_initial_state : bool, action : str, door_final_state : bool, prob : float):
        """Set the probability that the door will be in the final state if it started in the initial state and took
           the given action
           This is filling in the transition table 
        @param door_initial_state : boolean, if True, door started open
        @param action : one of "Opening" or "Closing" - action the robot took
        @param door_final_state : boolean, if True the door ends open
        @param prob - probability that the door will end in the given state, given the starting state and the action"""

        # Probability values are always between 0 and 1
        assert 0.0 <= prob <= 1.0

        # Action is one of "Opening" or "Closing" (checks the dictionary)
        assert action in DoorGroundTruth.actions

        # GUIDE: Part 3: Update your transition table
        # YOUR CODE HERE

    def robot_tries_to_open_door(self):
        """ The robot tries (once) to open the door, and succeeds (or fails) based on the probabilities in
        your transition table, the starting state, and the action (Opening)
        @ return the new state of the door"""

        # GUIDE:  Part 3: 
        #.  Randomly sample from the random variable for "open the door given the current door state" 
        #.    WHICH random variable depends on whether or not the door is actually open
        # (Possibly) change the state of the door to open and (possibly) change it to closed 
        #.  The latter can happen if there is some probability of the robot accidentally closing
        #.    The door when it is open 

        # YOUR CODE HERE

        return self.get_door_state()

    def robot_tries_to_close_door(self):
        """ The robot tries (once) to close the door, and succeeds (or fails) based on the probabilities"""

        # GUIDE: Part 3: 
        #.  Same as opening, but this time try closing 

        # YOUR CODE HERE

        return self.get_door_state()

    def get_door_state(self):
        """ GUIDE Part 1: Return the door state as a boolean (Open - True/Closed - False)"""
        # YOUR CODE HERE

    def __str__(self):
        """ Once you fill in get_door_state, this will print nicely """
        return f"Door state is: {self.get_door_state()}"


class DoorSensor():
    def __init__(self):
        # GUIDE: Part 2: Initialize probabilities to uniform probabilities
        #. I.e., whether or not the door is open or closed, 0.5 probability of saying door is open (or closed)
        #.  The methods will be used to set the probabilities to something other than uniform

        # YOUR CODE HERE
        ...

    def set_return_true_if_open_probability(self, prob: float):
        """ Set the probability of the sensor returning True if the door is open
        @param prob - the probability value (between 0 and 1) """

        # Probability values are always between 0 and 1
        assert 0.0 <= prob <= 1.0

        # GUIDE: Part 2: Set the random variable to the probability value
        # YOUR CODE HERE

    def set_return_false_if_closed_probability(self, prob: float):
        """ Set the probability of the sensor returning False if the door is closed
        @param prob - the probability value (between 0 and 1) """

        # Probability values are always between 0 and 1
        assert 0.0 <= prob <= 1.0

        # GUIDE: Set the random variable to the probability value
        # YOUR CODE HERE

    def sample_sensor(self, door_ground_truth : DoorGroundTruth):
        """ Sample the sensor
        @param door_ground_truth - has if the door is actually open or not
        @return True or False"""

        # GUIDE: Part 2: Simulate the sensor
        #. Reminder: There are two random variables here, one for if the door is open, one for if it
        #.  is closed. First determine which to sample from, THEN do the same thing you did in the
        #.  first problem in the jupyter notebook.
        #.
        # YOUR CODE HERE


class BeliefAboutDoor:
    def __init__(self):
        # Store the belief about the door being open/closed
        # Set it to be equally likely that the door is open/closed

        # YOUR CODE HERE
        self.reset_belief()
    
    def is_open_belief(self):
        """ Return your belief about the door being open"""
        # GUIDE: Should be a number between 0 and 1
        ...
        # YOUR CODE HERE
        
    def is_closed_belief(self):
        """ Return your belief about the door being closed"""
        # GUIDE: Should be a number between 0 and 1
        ...
        # YOUR CODE HERE
        
    def reset_belief(self):
        """ Set to 50/50 - equally likely door open/closed"""
        # GUIDE: Set your belief state to be 50/50
        # YOUR CODE HERE
        ...

    def update_belief_sensor(self, sensor : DoorSensor, sensor_reading : bool):
        """ Update the belief about the door being open/closed based on 
            the sensor reading
            @param sensor: You need this for the probabilities of the sensor being correct
            @param sensor_reading: This is the ACTUAL reading you got"""
        # GUIDE: One round of Bayes' sensor update
        # Remember, this is two equations/evaluations - one for updating the open belief, one for the closed
        # Do NOT update in place - should be something like
        #    new_belief = Blah
        #.   calculate new_belief from self.xxx (your current belief)
        #    Set self.xxx to be new_belief
        ...
        # YOUR CODE HERE

    def update_belief_action(self, door : DoorGroundTruth, action : str):
        """ Update the belief based on an action
        @param door - use this to get the transition probabilities
        @param action - the actual action, one of Opening or Closing"""
        ...
        # GUIDE Pick which set of transition probabilities based on the action
        #.  Then update the belief using the equation
        #.  Note: Update both open and closed belief, and each of those is the sum of two terms...
        # As before do
        #.    new belief = 0, 0
        #.    calculate new belief
        #.    set self.xxx to new belief
        
        # You'll probably want an if statement based on the action...
        ...
        # YOUR CODE HERE


# Check if the door and sensor are working correctly
def test_combo(prob_true_if_open: float, prob_false_if_closed: float):
    # Make the sensor
    door_sensor = DoorSensor()
    door_sensor.set_return_true_if_open_probability(prob=prob_true_if_open)
    door_sensor.set_return_false_if_closed_probability(prob=prob_false_if_closed)

    # Do both an open and a closed door
    n_total = 1000
    for b_door_gt, exp_val in zip([True, False], [prob_true_if_open, 1.0 - prob_false_if_closed]):
        # Make the door, with door either open or closed
        door_gt = DoorGroundTruth(door_open_state=b_door_gt)

        count_true = 0
        for _ in range(0, n_total):
            # Just count the true values
            if door_sensor.sample_sensor(door_gt):
                count_true += 1

        if not np.isclose(count_true / n_total, exp_val, atol=0.05):
            print(f"For door in state {door_gt}, expected {exp_val}, got {count_true / n_total}")
            return False

    return True

if __name__ == '__main__':

    # GUIDE: These are the tests; they're the same ones as in the jupyter notebook
    #.  Once the code for Part 1 is all written the first test will pass and so on
    
    # Tests, in order
    # part 1: Create a DoorGroundTruth instance and check that it has the door state set correctly

    door_start_open = DoorGroundTruth(True)
    door_start_closed = DoorGroundTruth(False)

    # You can print variables out, btw - or look in the variable window to see what the values are
    print(f"Door state open: {door_start_open}")
    print(f"Door state closed: {door_start_closed}")

    # Check
    assert door_start_open.get_door_state() == True
    assert door_start_closed.get_door_state() == False

    print("Part 1 passed")

    # Part 2: Check sensor; if the first one(s) work but the second don't, you have one of your if
    #. statements backwards
    assert test_combo(0.5, 0.5)
    assert test_combo(0.7, 0.5)
    assert test_combo(0.5, 0.8)
    assert test_combo(0.85, 0.71)

    print("Part 2 passed")

    # Part 3: Check actions
    #. See JN if you want to do just one of these checks
    n_samples = 200
    b_ret = True
    for door_start_state in [True, False]:
        for action in DoorGroundTruth.actions:
            for door_end_state in [True, False]:
                for prob in [0.0, 0.2, 0.8, 1.0]:
                    count_in_state = 0
                    for _ in range(0, n_samples):
                        my_door = DoorGroundTruth(door_open_state=door_start_state)
                        my_door.set_probability(door_initial_state=door_start_state,
                                                action=action,
                                                door_final_state=door_end_state,
                                                prob=prob)
                        if action == "Opening":
                            my_door.robot_tries_to_open_door()
                        else:
                            my_door.robot_tries_to_close_door()
                        if my_door.get_door_state() == door_end_state:
                            count_in_state += 1                    

                    prob_in_state = count_in_state / n_samples
                    if not np.isclose(prob_in_state, prob, atol=0.1):
                        print(f"Failed start state {door_start_state}, door end state {door_end_state}, action {action}")
                        print(f" Expected {prob}, got {prob_in_state}")
                        b_ret = False
    assert b_ret
    print(f"Part 3 passed")

    # Part 4 - GUIDE: fill in the methods for BeliefAboutDoor class
    #. Side note - this only checks the values for the slides in class. It does not check
    #.  that you correctly update the belief if the door sensor returns false or you open the door instead....
    belief = BeliefAboutDoor()

    assert np.isclose(belief.is_open_belief(), 0.5)
    assert np.isclose(belief.is_closed_belief(), 0.5)

    # Example from slides
    door_sensor1 = DoorSensor()
    door_sensor1.set_return_true_if_open_probability(0.6)
    door_sensor1.set_return_false_if_closed_probability(0.7)
    
    # Update with first sensor
    belief.update_belief_sensor(door_sensor1, True)
    assert np.isclose(belief.is_open_belief(), 2.0 / 3.0)
    assert np.isclose(belief.is_closed_belief(), 1.0 / 3.0)

    door_sensor2 = DoorSensor()
    door_sensor2.set_return_true_if_open_probability(0.5)
    door_sensor2.set_return_false_if_closed_probability(0.4)

    belief.update_belief_sensor(door_sensor2, True)
    assert np.isclose(belief.is_open_belief(), 5.0 / 8.0)
    assert np.isclose(belief.is_closed_belief(), 3.0 / 8.0)

    # Now take the action
    door_example_probs = DoorGroundTruth(True)
    # Sets two of the arrows for closing the door - the other two
    #.  should be set by taking 1.0 - x in your code
    door_example_probs.set_probability(True, "Closing", True, 0.1)
    door_example_probs.set_probability(False, "Closing", False, 1.0)

    belief.update_belief_action(door_example_probs, "Closed")
    assert np.isclose(belief.is_open_belief(), 1.0 / 16.0)
    assert np.isclose(belief.is_closed_belief(), 15.0 / 16.0)

    