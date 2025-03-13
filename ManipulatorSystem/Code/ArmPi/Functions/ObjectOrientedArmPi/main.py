#!/usr/bin/python3
# coding=utf8

import time

from Perception import Perception
from Motion import Motion

def main(stacking):
    # Start the perception object
    perception = Perception()
    perception.start()
    # Start the motion object
    motion = Motion(reset_time_ms=650)
    motion.start()

    # Continuous loop
    while True:
        # get the labelled frame and position dictionary from running the perception object
        frame_labelled, position_dictionary = perception.run()
        # restart the loop if the frame was not captured
        if(frame_labelled is None):
            # stall a bit to not use resources
            time.sleep(0.01)
            continue
        # If there are no objects in the field, then reset the robot and wait
        if(len(position_dictionary) == 0):
            motion.reset_position()
            continue
        # code to run when stacking
        if(stacking):
            # Get the stack index from the number of objects on the field
            stack_index = 3 - len(position_dictionary)
            # Get the first instance in the color-position dictionary to check the status of the field again
            color = list(position_dictionary.keys())[0]
            # move the arm to stack the colors
            print(f"Getting {color}")
            motion.run_stack(stack_index, position_dictionary[color], speed=3.5)
        # code to run when sorting
        else:
            # Get the stack index from the number of objects on the field
            stack_index = 3 - len(position_dictionary)
            # Get the first instance in the color-position dictionary to check the status of the field again
            color = list(position_dictionary.keys())[0]
            # move the arm to stack the colors
            print(f"Getting {color}")
            motion.run_sort(color, position_dictionary[color], speed=5.0)


if(__name__ == "__main__"):
    # run main with stacking option; False=sorting
    main(stacking=True)