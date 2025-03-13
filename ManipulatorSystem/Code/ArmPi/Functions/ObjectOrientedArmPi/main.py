#!/usr/bin/python3
# coding=utf8

import time

from Perception import Perception
from Motion import Motion

def main():
    # Start the perception object
    perception = Perception()
    perception.start()
    # Start the motion object
    motion = Motion()
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
        # move the arm to sort the colors
        for color in position_dictionary.keys():
            print(f"Getting {color}")
            motion.run(color, position_dictionary[color])

if(__name__ == "__main__"):
    main()