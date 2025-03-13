#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/RobotSystems/ManipulatorSystem/Code/ArmPi/')

import time
import numpy as np

from ArmIK import Transform
from ArmIK import ArmMoveIK
import HiwonderSDK.Board as Board


class Motion():
    def __init__(self):
        # define how far up to grab the cube
        self.cube_grab_from_bottom_cm = 1.0
        # coordinates for colored cube placement
        self.home_coordinate = {
            "red":   (-15.5, 12.5, self.cube_grab_from_bottom_cm),
            "green": (-15.5, 6.5,  self.cube_grab_from_bottom_cm),
            "blue":  (-15.5, 0.5,  self.cube_grab_from_bottom_cm),
        }
        # set an offset for the world coordinates
        self.offset_world_xy = np.array([1.5, 0.25])
        # arm inverse kinematics object
        self.AK = ArmMoveIK.ArmIK()
        # Settings for the arm's motion
        self.servo1 = 500
        # reset the arms position
        self.reset_position()

    def _reset(self):
        # Initiallize class variables
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

    def start(self):
        # reset the class variables
        self._reset()
        self.reset_position()

    def reset_position(self, reset_time_ms=1500):
        Board.setBusServoPulse(1, self.servo1 - 50, min(300, reset_time_ms))
        Board.setBusServoPulse(2, 500, min(500, reset_time_ms))
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, reset_time_ms)
        time.sleep(reset_time_ms/1000)

    def run(self, color_of_interest, position_vector, speed=1.0):
        # ensure that the color or position vector is not 'None'
        if((color_of_interest == "None") or (position_vector is None)):
            return
        # get the individual measures from the position vector
        world_X, world_Y = np.array(position_vector[0]) + self.offset_world_xy
        width, height = position_vector[1]
        rotation_angle = position_vector[2]
        # get color home coordinates
        color_home_X, color_home_Y, color_home_Z = self.home_coordinate[color_of_interest]
        
        # phase 0 - move end-effector to hover 7cm above the object
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (world_X, world_Y, 7), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0,
            movetime = int(1500/speed)
        )
        time.sleep(movetime_ms/1000)

        # phase 1 - rotate the end-effector to match the cube rotation and open the claw
        servo2_angle = Transform.getAngle(world_X, world_Y, rotation_angle) #calculate gripper angle rotation
        Board.setBusServoPulse(1, self.servo1 - 280, int(500/speed))  # open claw
        Board.setBusServoPulse(2, servo2_angle, int(500/speed))
        time.sleep(0.5)
        
        # phase 2 - move toward object, with end-effector being half the height of the block (1.5 cm high)
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (world_X, world_Y, self.cube_grab_from_bottom_cm), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0, 
            movetime = int(1000/speed)
        )
        time.sleep(movetime_ms/1000)

        # phase 3 - close the claw
        Board.setBusServoPulse(1, self.servo1, int(500/speed))  # close claw
        time.sleep(0.8/speed)

        # phase 4 - raise arm up with cube in hand to 12 cm high
        Board.setBusServoPulse(2, 500, int(500/speed))
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (world_X, world_Y, 12), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0, 
            movetime = int(1000/speed)
        )
        time.sleep(movetime_ms/1000)

        # phase 5 - move arm with cube 12 cm above the color's home position
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (color_home_X, color_home_Y, 12), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0,
            movetime = int(1500/speed)
        )
        time.sleep(movetime_ms/1000)
        
        # phase 6 - adjust the claw's hand angle to line up with the home base
        servo2_angle = Transform.getAngle(color_home_X, color_home_Y, -90)
        Board.setBusServoPulse(2, servo2_angle, int(500/speed))
        time.sleep(0.5/speed)

        # phase 7 - move arm 3 inches above normal cube home postion
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (color_home_X, color_home_Y, color_home_Z+3), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0, 
            movetime = int(500/speed)
        )
        time.sleep(movetime_ms/1000)
        
        # phase 8 - slowly move cube to its home position
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (color_home_X, color_home_Y, color_home_Z), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0, 
            movetime = int(1000/speed)
        )
        time.sleep(movetime_ms/1000)

        # phase 9 - open the claw
        Board.setBusServoPulse(1, self.servo1 - 200, int(500/speed))
        time.sleep(0.8/speed)

        # phase 10 - move to 12 cm above home position
        servos, alpha, movetime_ms = self.AK.setPitchRangeMoving(
            coordinate_data = (color_home_X, color_home_Y, 12), 
            alpha = -90, 
            alpha1 = -90, 
            alpha2 = 0, 
            movetime = int(800/speed)
        )
        time.sleep(movetime_ms/1000)

        # phase 11 - reset the arm position to its default state
        self.reset_position(reset_time_ms=750)
        time.sleep(1.5)