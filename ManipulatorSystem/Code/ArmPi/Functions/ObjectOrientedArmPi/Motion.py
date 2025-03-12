#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/RobotSystems/ManipulatorSystem/Code/ArmPi/')
import cv2
import time
import Camera
import threading

from LABConfig import *

from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

class Motion():
    def __init__(self):
        # Declare class variabels
        self.count = None
        self.track = None
        self._stop = None
        self.get_roi = None
        self.first_move = None
        self.center_list = None
        self.__isRunning = None
        self.detect_color = None
        self.action_finish = None
        self.start_pick_up = None
        self.__target_color = None
        self.start_count_t1 = None

        #
        self.AK = ArmIK()

        # Settings for the arm's motion
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

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
        # note that the robot is currently running
        self.__isRunning = True

    
    def move():
        global rect
        global _stop
        global get_roi
        global unreachable
        global __isRunning
        global detect_color
        global start_pick_up
        global rotation_angle
        global world_X, world_Y
        
        #放置坐标
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        while True:
            if __isRunning:        
                if detect_color != 'None' and start_pick_up:  #如果检测到方块没有移动一段时间后，开始夹取
                    #移到目标位置，高度6cm, 通过返回的结果判断是否能到达指定位置
                    #如果不给出运行时间参数，则自动计算，并通过结果返回
                    set_rgb(detect_color)
                    setBuzzer(0.1)
                    result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  
                    if result == False:
                        unreachable = True
                    else:
                        unreachable = False
                        time.sleep(result[2]/1000) #如果可以到达指定位置，则获取运行时间

                        if not __isRunning:
                            continue
                        servo2_angle = getAngle(world_X, world_Y, rotation_angle) #计算夹持器需要旋转的角度
                        Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)
                        
                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                        time.sleep(1.5)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1, 500)  #夹持器闭合
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起
                        time.sleep(1)

                        if not __isRunning:
                            continue
                        result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not __isRunning:
                            continue                   
                        servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not __isRunning:
                            continue                    
                        AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开  ，放下物体
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        initMove()  # 回到初始位置
                        time.sleep(1.5)

                        detect_color = 'None'
                        get_roi = False
                        start_pick_up = False
                        set_rgb(detect_color)
            else:
                if _stop:
                    _stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)