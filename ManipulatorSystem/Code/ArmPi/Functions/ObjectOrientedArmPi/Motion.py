#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/RobotSystems/Code/ArmPi/')
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

        # Settings for the arm's motion
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

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

    def run(img):
        global roi
        global rect
        global count
        global get_roi
        global center_list
        global unreachable
        global __isRunning
        global start_pick_up
        global rotation_angle
        global last_x, last_y
        global world_X, world_Y
        global start_count_t1, t1
        global detect_color, draw_color, color_list
        
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not __isRunning:
            return img

        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
        if get_roi and not start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, roi, size)      
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
        
        if not start_pick_up:
            for i in color_range:
                if i in __target_color:
                    frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  #对原图像和掩模进行位运算
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  #开运算
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) #闭运算
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
                    areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
                    if areaMaxContour is not None:
                        if area_max > max_area:#找最大面积
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 2500:  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour_max)
                box = np.int0(cv2.boxPoints(rect))
                
                roi = getROI(box) #获取roi区域
                get_roi = True
                img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
                
                world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标
                
                cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
                cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color_area_max], 1) #绘制中心点
                
                distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
                last_x, last_y = world_x, world_y
                if not start_pick_up:
                    if color_area_max == 'red':  #红色最大
                        color = 1
                    elif color_area_max == 'green':  #绿色最大
                        color = 2
                    elif color_area_max == 'blue':  #蓝色最大
                        color = 3
                    else:
                        color = 0
                    color_list.append(color)
                    # 累计判断
                    if distance < 0.5:
                        count += 1
                        center_list.extend((world_x, world_y))
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1:
                            rotation_angle = rect[2] 
                            start_count_t1 = True
                            world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                            center_list = []
                            count = 0
                            start_pick_up = True
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        center_list = []
                        count = 0

                    if len(color_list) == 3:  #多次判断
                        # 取平均值
                        color = int(round(np.mean(np.array(color_list))))
                        color_list = []
                        if color == 1:
                            detect_color = 'red'
                            draw_color = range_rgb["red"]
                        elif color == 2:
                            detect_color = 'green'
                            draw_color = range_rgb["green"]
                        elif color == 3:
                            detect_color = 'blue'
                            draw_color = range_rgb["blue"]
                        else:
                            detect_color = 'None'
                            draw_color = range_rgb["black"]
            else:
                if not start_pick_up:
                    draw_color = (0, 0, 0)
                    detect_color = "None"

        cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
        return img
    