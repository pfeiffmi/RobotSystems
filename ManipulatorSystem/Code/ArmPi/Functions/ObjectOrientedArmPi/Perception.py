import sys
sys.path.append('/home/pi/RobotSystems/Code/ArmPi/')
import cv2
import time
import Camera

import threading
import LABConfig
import ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

class Perception():
    def __init__(self):
        #
        self.roi = None
        self.rect = None
        self.count = None
        self.get_roi = None
        self.center_list = None
        self.unreachable = None
        self.__isRunning = None
        self.start_pick_up = None
        self.rotation_angle = None
        self.last_x = None
        self.last_y = None
        self.world_X = None
        self.world_Y = None
        self.start_count_t1 = None
        self.t1 = None
        self.detect_color = None
        self.draw_color = None
        self.color_list = None

        #
        self.__target_colors = ("red", "blue", "green")
        self.camera = Camera.Camera()

    def start(self):
        self.camera.camera_open()

    def _get_frame(self):
        return(self.camera.frame.copy())

    def preprocess_img(self, img):
        # Create a copy of the image and save its dimensions
        img_h, img_w = img.shape[:2]
        # Create two crossing lines to calibrate the image to the center of the world
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        # resize the image
        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        # blur the image to reduce noise effects
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        #return the pre-processed image
        return(frame_gb)

    def get_img_color_mask(self, img, color):
        # look for colors in the camera's view
        img_mask = getMaskROI(img, roi, size)
        # convert the image's color space to LAB
        img_mask = cv2.cvtColor(img_roi, cv2.COLOR_BGR2LAB)
        # get the image mask of the specified color
        frame_mask = cv2.inRange(img_mask, LABConfig.color_range[color][0], LABConfig.color_range[color][1])
        # neaten mask by removing small active mask points that are not in larger group (morph open)
        frame_mask = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))
        # neaten mask by filling small inactive mask points that are in larger group (morph close)
        frame_mask = cv2.morphologyEx(frame_mask, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8))
        # return the image mask
        return(frame_mask)


    def run(self, img, color):
        # preproccess the image to be better suited for color detection
        preprocessed_img = self.preprocess_img(img)

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
        
        # ====Code for pickup not initiated yet======
        img_color_mask = self.get_img_color_mask(preprocessed_img, color)
        
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓
        
        if area_max > 2500:  # 有找到最大面积
            rect = cv2.minAreaRect(areaMaxContour)
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
        #=== end code that is ran if pickup not initiated===

        cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
        return img
    

    def __del__(self):
        self.camera.camera_close()