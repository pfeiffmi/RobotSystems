import sys
sys.path.append('/home/pi/RobotSystems/ManipulatorSystem/Code/ArmPi/')
import cv2
import time
import Camera

import threading
import LABConfig
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
import CameraCalibration.CalibrationConfig

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

        self.size = (640, 480)
        self.square_length = CameraCalibration.CalibrationConfig.square_length

        #
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        #
        self.camera = Camera.Camera()

    def start(self):
        self.camera.camera_open()

    def get_frame(self):
        return(self.camera.frame)

    def getAreaMaxContour(self, contours):
        # define local variables
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None
        # loop through all contours
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c
        # return the contour and area of the max area contour
        return(area_max_contour, contour_area_max)

    def _preprocess_img(self, img):
        # Create a copy of the image and save its dimensions
        img_h, img_w = img.shape[:2]
        # Create two crossing lines to calibrate the image to the center of the world
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        # resize the image
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        # blur the image to reduce noise effects
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        #return the pre-processed image
        return(frame_gb)

    def _get_img_color_mask(self, img, color):
        # convert the image's color space to LAB
        img_mask = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        # get the image mask of the specified color
        frame_mask = cv2.inRange(img_mask, LABConfig.color_range[color][0], LABConfig.color_range[color][1])
        # neaten mask by removing small active mask points that are not in larger group (morph open)
        frame_mask = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))
        # neaten mask by filling small inactive mask points that are in larger group (morph close)
        frame_mask = cv2.morphologyEx(frame_mask, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8))
        # return the image mask
        return(frame_mask)

    def _get_largest_area_contour(self, mask):
        # get all the contours in the mask
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        # get the contour with the largest area
        max_area_contour, max_area = self.getAreaMaxContour(contours)
        # return the contour with the largest area and its corresponding area
        return(max_area_contour, max_area)

    def run(self, img, color):
        # preproccess the image to be better suited for color detection
        preprocessed_img = self._preprocess_img(img.copy())
        # get the mask using the specific color from the pre-processed image
        img_color_mask = self._get_img_color_mask(preprocessed_img, color)
        # get the max contour for the max
        contour, area = self._get_largest_area_contour(img_color_mask)
        
        # proceed if the area of the max contour is larger than 2500 square pixels
        if(area > 2500):

            self.rect = cv2.minAreaRect(contour)
            self.box = np.int0(cv2.boxPoints(self.rect))
            self.roi = getROI(self.box)

            img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, self.square_length)
            world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)
            
            draw_color = self.range_rgb[color]

            cv2.drawContours(img, [self.box], -1, draw_color, 2)
            
            cv2.putText(
                img, 
                '(' + str(world_x) + ',' + str(world_y) + ')', 
                (min(self.box[0, 0], self.box[2, 0]), self.box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                draw_color, 
                1
            )

            rotation_angle = self.rect[2] 
        else:
            draw_color = (0,0,0)
            color = "None"
        
        #cv2.rectangle(img, (5, img.shape[0] - 30), (150, img.shape[0] - 5), (255, 255, 255), -1)
        #cv2.putText(img, "Color: " + color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
        return img
    
    def __del__(self):
        self.camera.camera_close()


if __name__ == '__main__':
    perception = Perception()
    perception.start()

    color_list = ("red", "green", "blue")

    while True:
        frame = perception.get_frame()
        if(frame is not None):
            for color in color_list:
                frame = perception.run(frame, color)
            
            cv2.imshow('Frame', frame)
            
            key = cv2.waitKey(1)
            if key == 27:
                break
    
    cv2.destroyAllWindows()