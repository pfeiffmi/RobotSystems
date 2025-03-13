#!/usr/bin/python3
# coding=utf8

# specify path for other required files
import sys
sys.path.append('/home/pi/RobotSystems/ManipulatorSystem/Code/ArmPi/')

# import python libraries
import cv2
import time
import Camera
import math
import numpy as np

# import local files
import LABConfig
from ArmIK import Transform
import CameraCalibration.CalibrationConfig

# Class for detection of colored cubes in the camera view
class Perception():
    def __init__(self):
        #
        self.size = (640, 480)
        self.square_length = CameraCalibration.CalibrationConfig.square_length

        #
        self.color_list = ("red", "green", "blue")
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

    def _get_position_vector(self, rect, roi):
        # get the x, y centroid with respect to the world coordinate system (origin is center-camera)
        img_centerx, img_centery = Transform.getCenter(rect, roi, self.size, self.square_length)
        world_x, world_y = Transform.convertCoordinate(img_centerx, img_centery, self.size)
        # get the width and height
        width, height = rect[1]
        rotation_angle = rect[2]
        # Define the position vector
        position_vector = ((world_x, world_y), (width, height), rotation_angle)
        # return the position vector
        return(position_vector)

    def draw_bb_on_image(self, img, box, position_vector, color):
        # Get the color to draw
        draw_color = self.range_rgb[color]
        # Draw the contour as defined by the bb
        cv2.drawContours(img, [box], -1, draw_color, 2)
        # draw 
        cv2.putText(
            img,
            f"({position_vector[0][0]:.2f}, {position_vector[0][1]:.2f}), ({position_vector[1][0]:.2f}, {position_vector[1][1]:.2f}), {position_vector[2]:.2f}",
            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            draw_color, 
            1
        )

    def run_color(self, img, color):
        # preproccess the image to be better suited for color detection
        preprocessed_img = self._preprocess_img(img.copy())
        # get the mask using the specific color from the pre-processed image
        img_color_mask = self._get_img_color_mask(preprocessed_img, color)
        # get the max contour for the max
        contour, area = self._get_largest_area_contour(img_color_mask)
        
        # proceed if the area of the max contour is larger than 2500 square pixels
        if(area > 2500):
            # Compute the bounding box
            rect = cv2.minAreaRect(contour)
            box = np.int0(cv2.boxPoints(rect))
            roi = Transform.getROI(box)
            # compute the position vector
            position_vector = self._get_position_vector(rect, roi)
            # draw the bounding box on the image
            self.draw_bb_on_image(img, box, position_vector, color)
        
        # if contour too small, specify a Null position vector
        else:
            position_vector = None
        
        # return the image and the position vector
        return(img, position_vector)
    
    def run(self):
        # Initiallize the position dictionary
        position_dict = dict()
        # get the frame
        frame = self.get_frame()
        # check if it is a valid frame
        if(frame is not None):
            # loop through each specified color
            for color in self.color_list:
                # get the labeled frame and position of the perception code for each object
                frame, position = self.run_color(frame, color)
                if(position is not None):
                    position_dict[color] = position
        # return the labelled frame and position dictionary
        return(frame, position_dict)

    def __del__(self):
        # close the camera on object destruction
        self.camera.camera_close()


# main function to test perception class
def main():
    # Start the perception object
    perception = Perception()
    perception.start()

    # Continuous loop
    while True:
        # get the labelled frame and position dictionary from running the perception object
        frame_labelled, position_dictionary = perception.run()
        # restart the loop if the frame was not captured
        if(frame_labelled is None):
            # stall a bit to not use resources
            time.sleep(0.01)
            continue
        # Display the frame (press [ESC] to quit)
        cv2.imshow('Frame (Labelled)', frame_labelled)
        key = cv2.waitKey(1)
        if key == 27:
            break

    # Destroy the CV2 window
    cv2.destroyAllWindows()


# Specify execution of main function when file is ran directly
if __name__ == '__main__':
    main()