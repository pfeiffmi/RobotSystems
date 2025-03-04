import sys
sys.path.append('/home/pi/RobotSystems/Code/ArmPi/')
import cv2
import time
import Camera

class Perception():
    def __init__(self):
        self.camera = Camera.Camera()

    def start(self):
        self.camera.camera_open()

    def get_frame(self):
        pass

    def __del__(self):
        self.camera.camera_close()