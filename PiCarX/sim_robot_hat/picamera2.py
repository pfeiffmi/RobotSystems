import cv2
import numpy as np

class Picamera2():
    def __init__(self):
        self.cam = None
        self.frame_width = None
        self.frame_height = None
        self.configuration = None

    def create_preview_configuration(self):
        return(0)
    
    def configure(self, camera_config):
        self.configuration = camera_config

    def start(self):
        self.cam = cv2.VideoCapture(0)
        # Get the default frame width and height
        self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def capture_array(self, video_feed):
        frame = None
        if(video_feed == "main"):
            ret, frame = self.cam.read()
        
        else:
            dim = (self.frame_height, self.frame_width)
            frame = np.zeros(dim)
        
        return(frame)

    def __del__(self):
        self.cam.release()
