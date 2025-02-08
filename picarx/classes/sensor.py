import time
import numpy as np
from picamera2 import Picamera2, Preview
import cv2

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
    on_robot = True
    reset_mcu()
    time.sleep(0.2)

except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    #from sim_robot_hat.utils import reset_mcu, run_command
    #from logdecorator import log_on_start, log_on_end, log_on_error
    on_robot = False

class Sensor():
    def __init__(self, method="grayscale"):
        # set adc structure using self.syntax
        self.method = method
        if(self.method == "grayscale"):
            self.adc = [ADC(0), ADC(1), ADC(2)]

            self.rotation_index = 0
            self.num_samples = 3
            self.num_sensors = 3
            self.grayscale_data = np.zeros((self.num_samples, self.num_sensors))

            for i in range(self.num_samples):
                for j in range(self.num_sensors):
                    self.grayscale_data[i, j] = self.adc[j].read()
        
        elif(self.method == "vision"):
            self.camera = Picamera2()
            camera_config = self.camera.create_preview_configuration()
            self.camera.configure(camera_config)
            #self.camera.start_preview(Preview.DRM)
            self.camera.start()
            time.sleep(2)
            self.image = np.array(self.camera.capture_array("main"))
            self.image_dims = (400, 500)
            self.image = cv2.resize(self.image, self.image_dims)
            self.image = cv2.GaussianBlur(self.image, ksize=(13,13), sigmaX=0)
            
    
    def read_data(self):
        if(self.method == "grayscale"):
            data = self.read_grayscale_data()
        elif(self.method == "vision"):
            data = self.read_vision_data()
        return(data)
        

    def read_grayscale_data(self):
        # get grayscale values
        for i in range(self.num_sensors):
            self.grayscale_data[self.rotation_index, i] = self.adc[i].read()

        self.rotation_index += 1
        self.rotation_index %= self.num_samples

        data = np.mean(self.grayscale_data, axis=0)
        print("data:", data)
        # return values
        return(data)


    def read_vision_data(self):
        self.image = self.camera.capture_array("main")
        
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.image = cv2.resize(self.image, self.image_dims)
        self.image = cv2.GaussianBlur(self.image, ksize=(13,13), sigmaX=0)
        
        _, self.image = cv2.threshold(self.image, thresh=100, maxval=255, type=cv2.THRESH_BINARY_INV)

        data = np.mean(self.image[self.image_dims[0]-50:self.image_dims[0], :], axis=0)
        
        return(data)
    

    def show_camera_image(self):
        while(True):
            self.read_vision_data()
            cv2.imshow('Grayscale', self.image)
            key = cv2.waitKey(0)
            if(key == 27):
                break


    def __del__(self):
        cv2.destroyAllWindows()