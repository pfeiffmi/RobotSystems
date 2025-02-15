import time
import numpy as np
import cv2
import os
import sys

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
    from picamera2 import Picamera2
    on_robot = True
    reset_mcu()
    time.sleep(0.2)

except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat.picamera2 import Picamera2
    #from sim_robot_hat.utils import reset_mcu, run_command
    #from logdecorator import log_on_start, log_on_end, log_on_error
    on_robot = False

class DistanceSensor():
    def __init__(self, method="sonar"):
        self.method = method
        if(self.method == "sonar"):
            pass
            """
            self.adc = [ADC(0), ADC(1), ADC(2)]

            self.rotation_index = 0
            self.num_samples = 3
            self.num_sensors = 3
            self.grayscale_data = np.zeros((self.num_samples, self.num_sensors))

            for i in range(self.num_samples):
                for j in range(self.num_sensors):
                    self.grayscale_data[i, j] = self.adc[j].read()
            """
        
        else:
            raise(Exception("Invalid sensor method used"))
        
    def read_data(self):
        if(self.method == "sonar"):
            data = self.read_sonar_data()
        
        else:
            raise(Exception("Invalid sensor method used"))
        
        return(data)
    
    def read_sonar_data(self):
        return(10)