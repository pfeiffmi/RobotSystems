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
    def __init__(self, method="ultrasonic", timeout=0.02):
        # Set the distance sensor method
        self.method = method

        # Untrasonic distance sensor
        if(self.method == "ultrasonic"):
            # Define the timeout
            self.timeout = timeout
            # Define the speed of sound in m per second
            self.speed_of_sound_m_per_sec = 343.3
            # Define ultrasonic pins
            self.trig = Pin("D2")
            self.echo = Pin("D3", mode=Pin.IN, pull=Pin.PULL_DOWN)
            # Ensure ultrasonic pins are reset closed
            self.trig.close()
            self.echo.close()
        
        # invalid method specified
        else:
            raise(Exception("Invalid sensor method used"))
        

    def read_data(self):
        # case for reading ultrasonic data
        if(self.method == "ultrasonic"):
            # read ultrasonic sensor data 10 times until a proper value is read
            for i in range(10):
                # read the ultrasonic distance sensor data
                data = self.read_ultrasonic_data()
                # Case if timeout reached
                if(data == -1):
                    continue
                # case if echo pin error occured
                elif(data == -2):
                    # break out of loop
                    break
                # case is proper ultrasonic sensor data read
                else:
                    #return data
                    return(data)
            
            # return error code if echo pin not working or all readings occur in timeout
            return(-2)
        
        # invalid method to read data from
        else:
            raise(Exception("Invalid sensor method used"))
        
        # Return the read data
        return(data)
    

    # Code modified from robot_hat/modules.py:Ultrasonic class
    def read_ultrasonic_data(self):
        # Send an initial pulse
        self.trig.off()
        time.sleep(0.001)
        self.trig.on()
        time.sleep(0.00001)
        self.trig.off()
        
        # Start timer for the timeout for the ultrasonic sensor
        timeout_start = time.time()
        
        # Loop while waiting for trig pin to send the initial pulse (will set echo pin to high when signal is sent)
        pulse_start = 0
        while(self.echo.gpio.value == 0):
            # Update the start time until the pulse is sent and we exit the loop
            pulse_start = time.time()
            # Case to determine if timeout has been reached
            if(pulse_start - timeout_start > self.timeout):
                return(-1)
        
        # Loop while waiting for the detected pulse to end (echo pin will be high until it measures the pulse)
        pulse_end = 0
        while(self.echo.gpio.value == 1):
            # Update the endtime until the pulse is received and we exit the loop
            pulse_end = time.time()
            # Case to determine if timeout has been reached
            if(pulse_end - timeout_start > self.timeout):
                return(-1)
        
        # Check to ensure nothing wrong happend with setting the echo pin (echo had pin value of 0 then 1)
        if(pulse_start == 0 or pulse_end == 0):
            return(-2)
        
        # Compute the time-of-flight (duration) of the pulse
        time_of_flight = pulse_end - pulse_start

        # Compute a distance from the pulse time-of-flight
        distance_m = time_of_flight*(self.speed_of_sound_m_per_sec/2)
        distance_cm = 100*distance_m
        return(distance_cm)
        