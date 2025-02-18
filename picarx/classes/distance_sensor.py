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
    def __init__(self, method="ultrasonic", timeout=0.01):
        # Set the distance sensor method
        self.method = method

        # Untrasonic distance sensor
        if(self.method == "ultrasonic"):
            # Define the timeout
            self.timeout = timeout
            # Define the speed of sound in m per second
            self.speed_of_sound_m_per_sec = 343.3
            # Define the max sensor reading (the largest distance possible in cm with the given timeout)
            self.max_distance_reading_cm = 100*(timeout*self.speed_of_sound_m_per_sec)
            # Define ultrasonic pins
            self.trig = Pin("D2")
            self.echo = Pin("D3", mode=Pin.IN, pull=Pin.PULL_DOWN)
        
        # invalid method specified
        else:
            raise(Exception("Invalid sensor method used"))
        

    def read_data(self):
        # case for reading ultrasonic data
        if(self.method == "ultrasonic"):
            data = self.read_avg_ultrasonic_data(num_samples=7)
        
        # invalid method to read data from
        else:
            raise(Exception("Invalid sensor method used"))
        
        # Return the read data
        return(data)


    def read_avg_ultrasonic_data(self, num_samples):
        # read ultrasonic sensor data 10 times until a proper value is read
        total_distance = 0
        for i in range(2*num_samples):
            # read the ultrasonic distance sensor data
            data = self.read_ultrasonic_data()
            # Case if timeout reached: assume no wall ahead
            if(data == -1):
                data = self.max_distance_reading_cm
            # case if echo pin error occured: assume there is a wall right at head
            elif(data == -2):
                data = 0
            # sum the distance reading (only on even index readings as every other sensor reading is corrupt for some reason)
            print(i, data)
            if(i % 2 == 0):
                total_distance += data
            else:
                continue
        
        print("-----")
        # return the average reading
        average_reading = total_distance/num_samples
        return(average_reading)


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
        

if(__name__ == "__main__"):
    sensor = DistanceSensor()
    while(True):
        data = sensor.read_data()
        print(data)
        time.sleep(0.3)
