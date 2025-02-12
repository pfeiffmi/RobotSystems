import time
import numpy as np
import os
import sys

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

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

class Controller():
    def __init__(self, max_turn_angle=30, init_turn_angle=0, init_tilt_angle=0):
        # Save the max turn angle
        self.max_turn_angle=max_turn_angle
        # Define the turn servo
        self.turn_servo = Servo("P2")
        self.turn_servo.angle(init_turn_angle)
        # Define the tilt angle
        self.tilt_servo = Servo("P1")
        self.tilt_servo.angle(init_tilt_angle)

    
    def set_turn_proportion(self, turn_proportion):
        turn_angle = float(self.max_turn_angle*turn_proportion)
        self.turn_servo.angle(turn_angle)

    
    def consumer(self, producer_bus_instance, delay_sec):
        while(True):
            self.set_turn_proportion(producer_bus_instance.read())
            time.sleep(delay_sec)