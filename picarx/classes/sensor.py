import time
import numpy as np

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
    def __init__(self):
        # set adc structure using self.syntax
        self.adc0 = ADC(0)
        self.adc1 = ADC(1)
        self.adc2 = ADC(2)

        self.grayscale_data = np.zeros(3)
        
        self.data = None
    
    def read_data(self):
        # get grayscale values
        self.grayscale_data[0] = self.adc0.read()
        self.grayscale_data[1] = self.adc1.read()
        self.grayscale_data[2] = self.adc2.read()
        # return values
        return(self.grayscale_data)
        