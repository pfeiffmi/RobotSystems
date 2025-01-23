import time
import os
import math

import sys

import atexit

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
    on_robot = False
    
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
import atexit

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# @log_on_start(logging.DEBUG, "{name:s}: Message when function starts")
# @log_on_end(logging.DEBUG, "Message when function ends successfully: {result!r}")
def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    # @log_on_start(logging.DEBUG, "init: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        #utils.reset_mcu()
        #time.sleep(0.2)

        # --------- config_flie ---------
        if(on_robot):
            self.config_flie = fileDB(config, 777, os.getlogin())
        else:
            self.config_flie = fileDB(config, 777, None)

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))

        # --------- Exit command -------------
        if(on_robot):
            atexit.register(self.set_motor_speed, 1, 0)
            atexit.register(self.set_motor_speed, 2, 0)
        else:
            atexit.register(logging.debug, f"setting_motor_speed: motor({1}, {0})")
            atexit.register(logging.debug, f"setting_motor_speed: motor({2}, {0})")
        

    # @log_on_start(logging.DEBUG, "set_motor_speed: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # Remove speed scaling (commented code below)
        # if speed != 0:
        #    speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if(on_robot):
            if direction < 0:
                self.motor_direction_pins[motor].high()
                self.motor_speed_pins[motor].pulse_width_percent(speed)
            else:
                self.motor_direction_pins[motor].low()
                self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            logging.debug(f"Setting motor speed: motor({motor}, {speed})")


    # @log_on_start(logging.DEBUG, "motor_speed_calibration: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0


    # @log_on_start(logging.DEBUG, "motor_direction_calibrate: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)


    # @log_on_start(logging.DEBUG, "dir_servo_calibrate: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)


    # @log_on_start(logging.DEBUG, "set_dir_servo_angle: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)


    # @log_on_start(logging.DEBUG, "cam_pan_servo_calibrate: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)


    # @log_on_start(logging.DEBUG, "cam_tilt_servo_calibrate: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)


    # @log_on_start(logging.DEBUG, "set_cam_pan_angle: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))


    # @log_on_start(logging.DEBUG, "set_cam_tilt_angle: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))


    # @log_on_start(logging.DEBUG, "set_power: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def generate_power_scale_value(self, turning_angle):
        turning_angle_deg = turning_angle*(180.0/math.pi)
        power_scale = math.cos(turning_angle_deg)
        return(power_scale)


    # @log_on_start(logging.DEBUG, "backward: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = self.generate_power_scale_value(turning_angle=abs_current_angle)
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  


    # @log_on_start(logging.DEBUG, "forward: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            #power_scale = (100 - abs_current_angle) / 100.0
            power_scale = self.generate_power_scale_value(turning_angle=abs_current_angle)
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)


    # @log_on_start(logging.DEBUG, "stop: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)


    # @log_on_start(logging.DEBUG, "get_distance: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def get_distance(self):
        return self.ultrasonic.read()


    # @log_on_start(logging.DEBUG, "set_grayscale_reference: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")


    # @log_on_start(logging.DEBUG, "get_grayscale_data: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())


    # @log_on_start(logging.DEBUG, "get_line_status: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)


    # @log_on_start(logging.DEBUG, "set_line_reference: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_line_reference(self, value):
        self.set_grayscale_reference(value)


    # @log_on_start(logging.DEBUG, "get_cliff_status: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False


    # @log_on_start(logging.DEBUG, "set_cliff_reference: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")


    # @log_on_start(logging.DEBUG, "reset: Message when function starts")
    # @log_on_end(logging.DEBUG, "Message when function ends successfully: None")
    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)


if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
