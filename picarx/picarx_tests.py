import os
import sys
import time

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

from picarx_improved import Picarx

from classes.sensor import Sensor
from classes.interpreter import Interpreter
from classes.controller import Controller

def test(picar):
    # =================
    # ===== Tests =====
    # =================
    
    # Stright-Line Speed:
    ## ~19.1 cm/s forward speed at 50% speed [with right drift] (measured empirically)
    ## ~19.4 cm/s backward speed at 50% speed [with right drift] (measured empirically)

    # Forward Circle:
    ## ~8.2 sec per 50% speed forward left circle at -30 degrees (measured empirically)
    ## ~8.7 sec per 50% speed forward right circle at 30 degrees (measured empirically)
    
    # Backward Circle:
    ## ~8.1 sec per 50% speed backward left circle at -30 degrees (measured empirically)
    ## ~8.9 sec per 50% speed backward right circle at 30 degrees (measured empirically)

    # ========================================
    # ===== Post Drift-Calibration Tests (drift_offset = 27.5%) =====
    # ========================================
    
    # Stright-Line Speed:
    ## ~16.34 cm/s forward speed at 50% speed [with calibration to minimize right drift] (measured empirically)
    ## ~x.x cm/s backward speed at 50% speed [with calibration to minimize right drift] (measured empirically)

    # Forward Circle:
    ## ~x.x sec per 50% speed forward left circle at -30 degrees (measured empirically)
    ## ~x.x sec per 50% speed forward right circle at 30 degrees (measured empirically)
    
    # Backward Circle:
    ## ~x.x sec per 50% speed backward left circle at -30 degrees (measured empirically)
    ## ~x.x sec per 50% speed backward right circle at 30 degrees (measured empirically)
    
    picar.set_dir_servo_angle(0)
    #picar.forward(50)
    picar.backward(50)
    time.sleep(19.5)
    picar.set_dir_servo_angle(0)

def forward_with_different_steering_angles(picar):
    for i in range(-30, 31, 1):
        picar.set_dir_servo_angle(i)
        picar.forward(50)
        time.sleep(0.05)
    for i in range(30, -31, -1):
        picar.set_dir_servo_angle(i)
        picar.forward(50)
        time.sleep(0.05)
    for i in range(-30, 1, 1):
        picar.set_dir_servo_angle(i)
        picar.forward(50)
        time.sleep(0.05)
    
    picar.set_dir_servo_angle(0)
    picar.stop()

def backward_with_different_steering_angles(picar):
    for i in range(-30, 31, 1):
        picar.set_dir_servo_angle(i)
        picar.backward(50)
        time.sleep(0.05)
    for i in range(30, -31, -1):
        picar.set_dir_servo_angle(i)
        picar.backward(50)
        time.sleep(0.05)
    for i in range(-30, 1, 1):
        picar.set_dir_servo_angle(i)
        picar.backward(50)
        time.sleep(0.05)
    
    picar.set_dir_servo_angle(0)
    picar.stop()


def left_turn_forward(picar, angle):
    picar.set_dir_servo_angle(-30)
    picar.forward(50)
    
    full_left_forward_turn_time = 8.2
    wait_time = full_left_forward_turn_time*(angle/360.0)
    time.sleep(wait_time)
    
    picar.set_dir_servo_angle(0)
    picar.stop()


def right_turn_forward(picar, angle):
    picar.set_dir_servo_angle(30)
    picar.forward(50)
    
    full_left_forward_turn_time = 8.7
    wait_time = full_left_forward_turn_time*(angle/360.0)
    time.sleep(wait_time)
    
    picar.set_dir_servo_angle(0)
    picar.stop()


def left_turn_backward(picar, angle):
    picar.set_dir_servo_angle(-30)
    picar.backward(50)
    
    full_left_forward_turn_time = 8.1
    wait_time = full_left_forward_turn_time*(angle/360.0)
    time.sleep(wait_time)
    
    picar.set_dir_servo_angle(0)
    picar.stop()


def right_turn_backward(picar, angle):
    picar.set_dir_servo_angle(30)
    picar.backward(50)
    
    full_left_forward_turn_time = 8.9
    wait_time = full_left_forward_turn_time*(angle/360.0)
    time.sleep(wait_time)
    
    picar.set_dir_servo_angle(0)
    picar.stop()


def parallel_park_left(picar):
    left_turn_forward(picar, angle=50)

    picar.set_dir_servo_angle(0)
    picar.forward(50)
    time.sleep(1)
    
    right_turn_forward(picar, angle=50)
    
    picar.backward(50)
    time.sleep(2)
    
    picar.set_dir_servo_angle(0)
    picar.stop()

def parallel_park_right(picar):
    right_turn_forward(picar, angle=50)

    picar.set_dir_servo_angle(0)
    picar.forward(50)
    time.sleep(1)
    
    left_turn_forward(picar, angle=50)
    
    picar.backward(50)
    time.sleep(2)
    
    picar.set_dir_servo_angle(0)
    picar.stop()

def k_point_turn(picar, k):
    for i in range(k):
        if(i % 2 == 0):
            left_turn_forward(picar, 180.0/k)
        else:
            right_turn_backward(picar, 180.0/k)
    
    picar.forward(50)
    time.sleep(1)

    picar.set_dir_servo_angle(0)
    picar.stop()

def line_follow(picar):
    sensor = Sensor()
    interpreter = Interpreter(sensitivity=1.0, is_dark_line=True)
    controller = Controller(max_turn_angle=30)

    try:
        while(True):
            picar.forward(50)
            data = sensor.read_data()
            turn_proportion = interpreter.interpret_sensor_reading_discrete(data, threshold=20)
            #interpreter.interpret_sensor_reading_proportional(data, scaling_function="linear", threshold=25)
            #interpreter.interpret_sensor_reading_PID(data)
            controller.set_turn_proportion(turn_proportion)
            time.sleep(0.1)
    except:
        pass
    
    

def user_control(picar):
    while(True):
        selection = input("==========\n-1. Quit\n0. forward_with_different_steering_angles\n1. backward_with_different_steering_angles\n2. parallel_park_left\n3. parallel_park_right\n4. k_point_turn(k=3)\n5. Line Follow\nSelection: ")
        selection = int(selection)
        match(selection):
            case 0:
                forward_with_different_steering_angles(picar)
            case 1:
                backward_with_different_steering_angles(picar)
            case 2:
                parallel_park_left(picar)
            case 3:
                parallel_park_right(picar)
            case 4:
                k_point_turn(picar, k=3)
            case 5:
                line_follow(picar)
            case _:
                break
        
        picar.set_dir_servo_angle(0)
        picar.stop()


if(__name__ == "__main__"):
    #px = Picarx(right_drift_offset = 0.275)
    px = Picarx(right_drift_offset = 0.0)
    
    user_control(px)
    
    time.sleep(1)
    px.stop()
    