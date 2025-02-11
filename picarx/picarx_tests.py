import os
import sys
import time
import concurrent.futures

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

from picarx_improved import Picarx

from classes.sensor import Sensor
from classes.interpreter import Interpreter
from classes.controller import Controller
from classes.bus import Bus

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

def line_follow(picar, method):
    try:
        if(method == "grayscale"):
            sensor = Sensor(method=method)
            interpreter = Interpreter(line_threshold=95, sensitivity=1.0, is_dark_line=True, method=method)
            controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)

            while(True):
                picar.forward(30)
                data = sensor.read_data()
                
                #turn_proportion = interpreter.interpret_sensor_reading_discrete(data, threshold=20)
                
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="cubic", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="square", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="linear", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="sin", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="logistic", threshold=125)
                
                # Oscillation: k_p=0.7, k_i=0.0, k_d=0.0
                # Mitigated oscillation: k_p=0.35, k_i=0.001, k_d=0.0
                # Smooth: k_p=0.35, k_i=0.005, k_d=0.02
                turn_proportion = interpreter.interpret_sensor_reading_PID(data, k_p=0.3, k_i=0.001, k_d=0.02)
                
                controller.set_turn_proportion(turn_proportion)
                time.sleep(0.015)
        
        elif(method == "vision"):
            sensor = Sensor(method=method)
            interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
            controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)
            while(True):
                picar.forward(30)
                data = sensor.read_data()
                
                #turn_proportion = interpreter.interpret_sensor_reading_discrete(data)
                
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="cubic", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="square", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="linear", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="sin", threshold=125)
                #turn_proportion = interpreter.interpret_sensor_reading_proportional(data, scaling_function="logistic", threshold=125)
                
                # Oscillation: k_p=0.025, k_i=0.0, k_d=0.0
                # Mitigated oscillation: k_p=0.015, k_i=0.005, k_d=0.0
                # Smooth: k_p=0.02, k_i=0.005, k_d=0.01
                turn_proportion = interpreter.interpret_sensor_reading_PID(data, k_p=0.03, k_i=0.005, k_d=0.01)
                
                controller.set_turn_proportion(turn_proportion)
                time.sleep(0.1)
    except Exception as e:
        print(e)


def concurrent_line_follow(picar, method):
    if(method == "grayscale"):
        sensor = Sensor(method=method)
        interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)
        producer_bus = Bus(init_message=[0, 0, 0])
        consumer_bus = Bus(init_message=[0, 0])
        
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            eSensor = executor.submit(sensor.producer, producer_bus, 0.01)
            eInterpreter = executor.submit(interpreter.producer_consumer, producer_bus, consumer_bus, 0.03)
            eController = executor.submit(controller.consumer, consumer_bus, 0.05)
    
    elif(method == "vision"):
        sensor = Sensor(method=method)
        interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)
        producer_bus = Bus(init_message=[0, 0, 0])
        consumer_bus = Bus(init_message=[0, 0])

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            eSensor = executor.submit(sensor.producer, producer_bus, 0.075)
            eInterpreter = executor.submit(interpreter.consumer_producer, producer_bus, consumer_bus, 0.175)
            eController = executor.submit(controller.consumer, consumer_bus, 0.2)

def user_control(picar):
    while(True):
        selection = input("==========\n-1. Quit\n0. forward_with_different_steering_angles\n1. backward_with_different_steering_angles\n2. parallel_park_left\n3. parallel_park_right\n4. k_point_turn(k=3)\n5. Line Follow (Grayscale)\n6. Line Follow (Vision)\nSelection: ")
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
                line_follow(picar, method="grayscale")
            case 6:
                line_follow(picar, method="vision")
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
    