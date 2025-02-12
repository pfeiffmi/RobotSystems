import os
import sys
import time
import concurrent.futures
import threading
import numpy as np

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
    # Define shutdown event
    shutdown_event = threading.Event()

    # Exception handle function: Sensor
    def handle_exception_sensor(future):
        exception = future.exception()
        if exception:
            print(f"Exception in Sensor worker thread: {exception}")

    # Exception handle function: Interpreter
    def handle_exception_interpreter(future):
        exception = future.exception()
        if exception:
            print(f"Exception in Interpreter worker thread: {exception}")

    # Exception handle function: Controller
    def handle_exception_controller(future):
        exception = future.exception()
        if exception:
            print(f"Exception in Controller worker thread: {exception}")

    if(method == "grayscale"):
        sensor = Sensor(method=method)
        interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)
        producer_bus = Bus(init_message=[0, 0, 0])
        consumer_bus = Bus(init_message=0)
        
        futures = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            # create tasks
            eSensor = executor.submit(sensor.producer, producer_bus, delay_sec=0.01)
            eInterpreter = executor.submit(interpreter.producer_consumer, producer_bus, consumer_bus, delay_sec=0.03)
            eController = executor.submit(controller.consumer, consumer_bus, delay_sec=0.05)
            # add exception callback
            eSensor.add_done_callback(handle_exception_sensor)
            eInterpreter.add_done_callback(handle_exception_interpreter)
            eController.add_done_callback(handle_exception_controller)
            # append the tasks to the task list
            futures.append(eSensor)
            futures.append(eInterpreter)
            futures.append(eController)
    
    elif(method == "vision"):
        sensor = Sensor(method=method)
        interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)
        producer_bus = Bus(init_message=np.zeros((100,100)))
        consumer_bus = Bus(init_message=0)

        futures = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            eSensor = executor.submit(sensor.producer, producer_bus, delay_sec=0.075)
            eInterpreter = executor.submit(interpreter.producer_consumer, producer_bus, consumer_bus, delay_sec=0.175)
            eController = executor.submit(controller.consumer, consumer_bus, delay_sec=0.2)
            # add exception callback
            eSensor.add_done_callback(handle_exception_sensor)
            eInterpreter.add_done_callback(handle_exception_interpreter)
            eController.add_done_callback(handle_exception_controller)
            # append the tasks to the task list
            futures.append(eSensor)
            futures.append(eInterpreter)
            futures.append(eController)
    
    try:
        # Set the speed of the car
        picar.forward(30)
        # Keep the main thread running to response for the kill signal
        while(not shutdown_event.is_set()):
            print(12)
            time.sleep(1)
    
    except KeyboardInterrupt:
        # Trigger the shutdown event when receive the kill signal
        print("Shutting down")
        shutdown_event.set()
    
    finally:
        # Ensures all threads finish
        executor.shutdown()

def user_control(picar):
    while(True):
        print("==========")
        print("0. Quit")
        print("1. forward_with_different_steering_angles")
        print("2. backward_with_different_steering_angles")
        print("3. parallel_park_left")
        print("4. parallel_park_right")
        print("5. k_point_turn(k=3)")
        print("6. Line Follow (Grayscale)")
        print("7. Line Follow (Vision)")
        print("8. Concurrent Line Follow (Grayscale)")
        print("9. Concurrent Line Follow (Vision)")
        print("==========")
        selection = input("Selection: ")
        print("==========")

        selection = int(selection)

        match(selection):
            case 0:
                break
            case 1:
                forward_with_different_steering_angles(picar)
            case 2:
                backward_with_different_steering_angles(picar)
            case 3:
                parallel_park_left(picar)
            case 4:
                parallel_park_right(picar)
            case 5:
                k_point_turn(picar, k=3)
            case 6:
                line_follow(picar, method="grayscale")
            case 7:
                line_follow(picar, method="vision")
            case 8:
                concurrent_line_follow(picar, method="grayscale")
            case 9:
                concurrent_line_follow(picar, method="vision")
            case _:
                continue
        
        picar.set_dir_servo_angle(0)
        picar.stop()


if(__name__ == "__main__"):
    #px = Picarx(right_drift_offset = 0.275)
    px = Picarx(right_drift_offset = 0.0)
    
    user_control(px)
    
    time.sleep(1)
    px.stop()
    