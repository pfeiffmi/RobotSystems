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

from classes.line_sensor import LineSensor
from classes.distance_sensor import DistanceSensor
from classes.interpreter import Interpreter
from classes.controller import Controller
from classes.bus import Bus

from classes import rossros as rr

import logging
import math

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
            sensor = LineSensor(method=method)
            interpreter = Interpreter(line_threshold=115, sensitivity=1.0, is_dark_line=True, method=method)
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
            sensor = LineSensor(method=method)
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
                # Smooth: k_p=0.015, k_i=0.001, k_d=0.001
                turn_proportion = interpreter.interpret_sensor_reading_PID(data, k_p=0.015, k_i=0.001, k_d=0.001)
                
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
        sensor = LineSensor(method=method)
        interpreter = Interpreter(line_threshold=115, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=0)
        producer_bus = Bus(init_message=np.array([0, 0, 0]))
        consumer_bus = Bus(init_message=0)
        
        futures = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            # create tasks
            eSensor = executor.submit(sensor.producer, producer_bus, delay_sec=0.03)
            eInterpreter = executor.submit(interpreter.producer_consumer, producer_bus, consumer_bus, delay_sec=0.05)
            eController = executor.submit(controller.consumer, consumer_bus, delay_sec=0.07)
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
                    time.sleep(1)
    
            except KeyboardInterrupt:
                # Trigger the shutdown event when receive the kill signal
                print("Shutting down")
                shutdown_event.set()
    
            finally:
                # Ensures all threads finish
                executor.shutdown()
    
    elif(method == "vision"):
        sensor = LineSensor(method=method)
        interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)
        producer_bus = Bus(init_message=np.zeros((100,100)))
        consumer_bus = Bus(init_message=0)

        futures = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            eSensor = executor.submit(sensor.producer, producer_bus, delay_sec=0.07)
            eInterpreter = executor.submit(interpreter.producer_consumer, producer_bus, consumer_bus, delay_sec=0.11)
            eController = executor.submit(controller.consumer, consumer_bus, delay_sec=0.13)
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
                    time.sleep(1)
    
            except KeyboardInterrupt:
                # Trigger the shutdown event when receive the kill signal
                print("Shutting down")
                shutdown_event.set()
    
            finally:
                # Ensures all threads finish
                executor.shutdown()


# Code modified from Ross Hattan on GitHub: https://github.com/rlhatton/RossROS/blob/main/rr_demo.py
def ross_ros_concurrent_line_follow_and_collision_avoidance(picar, method):
    # ross ros for concurrent line following using the grayscale sensor
    if(method == "grayscale"):
        # Instantiate sensor, interpreter, and controller object for sense-think-act architecture
        line_sensor = LineSensor(method=method)
        distance_sensor = DistanceSensor()
        interpreter = Interpreter(line_threshold=115, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(picar=picar, max_turn_angle=30, init_turn_angle=0, init_tilt_angle=0)

        # define delays (in seconds)
        sensor_read_delay = 0.03
        interpreter_read_delay = 0.05
        control_read_delay = 0.07

        # define PID gain constants
        k_p = 0.3
        k_i = 0.001
        k_d = 0.02

    # ross ros for concurrent line following using the camera
    elif(method == "vision"):
        # Instantiate sensor, interpreter, and controller object for sense-think-act architecture
        line_sensor = LineSensor(method=method)
        distance_sensor = DistanceSensor()
        interpreter = Interpreter(line_threshold=35, sensitivity=1.0, is_dark_line=True, method=method)
        controller = Controller(picar=picar, max_turn_angle=30, init_turn_angle=0, init_tilt_angle=50)

        # define delays (in seconds)
        sensor_read_delay = 0.07
        interpreter_read_delay = 0.011
        control_read_delay = 0.013

        # define PID gain constants
        k_p = 0.015
        k_i = 0.001
        k_d = 0.001

    else:
        raise(Exception(f"Invalid method (\"{method}\") passed to ross_ros_concurrent_line_follow"))
    
    # Set up logger
    logging.getLogger().setLevel(logging.INFO)

    # Set buses related to the sensors
    bus_line_sensor = rr.Bus(
        initial_message = line_sensor.read_data(), 
        name = "Line Sensor Bus"
    )
    bus_distance_sensor = rr.Bus(
        initial_message = distance_sensor.read_data(), 
        name = "Distance Sensor Bus"
    )

    # Set busses related to the interpreter
    bus_k_p = rr.Bus(
        initial_message = k_p,
        name = "k_p Bus"
    )
    bus_k_i = rr.Bus(
        initial_message = k_i,
        name = "k_i Bus"
    )
    bus_k_d = rr.Bus(
        initial_message = k_d,
        name = "k_d Bus"
    )
    bus_line_interpreter = rr.Bus(
        initial_message = interpreter.interpret_sensor_reading_PID(
            bus_line_sensor.get_message(),
            k_p = bus_k_p.get_message(), 
            k_i = bus_k_i.get_message(), 
            k_d = bus_k_d.get_message()
        ), 
        name = "Line Interpreter Bus"
    )
    bus_distance_interpreter = rr.Bus(
        initial_message = interpreter.interpret_distance(
            bus_distance_sensor.get_message()
        ), 
        name = "Distance Interpreter Bus"
    )
    
    # Set bus for termination
    bus_terminate = rr.Bus(
        initial_message = 0, 
        name = "Termination Bus"
    )

    # Wrap the signals for the sensor, interpreter, and controller object
    read_line_sensor = rr.Producer(
        producer_function = line_sensor.read_data,
        output_buses = bus_line_sensor,
        delay = sensor_read_delay,
        termination_buses = bus_terminate,
        name = "Read line sensor signal"
    )
    read_distance_sensor = rr.Producer(
        producer_function = distance_sensor.read_data,
        output_buses = bus_distance_sensor,
        delay = sensor_read_delay,
        termination_buses = bus_terminate,
        name = "Read distance sensor signal"
    )
    interpret_line_sensor = rr.ConsumerProducer(
        consumer_producer_function = interpreter.interpret_sensor_reading_PID,  # function that will generate data
        input_buses = (bus_line_sensor, bus_k_p, bus_k_i, bus_k_d),
        output_buses = bus_line_interpreter,  # output data bus
        delay = interpreter_read_delay,  # delay between data generation cycles
        termination_buses = bus_terminate,  # bus to watch for termination signal
        name = "Read interpreter signal"
    )
    interpret_distance_sensor = rr.ConsumerProducer(
        consumer_producer_function = interpreter.interpret_distance,  # function that will generate data
        input_buses = bus_distance_sensor,
        output_buses = bus_distance_interpreter,  # output data bus
        delay = interpreter_read_delay,  # delay between data generation cycles
        termination_buses = bus_terminate,  # bus to watch for termination signal
        name = "Read interpreter signal"
    )
    control_motors = rr.Consumer(
        consumer_function = controller.set_picar_parameters,  # function that will process data
        input_buses = (bus_line_interpreter, bus_distance_interpreter),  # input data buses
        delay = control_read_delay,  # delay between data control cycles
        termination_buses = bus_terminate,  # bus to watch for termination signal
        name = "Control motor slot"
    )

    # Create Printer and Termination Timer objects
    print_buses = rr.Printer(
        printer_bus = (bus_line_sensor, bus_distance_sensor, bus_line_interpreter, bus_distance_interpreter, bus_terminate),  # input data buses
        delay = 0.5,  # delay between printing cycles
        termination_buses = bus_terminate,  # bus to watch for termination signal
        name = "Print raw and derived data",  # Name of printer
        print_prefix = "Data bus readings are: "
    )
    termination_timer = rr.Timer(
        output_buses = bus_terminate,  # Output data bus
        duration = 5,  # Duration
        delay = 0.1,  # Delay between checking for termination time
        termination_buses = bus_terminate,  # Bus to check for termination signal
        name = "Termination timer")  # Name of this timer

    # Create a list of producer-consumers to execute concurrently
    producer_consumer_list = [
        read_line_sensor,
        read_distance_sensor,
        interpret_line_sensor,
        interpret_distance_sensor,
        control_motors,
        print_buses,
        termination_timer
    ]

    # Execute the list of producer-consumers concurrently
    rr.runConcurrently(producer_consumer_list)


def user_control(picar):
    while(True):
        print("==========")
        print(" 0. Quit")
        print(" 1. forward_with_different_steering_angles")
        print(" 2. backward_with_different_steering_angles")
        print(" 3. parallel_park_left")
        print(" 4. parallel_park_right")
        print(" 5. k_point_turn(k=3)")
        print(" 6. Line Follow (Grayscale)")
        print(" 7. Line Follow (Vision)")
        print(" 8. Concurrent Line Follow (Grayscale)")
        print(" 9. Concurrent Line Follow (Vision)")
        print("10. RossROS Concurrent Line Follow & Collision Avoidance (Grayscale)")
        print("11. RossROS Concurrent Line Follow & Collision Avoidance (Vision)")
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
            case 10:
                ross_ros_concurrent_line_follow_and_collision_avoidance(picar, method="grayscale")
            case 11:
                ross_ros_concurrent_line_follow_and_collision_avoidance(picar, method="vision")
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
    