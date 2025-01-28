import os
import sys
import time

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

from picarx_improved import Picarx

def test(picar):
    # =================
    # ===== Tests =====
    # =================
    
    # Stright-Line Speed:
    ## ~19.12 mm/s forward speed at 50% speed [with right drift] (measured empirically)
    ## ~19.42 mm/s backward speed at 50% speed [with right drift] (measured empirically)

    # Forward Circle:
    ## ~8.16 sec per 50% speed forward left circle at -30 degrees (measured empirically)
    ## ~8.72 sec per 50% speed forward right circle at 30 degrees (measured empirically)
    
    # Backward Circle:
    ## ~8.12 sec per 50% speed backward left circle at -30 degrees (measured empirically)
    ## ~8.86 sec per 50% speed backward right circle at 30 degrees (measured empirically)
    
    picar.set_dir_servo_angle(0)
    #picar.forward(50)
    picar.backward(50)
    time.sleep(10)
    picar.set_dir_servo_angle(0)

def forward_with_different_steering_angles(picar):
    for i in range(-30, 31, 1):
        print(i)
        picar.set_dir_servo_angle(i)
        picar.forward(50)
        time.sleep(0.05)
    for i in range(30, -1, -1):
        print(i)
        picar.set_dir_servo_angle(i)
        picar.forward(50)
        time.sleep(0.05)

def backward_with_different_steering_angles(picar):
    for i in range(-30, 31, 1):
        print(i)
        picar.set_dir_servo_angle(i)
        picar.backward(50)
        time.sleep(0.05)
    for i in range(30, -1, -1):
        print(i)
        picar.set_dir_servo_angle(i)
        picar.backward(50)
        time.sleep(0.05)

def parallel_park_left(picar):
    pass

def parallel_park_right(picar):
    pass

def k_point_turn(picar, k):
    pass

def user_control(picar):
    pass

if(__name__ == "__main__"):
    px = Picarx()
    
    x = 0
    match(x):
        case 0:
            forward_with_different_steering_angles(px)
        case 1:
            backward_with_different_steering_angles(px)
        case 2:
            parallel_park_left(px)
        case 3:
            parallel_park_right(px)
        case 4:
            k_point_turn(px, k=3)
        case 5:
            user_control(px)
        case _:
            pass
    
    time.sleep(1)
    px.stop()
    