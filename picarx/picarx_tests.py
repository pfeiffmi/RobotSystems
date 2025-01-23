import os
import sys

path = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(path, "..")
sys.path.append(path)

from picarx.picarx_improved import Picarx

def forward_with_different_steering_angles():
    pass

def backward_with_different_steering_angles():
    pass

def parallel_park_left():
    pass

def parallel_park_right():
    pass

def k_point_turn(k):
    pass

def user_control():
    pass

if(__name__ == "__main__"):
    forward_with_different_steering_angles()
    