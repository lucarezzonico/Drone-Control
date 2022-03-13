import math
from typing import Sequence
import numpy as np
from numpy import linalg as la
import random
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
import matplotlib.pyplot as plt
import statistics

LIST_END = -1

class Potential_Field:
    # Every constant related to potential field
    def __init__(self):
        self.ko = 0.05
        self.krot = 0.02
        self.kd1 = 1.5
        self.kd2 = 1.65
        self.m = 0.033
        self.mu = 1


class Force(Potential_Field):
    def __init__(self):
        super().__init__() 
        self.x = 0.0
        self.y = 0.0
    
    def get(self):
        return (self.x, self.y)
    
    def add(self, applied_force):
        self.x += applied_force[0]
        self.y += applied_force[1] 
    
    def is_zero(self):
        if (self.x * self.y) == 0.0:
            return True
        else:
            return False
    
    def to_velocity(self, dt):
        return [self.x/self.m * dt, self.y/self.m * dt]         


def is_close(range, MIN_DISTANCE = 0.28):
    # RETURN True if the value is below MIN_DISTANCE
    # RETURN False else
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def is_close_up(range, MIN_DISTANCE = 0.1):
    # RETURN True if the value is below MIN_DISTANCE
    # RETURN False else
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def bound(v, b=0.5):
    # RETURN bounded velocity
    if (v[0] > b):
        v[0] = b
    if (v[0] < -b):
        v[0] = -b
    if (v[1] > b):
        v[1] = b
    if (v[1] < -b):
        v[1] = -b
    return v 

def destination_reached(goal, stream, thresh=0.07):
    p = goal.get()
    q = stream.get_pos()
    # RETURN whether a position if the points are near enough 
    if pow(p[0]-q[0],2)+pow(p[1]-q[1],2) < pow(thresh,2):
        return True
    return False

def iterate(it, route):
    # RETURN None if the value cannot be iterated
    # RETURN next route value and index otherwise 
    it = it+1
    if it >= len(route): # If route(it+1) is not possible
        print("LIST END")
        return LIST_END, route[it-1]
    else:
        print("New goal is: ", route[it])
        return it, route[it]

def pos_sub(p1, p2):
    return(p1[0]-p2[0],p1[1]-p2[1])

def pos_sum(p1, p2):
    return(p1[0]+p2[0],p1[1]+p2[1])

def find_rect_mid(sides_list,axe):
    print("find_rect_mid()")
    axe_x, axe_y = sides_list[0], sides_list[1]
    if axe:
        box_pos = ((axe_x[0][0]+axe_x[1][0])/2,(axe_y[0][1]+axe_y[1][1])/2)
    else:
        box_pos = ((axe_y[0][0]+axe_y[1][0])/2,(axe_x[0][1]+axe_x[1][1])/2)
    return box_pos

def find_segment_mid(segment):
    print("find_segment_mid")
    return (((segment[0][0]+segment[1][0])/2),(segment[0][1]+segment[1][1])/2)

def find_triangle_rectangle(point_x,point_y,a):
    print("find_triangle_rectangle")
    if a:
        return (point_x[0], point_y[1])
    else:
        return (point_y[0], point_x[1])