#!/usr/bin/python
# -*- encoding: utf8 -*-

"""
.. module:: Supervisor class
   :platform: Unix (Raspberry Pi Debian)
   :synopsis: Supervise the call of class Motor, Encoder and Sonar

.. moduleauthor:: Pierre
"""
import sys
import time

import RPi.GPIO as GPIO

class Supervisor(object):
    def __init__(self, m, e, s):
        #: Cycle number
        self.cycle = 0
        #: Range detection
        self.rangeDetection = 17  # minimum distance range
        #: Motor object
        self.m = m
        #: Encoder object
        self.e = e
        #: Sonar object
        self.s = s
        #: Time of the simulation (s)
        self.t = time.time()
        #: Previous direction
        self.previous_direction = 'R'
        #: Motion vector [(direction, distance)]
        self.Vxy = [('F', 800)]

    """
    ..function:: choose_direction(self)
    Change the direction in function of the Motion Vector Vxy
    """
    def choose_direction(self):
        self.m.move(self.Vxy)

    """
    ..function:: motion_control(self)
    Make the PID regulation
    """
    def motion_control(self):
        self.m.regulation(self.e, speed_target=70)

    """
    ..function:: distance_reach(self)
    Check the distances, and modify the motion vector Vxy
    In case which it is finished, motors are stopped and US are unactivated
    """
    def distance_reach(self):
        if not self.Vxy:
            self.m.vacuum_cleaner_stop()
            self.m.fast_motor_stop()
            self.m.stop_motors()
            self.s.unactivate_us()
            GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            sys.exit(0)
        else:
            distance = self.Vxy[0][1]
            if self.e.is_distance_reached(distance):
                self.Vxy.pop(0)
    """
    ..function:: detection_obstacle(self)
    If a obstacle is detected, the mobile makes a ZigZag moving
    """
    def detection_obstacle(self):
            Ts = 0.1  # Sampling period = 0.05s
            actual = time.time()
            T = actual - self.t
            if T > Ts:  # each seconds
                self.t = actual
                dist_right = self.s.getUSRight()
                dist_front = self.s.getUSFront()
                dist_left  = self.s.getUSLeft()
                if dist_front < self.rangeDetection \
                        or dist_right < self.rangeDetection \
                        or dist_left < self.rangeDetection:
                    self.m.vacuum_cleaner_stop()
                    distance_covered = (self.e.get_cm_right()
                                        + self.e.get_cm_left())/2
                    if self.Vxy[0][0] == 'F':
                        d = self.Vxy[0][1] - distance_covered
                        self.e.set_zero_distance()
                        self.Vxy.pop(0)
                        if self.previous_direction == 'R':
                            self.previous_direction = 'L'
                            self.Vxy.append(['L', 19.5])
                            self.Vxy.append(['F', 30])
                            self.Vxy.append(['L', 19.5])
                        else:
                            self.previous_direction = 'R'
                            self.Vxy.append(['R', 19.5])
                            self.Vxy.append(['F', 30])
                            self.Vxy.append(['R', 19.5])
                        self.Vxy.append(['F', d])
                    print(str(self.Vxy))
            #self.m.vacuum_cleaner_start()