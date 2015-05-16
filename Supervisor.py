#!/usr/bin/python
# -*- encoding: utf8 -*-

# File: Supervisor class
# Author: Pierre

import sys
import time

import RPi.GPIO as GPIO

class Supervisor(object):
    def __init__(self, m, e, s):
        self.cycle = 0
        self.rangeDetection = 17  # minimum distance range
        self.m = m
        self.e = e
        self.s = s
        self.t = time.time()  # time in s
        self.previous_direction = 'R'
        self.Vxy = [('F', 400)]

    def choose_direction(self):
        self.m.move(self.Vxy)

    def motion_control(self):
        self.m.regulation(self.e, speed_target=70)

    def distance_reach(self):
        if not self.Vxy:
            self.m.fast_motor_stop()
        else:
            distance = self.Vxy[0][1]
            if self.e.is_distance_reached(distance):
                self.Vxy.pop(0)

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

    def end(self, dist_target):
        if self.e.is_distance_reached(dist_target):
            self.m.fast_motor_stop()
            self.m.vacuum_cleaner_stop()
            time.sleep(2)
            self.m.stop_motors()
            self.s.unactivate_us()
            GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            sys.exit(0)