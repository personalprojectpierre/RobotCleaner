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
        self.t = time.time() # time in s
        self.previous_direction = 'R'

    def motion_control(self,m,e,s):
        m.regulation(e, speed_target=70)

    def distance_reach(self, m, e, s, Vxy):
        if not Vxy:
            self.m.fast_motor_stop()
        else:
            distance = Vxy[0][1]
            if e.is_distance_reached(distance):
                Vxy.pop(0)

    def detection_obstacle(self, m, e, s, Vxy):
            Ts = 0.05  # Sampling period = 0.05s
            actual = time.time()
            T = actual - self.t
            if T > Ts:  # each seconds
                self.t = actual
                dist_front = self.s.getUSFront()
                if dist_front < self.rangeDetection:
                    self.s.unactivate_us_front()
                    self.m.vacuum_cleaner_stop()
                    distance_covered = (e.get_cm_left() + e.get_cm_left())/2
                    if Vxy[0][0] == 'F':
                        d = Vxy[0][1] - distance_covered
                        e.set_zero_distance()
                        Vxy.pop(0)
                        if self.previous_direction == 'R':
                            self.previous_direction = 'L'
                            Vxy.append(['L', 19.5])
                            Vxy.append(['F', 30])
                            Vxy.append(['L', 19.5])
                        else:
                            self.previous_direction = 'R'
                            Vxy.append(['R', 19.5])
                            Vxy.append(['F', 30])
                            Vxy.append(['R', 19.5])

                        Vxy.append(['F', d])
                    self.s.activate_us_front()
                    print(str(Vxy))
            #self.m.vacuum_cleaner_start()

    def end(self, dist_target):
        if self.e.is_distance_reached(dist_target):
            self.m.fast_motor_stop()
            self.m.vacuum_cleaner_stop()
            time.sleep(2)  # Watch out to the time.sleep() function
            self.m.stop_motors()
            self.s.unactivate_us_front()
            GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            sys.exit(0)

    def demo(self):
        self.detection_obstacle()
        dist_right = self.e.get_distance_right()
        dist_left = self.e.get_distance_left()
        #self.m.enslavement_position(target=dist_right, process_variable=dist_left)