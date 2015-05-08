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
        self.t = 0  # time in s


    def detection_obstacle(self):
            dist_front = self.s.getUSFront()
            if dist_front < self.rangeDetection:
                self.m.vacuum_cleaner_stop()
                self.m.move_backward(speed_left=100, speed_right=100)
                time.sleep(2)
                self.m.turn_right(speed_left=100, speed_right=100)
                time.sleep(2.5)
                #reach_distance(e, distance=20)
                #while self.s.getUSFront() < self.rangeDetection+5:
                #while self.s.getUSFront() < self.rangeDetection+5:
                #self.m.move_backward(speed_left=100, speed_right=100)
                # while self.s.getUSFront() < self.rangeDetection+20:
                # self.m.turn_right(speed_left=100, speed_right=100)
                self.m.move_forward(speed_left=33, speed_right=50)
                self.m.vacuum_cleaner_start()

    def end(self, dist_target, dist_right, dist_left):
        if self.e.distance_reached(dist_target, dist_right, dist_left):
            self.m.fast_motor_stop()
            self.m.vacuum_cleaner_stop()
            delta = dist_right-dist_left
            print("Delta: "+str(delta))
            time.sleep(2)  # Watch out to the time.sleep() function
            self.m.stop_motors()
            GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            sys.exit(0)

    def demo(self):
        self.detection_obstacle()
        dist_right = self.e.get_distance_right()
        dist_left = self.e.get_distance_left()
        self.m.enslavement_position(target=dist_right, process_variable=dist_left)