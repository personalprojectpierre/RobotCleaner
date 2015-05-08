#!/usr/bin/python
# -*- encoding: utf8 -*-
import time
import RPi.GPIO as GPIO

class Motor(object):
    def __init__(self, mcp):
        #: pin_motor_right_e
        self.pin_11 = 11  # BOARD
        #: pin_motor_left_e
        self.pin_13 = 13  # BOARD

        self.p1 = None
        self.p2 = None

        pin_7 = 7

        pin_3 = 3
        pin_4 = 4
        pin_5 = 5
        pin_6 = 6

        #: pin_motor_left_e
        self.m3e = pin_7

        #: pin_motor_right_a
        self.m1a = pin_3
        #: pin_motor_right_b
        self.m1b = pin_4

        #: pin_motor_left_a
        self.m2a = pin_5
        #: pin_motor_left_b
        self.m2b = pin_6


        #: MCP23017 (expander control)
        self.mcp = mcp

        self.speed_right = 100
        self.speed_left = 100

        self.previous = 0

    def init_motor(self, speed_right=0, speed_left=0):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin_13, GPIO.OUT)
        GPIO.setup(self.pin_11, GPIO.OUT)
        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1 = GPIO.PWM(self.pin_13, 500)  # (channel=4)  frequency=500Hz
        self.p2 = GPIO.PWM(self.pin_11, 500)  # (channel=4)  frequency=500Hz
        self.start_motors()

        self.mcp.config(self.m3e,  self.mcp.OUTPUT)
        self.vacuum_cleaner_stop()

        self.mcp.config(self.m1a,  self.mcp.OUTPUT)
        self.mcp.config(self.m1b,  self.mcp.OUTPUT)

        self.mcp.config(self.m2a,  self.mcp.OUTPUT)
        self.mcp.config(self.m2b,  self.mcp.OUTPUT)


    # Direction
    def move_forward(self, speed_right=0, speed_left=0):
        # percentage 0% -> 100% of maximal speed
        print("moveForward")
        self.mcp.output(self.m1a, 0)
        self.mcp.output(self.m1b, 1)

        self.mcp.output(self.m2a, 0)
        self.mcp.output(self.m2b, 1)

        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1.ChangeDutyCycle(self.speed_right)
        self.p2.ChangeDutyCycle(self.speed_left)

    def move_backward(self, speed_right=0, speed_left=0):
        # percentage 0% -> 100% of maximal speed
        print("moveBackward")
        self.mcp.output(self.m1a, 1)
        self.mcp.output(self.m1b, 0)

        self.mcp.output(self.m2a, 1)
        self.mcp.output(self.m2b, 0)

        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1.ChangeDutyCycle(self.speed_right)
        self.p2.ChangeDutyCycle(self.speed_left)

    def turn_left(self, speed_right=0, speed_left=0):
        # percentage 0% -> 100% of maximal speed
        print("turn left")
        self.mcp.output(self.m1a, 0)
        self.mcp.output(self.m1b, 1)

        self.mcp.output(self.m2a, 1)
        self.mcp.output(self.m2b, 0)

        if speed_left == 0:
            self.fast_motor_stop_left()
        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1.ChangeDutyCycle(self.speed_right)
        self.p2.ChangeDutyCycle(self.speed_left)

    def turn_right(self,  speed_left=0, speed_right=0):
        # percentage 0% -> 100% of maximal speed
        print("turn right")
        self.mcp.output(self.m1a, 1)
        self.mcp.output(self.m1b, 0)

        self.mcp.output(self.m2a, 0)
        self.mcp.output(self.m2b, 1)

        if speed_right == 0:
            self.fast_motor_stop_right()
        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1.ChangeDutyCycle(self.speed_right)
        self.p2.ChangeDutyCycle(self.speed_left)


    # Start/Stop
    def start_motors(self):
        # percentage 0% -> 100% of maximal speed
        print("start motors")
        self.p1.start(0)
        self.p2.start(0)
        print("motors stared")

    def stop_motors(self):
        # percentage 0% -> 100% of maximal speed
        print("stopMotors")
        self.p1.stop()
        self.p2.stop()
        print("motors stopped.")

    def fast_motor_stop(self):
        self.fast_motor_stop_right()
        self.fast_motor_stop_left()

    def fast_motor_stop_right(self):
        self.mcp.output(self.m1a, 0)
        self.mcp.output(self.m1b, 0)

    def fast_motor_stop_left(self):
        self.mcp.output(self.m2a, 0)
        self.mcp.output(self.m2b, 0)

    def vacuum_cleaner_start(self):
        print("vacuum_cleaner_start")
        self.mcp.output(self.m3e, 1)

    def vacuum_cleaner_stop(self, duration=1):
        print("turnRight")
        self.mcp.output(self.m3e, 0)


    # Speed
    def change_speed_right(self, speed):
        self.p1.ChangeDutyCycle(speed)

    def change_speed_left(self, speed):
        self.p2.ChangeDutyCycle(speed)

    def enslavement_position(self, target, process_variable):
        if target > process_variable+1:  # Error computing
            print("+")
            self.speed_left = self.speed_left + 1
            if self.speed_left > 60:
                self.speed_left = 60
            self.change_speed_left(self.speed_left)
        if target < process_variable-1:  # Error computing
            print("-")
            self.speed_left = self.speed_left - 1
            if self.speed_left < 35:
                self.speed_left = 35
            self.change_speed_left(self.speed_left)

    def enslavement_right(self, target,process_variable):
            actual = time.time()
            diff = actual - self.previous
            if diff > 0.30:  # Time step to avoid redundancy
                self.previous = actual
                if target > process_variable+0.05:  # Error computing
                    #print("+")
                    self.speed_right = self.speed_right + 1
                    if self.speed_right > 60:
                        self.speed_right = 60
                    self.change_speed_right(self.speed_right)
                if target < process_variable-0.05:
                    #print("-")
                    self.speed_right = self.speed_right - 1
                    if self.speed_right < 33:
                        self.speed_right = 33
                    self.change_speed_right(self.speed_right)

    def enslavement_left(self, target,process_variable):
        actual = time.time()
        diff = actual - self.previous
        if diff > 0.30:  # Time step to avoid redundancy
            self.previous = actual
            if target > process_variable+0.05:
                #print("+")
                self.speed_left = self.speed_left + 1
                if self.speed_left > 47:
                    self.speed_left = 47
                self.change_speed_left(self.speed_left)
            if target < process_variable-0.05:
                #print("-")
                self.speed_left = self.speed_left - 1
                if self.speed_left < 20:
                    self.speed_left = 20
                self.change_speed_left(self.speed_left)

    def enslavement_speed(self, target, process_variable):
        actual = time.time()
        diff = actual - self.previous
        if diff > 0.30:  # Time step to avoid redundancy
            self.previous = actual
            if target > process_variable+1:
                print("+")
                self.speed_left = self.speed_left + 1
                if self.speed_left > 82:
                    self.speed_left = 82
                self.change_speed_left(self.speed_left)
            if target < process_variable-1:
                print("-")
                self.speed_left = self.speed_left - 1
                if self.speed_left < 22:
                    self.speed_left = 22
                self.change_speed_left(self.speed_left)

    # Moving
    #def move(self,dx,dy):
