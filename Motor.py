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
        #: pin_motor_left_e
        self.m3e = 7
        #: pin_motor_right_a
        self.m1a = 3
        #: pin_motor_right_b
        self.m1b = 4
        #: pin_motor_left_a
        self.m2a = 5
        #: pin_motor_left_b
        self.m2b = 6
        #: MCP23017 (expander control)
        self.mcp = mcp
        self.speed_right = 0
        self.speed_left = 0
        self.previous_time = 0
        self.pwm_previous_right = 0
        self.pwm_previous_left = 0

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

    # Accessing methods
    def get_speed_right(self):
        return self.speed_right

    def get_speed_left(self):
        return self.speed_left

    # Setting functions
    # Direction
    def move_forward(self, speed_right=0, speed_left=0):
        print("moveForward "+"R:"+str(speed_right)+" L:"+str(speed_left))
        self.mcp.output(self.m1a, 0)
        self.mcp.output(self.m1b, 1)
        self.mcp.output(self.m2a, 0)
        self.mcp.output(self.m2b, 1)
        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1.ChangeDutyCycle(self.speed_right)
        self.p2.ChangeDutyCycle(self.speed_left)

    def move_backward(self, speed_right=0, speed_left=0):
        print("moveBackward"+"R:"+str(speed_right)+" L:"+str(speed_left))
        self.mcp.output(self.m1a, 1)
        self.mcp.output(self.m1b, 0)
        self.mcp.output(self.m2a, 1)
        self.mcp.output(self.m2b, 0)
        self.speed_right = speed_right
        self.speed_left = speed_left
        self.p1.ChangeDutyCycle(self.speed_right)
        self.p2.ChangeDutyCycle(self.speed_left)

    def turn_left(self, speed_right=0, speed_left=0):
        print("turn left"+"R:"+str(speed_right)+" L:"+str(speed_left))
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
        print("turn right"+"R:"+str(speed_right)+" L:"+str(speed_left))
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
        print("start motors")
        self.p1.start(0)
        self.p2.start(0)
        print("motors stared")

    def stop_motors(self):
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

    # Motion control
    """
    ..function:: enslavement_right(self, x, yr)

    Regulate the right wheel:

    Formula:
    command = Kp * error + Ki * sum_errors + Kd * (error - previous_error)

    Algorithm:
    Each x milliseconds, do :
        error = consign - measure;
        sum_errors += error;
        error_variation = error - previous_error;
        command = Kp * erreur + Ki * sum_errors + Kd * error_variation;
        previous_error = error

       :type int: integer tick number
       :param x: distance to reach = consign
       :type int: integer tick number
       :param yr: distance reached = measured
       :rtype: None
       :return: None
   """
    def enslavement_right(self, x, yr):
        Kp = 1
        # calcul de l'erreur
        error = x - yr
        p = Kp *error
        c = p + self.pwm_previous_right
        self.pwm_previous_right = c
        if c > 100:
            c = 100.0
        if c < 00:
            c = 0.0
        self.change_speed_right(c)

    def enslavement_left(self, x, yl):
        Kp = 1
        error = x - yl
        p = Kp * error
        c = p + self.pwm_previous_left
        self.pwm_previous_left = c
        if c > 100:
            c = 100.0
        if c < 00:
            c = 0.0
        self.change_speed_left(c)

    def regulation(self, e, speed_target):
        T = 1  # Sampling period = 1s
        homogenization_coefficient = 3.3
        actual = time.time()
        Te = actual - self.previous_time
        if Te > T:  # each seconds
            self.previous_time = actual
            speed_right = e.get_speed_right_instantaneous()*homogenization_coefficient
            speed_left = e.get_speed_left_instantaneous()*homogenization_coefficient
            self.enslavement_right(x=speed_target, yr=speed_right)
            self.enslavement_left(x=speed_target, yl=speed_left)

    # Moving
    #def move(self,dx,dy):






