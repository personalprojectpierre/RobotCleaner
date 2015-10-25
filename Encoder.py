#!/usr/bin/python
# -*- encoding: utf8 -*-
import time
import RPi.GPIO as GPIO

"""
.. module:: Encoder class
   :platform: Unix (Raspberry Pi Debian)
   :synopsis: Supervise the encoder

.. moduleauthor:: Pierre
"""

class Encoder(object):
    def __init__(self):
        #: Step number of the wheel (GPIO.BOTH)
        self.step = 60.0000000
        #: perimeter / step = 31.4 /60 = 0.5233333 (/30 =  1.0471975)
        self.step_distance = 1.0000000
        #: Chanel definition
        self.pin_encoder_right = 21  # chanel_encoder_right = 22
        self.pin_encoder_left = 19    # chanel_encoder_left = 4
        #: Distance covered
        self.distance_right = 0.0000000
        self.distance_left = 0.0000000
        #: Average speed
        self.speed_right = 0.0000000
        self.speed_left = 0.0000000
        #: Time
        self.start_time = time.time()
        #: Distance
        self.distance_right_previous = 0.0000000
        self.distance_left_previous = 0.0000000

    def initEncoder(self):
        # With chanel handling
        GPIO.setmode(GPIO.BOARD) #BCM
        GPIO.setwarnings(False)
        # Pull up (to get 0 instead of high impedance state)
        GPIO.setup(self.pin_encoder_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_encoder_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Interruption configuration: speed max => 100 => bouncetime max => 60
        GPIO.add_event_detect(self.pin_encoder_right, GPIO.BOTH,
                               callback=self.increase_distance_right)
        GPIO.add_event_detect(self.pin_encoder_left, GPIO.BOTH,
                              callback=self.increase_distance_left)

    # Accessing methods
    def get_distance_right(self):
        return self.distance_right

    def get_distance_left(self):
        return self.distance_left

    def get_cm_right(self):
        return self.get_distance_right()*0.4570000

    def get_cm_left(self):
        return self.get_distance_left()*0.4570000

    def get_speed_right_average(self):
        delta = time.time() - self.start_time
        return self.get_distance_right() / delta  # cm/s

    def get_speed_left_average(self):
        delta = time.time() - self.start_time
        return self.get_distance_left() / delta

    def get_speed_right_instantaneous(self):
        distance = self.get_distance_right()
        speed_right = distance - self.distance_right_previous
        self.distance_right_previous = distance
        return speed_right

    def get_speed_left_instantaneous(self):
        distance = self.get_distance_left()
        speed_left = distance - self.distance_left_previous
        self.distance_left_previous = distance
        return speed_left

    # Setting functions
    def increase_distance_right(self, channel):
        self.distance_right = self.get_distance_right() + self.step_distance

    def increase_distance_left(self, channel):
        self.distance_left = self.get_distance_left() + self.step_distance

    def set_zero_distance(self):
        self.distance_right = 0
        self.distance_left = 0

    # Predicates
    """
    ..function:: is_distance_reached(self, distance)
     Check the distance covered, and make a reset of the distance
    """
    def is_distance_reached(self, distance):
        dist_right = self.get_cm_right()
        dist_left = self.get_cm_left()
        if (dist_left >= distance) or (dist_right >= distance):
            print("Delta: "+str(dist_right-dist_left))
            self.set_zero_distance()
            return True
        return False
        

