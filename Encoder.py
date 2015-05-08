#!/usr/bin/python
# -*- encoding: utf8 -*-
import time
import RPi.GPIO as GPIO


class Encoder(object):
    def __init__(self):
        #: Step number of the wheel
        self.step = 30.0000000
        #: perimeter / step = 31.4 / 30
        self.step_distance = 1.0471975
        #: Chanel definition
        pin_encoder_right = 15  # 22
        pin_encoder_left = 7  # 4
        # chanel_encoder_right = 22
        # chanel_encoder_left = 4
        self.pin_encoder_right = pin_encoder_right #chanel_encoder_right
        self.pin_encoder_left = pin_encoder_left #chanel_encoder_left
        #: Distance covered
        self.distance_right = 0.0000000
        self.distance_left = 0.0000000
        #: Average speed
        self.speed_right = 0.0000000
        self.speed_left = 0.0000000
        #: Time
        self.start_time = time.time()

    def initEncoder(self):
        # With chanel handling
        GPIO.setmode(GPIO.BOARD) #BCM
        GPIO.setwarnings(False)
        # Pull up (to get 0 instead of high impedance state)
        GPIO.setup(self.pin_encoder_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_encoder_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Interruption configuration (event)
        GPIO.add_event_detect(self.pin_encoder_right, GPIO.FALLING,
                               callback=self.increase_distance_right, bouncetime=90)
        # Experimental measure: speed max => 100 => bouncetime max => 60
        GPIO.add_event_detect(self.pin_encoder_left, GPIO.FALLING,
                              callback=self.increase_distance_left, bouncetime=90)

    # Accessing methods
    def get_distance_right(self):
        return self.distance_right

    def get_distance_left(self):
        return self.distance_left

    def get_speed_right(self):
        delta = time.time() - self.start_time
        return self.get_distance_right() / delta

    def get_speed_left(self):
        delta = time.time() - self.start_time
        return self.get_distance_left() / delta

    # Setting functions
    def increase_distance_right(self, channel):
        # If they are 30 detections, whereas 1 round is covered
        self.distance_right = self.get_distance_right() + self.step_distance

    def increase_distance_left(self, channel):
        # If they are 30 detections, whereas 1 round is covered
        self.distance_left = self.get_distance_left() + self.step_distance

    # Predicates
    # Right is the handler wheel
    def reach_distance(self, distance):
        zero = self.get_distance_right()
        while (self.get_distance_right() - zero) < distance:
            return True
        return False
        

