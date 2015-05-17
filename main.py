#!/usr/bin/python
# -*- encoding: utf8 -*-
import sys
import os
import time

from Adafruit_MCP230xx import Adafruit_MCP230XX
import RPi.GPIO as GPIO

from Motor import Motor
from Encoder import Encoder
from Sonar import Sonar
from Supervisor import Supervisor
from test import *

if __name__ == '__main__':
    # MCP23017
    mcp = Adafruit_MCP230XX(busnum=1, address=0x20, num_gpios=16)

    # Objects declaration
    m = Motor(mcp)
    e = Encoder()
    s = Sonar(mcp)
    sup = Supervisor(m, e, s)

    # Initialisation
    m.init_motor()
    e.initEncoder()
    s.initSonar()

    # Ultrasonic sensors activation
    s.activate_us()

    # Vacuum cleaner starting
    #m.vacuum_cleaner_start()

    # Temporising before starting
    time.sleep(0.4)
    try:
        # Infinite and main loop
        while True:
            sup.detection_obstacle()
            sup.choose_direction()
            sup.motion_control()
            sup.distance_reach()
            sup.cycle = sup.cycle + 1

    except KeyboardInterrupt:
        m.vacuum_cleaner_stop()
        m.fast_motor_stop()
        m.stop_motors()
        s.unactivate_us()
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
        sys.exit(0)
    GPIO.cleanup()           # clean up GPIO on normal exit