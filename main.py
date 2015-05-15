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

    s.activate_us_front()
    time.sleep(0.4)
    #m.vacuum_cleaner_start()
    Vxy = [('F', 400)]
    T = 0.4
    try:
        # Infinite and main loop
        while True:
            sup.detection_obstacle(m,e,s, Vxy)
            sup.m.move(e,Vxy)
            sup.motion_control(m,e,s)
            sup.distance_reach(m,e,s,Vxy)
            sup.cycle = sup.cycle + 1

    except KeyboardInterrupt:
        m.vacuum_cleaner_stop()
        m.stop_motors()
        s.unactivate_us_front()
        s.unactivate_us_right()
        s.unactivate_us_left()
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
        sys.exit(0)
    GPIO.cleanup()           # clean up GPIO on normal exit


# Algo
# Demarage
# Vérification position des codeurs tous sur blanc

#detection_obstacle(m)
# if e.is_one_round_left():
#     print(e.get_count_left())
#     m.moveForward(0)
#     time.sleep(3)
#     m.moveForward(60)
