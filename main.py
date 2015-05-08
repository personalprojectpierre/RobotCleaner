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

    # Right wheel is the target
    # Left wheel follow the right wheel
    s.activate_us_front()
    time.sleep(0.5)
    m.vacuum_cleaner_start()
    #s.unactivate_us_front()
    #m.move_forward(speed_left=45, speed_right=53)
    m.move_forward(speed_left=30, speed_right=45)
    try:
        # Infinite and main loop
        while True:
            test_line_moving(m, e, s)
            #sup.demo()
            sup.cycle = sup.cycle + 1

    # try:
    #     while 1:
    #         cmd = raw_input("Command, f/r 0..9, E.g. f5 :")
    #         direction = cmd[0]
    #         print(direction)
    #         if direction == 'g':
    #             m.move_forward(speed_left=60, speed_right=20)
    #         elif direction == 'd':
    #             m.turn_right(speed_left=20, speed_right=60)
    #         else:
    #             m.move_forward(speed_left=43, speed_right=53)
    #         print(direction)

    except KeyboardInterrupt:
        m.vacuum_cleaner_stop()
        m.stop_motors()
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