#!/usr/bin/python
# -*- encoding: utf8 -*-

# File: test
# Author: Pierre

import sys
import time

import RPi.GPIO as GPIO

def test_number_round_separately(m, e, s):

    print("- right: "+str(e.get_distance_right()))
    if e.get_distance_right() > 450:
            m.fast_motor_stop_right()

    print("+ left: "+str(e.get_distance_left()))
    if e.get_distance_left() > 450:
            m.fast_motor_stop_left()
    # Conclusion:
    # 38 - 53 left
    # 41 - 53 left
    # Unstable, Seems to change in function of the battery level
    # Distance covered seems good < 5%
    # Need to set a regulation loop to control motors velocity

def test_line_distance(m, e, s):
    dist_target = 225.0000000
    dist_right = e.get_distance_right()
    dist_left = e.get_distance_left()
    print("Dist_right: "+str(dist_right)+" dist_left: "+str(dist_left))
    if (dist_left >= dist_target) or (dist_right >= dist_target):
        delta = dist_right-dist_left
        print("Dist right: "+str(dist_right)+" Dist left: "+str(dist_left))
        print("Delta: "+str(delta))
        m.fast_motor_stop()
        m.vacuum_cleaner_stop()
        time.sleep(2)
        m.stop_motors()
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
        sys.exit(0)
        # Conclusion: It depends on the bouncetime and speed

def test_line_moving(m, e, s):
    dist_target = 225.0000000
    dist_right = e.get_distance_right()
    dist_left = e.get_distance_left()
   # print("Dist_right: "+str(dist_right)+" dist_left: "+str(dist_left))
    print("motor_right: "+str(m.speed_right)+" motor_left: "+str(m.speed_left))
   # m.enslavement_position(target=dist_right, process_variable=dist_left)
    speed_right = e.get_speed_right()
    speed_left = e.get_speed_left()

    #print("Speed_right_e: "+str(speed_right)+" Speed_left_e: "+str(speed_left))
    #m.enslavement_left(target=10, process_variable=speed_left)
    #m.enslavement_right(target=10, process_variable=speed_right)

    if (dist_left >= dist_target) or (dist_right >= dist_target):
        delta = dist_right-dist_left
        print("Dist right: "+str(dist_right)+" Dist left: "+str(dist_left))
        print("Delta: "+str(delta))
        m.fast_motor_stop()
        time.sleep(2)
        m.stop_motors()
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
        sys.exit(0)
    # Conclusion:
    # Too much reactive


def test_round_distance(m, e, s):
    #m.change_speed_right(m.speed_right) 53
    #m.change_speed_left(m.speed_left)  33
    dist_right = e.get_distance_right()
    dist_left = e.get_distance_left()
    delta = dist_right-dist_left
    if (dist_left >= 450.0) or (dist_right >= 450.0):
        print("Delta: "+str(delta))
        m.fast_motor_stop()
        time.sleep(2)
        m.stop_motors()
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
        sys.exit(0)



#         import time
# import RPi.GPIO as GPIO
#
# GPIO.setmode(GPIO.BOARD)
#
#
# dc = 100 # Duty cycle
# in1_pin = 11
# in2_pin = 13
#
# GPIO.setup(in1_pin, GPIO.OUT)
# GPIO.setup(in2_pin, GPIO.OUT)
#
# p = GPIO.PWM(in1_pin, 500) # channel=4  frequency=500Hz
#
# p.start(0)
# p.ChangeDutyCycle(dc)
# try:
#     while 1:
#         print("")
#         cmd = raw_input("Command, f/r 0..9, E.g. f5 :")
#         direction = cmd[0]
#         speed = int(cmd[1]) * 11
#         print(direction)
#         print(speed)
#         p.ChangeDutyCycle(speed)
#         time.sleep(2)
# except KeyboardInterrupt:
#     pass
# p.stop()
# GPIO.cleanup(


#
# cmd = raw_input("Command, f/r 0..9, E.g. f5 :")
# direction = cmd[0]
# speed = int(cmd[1])
# print(direction)
# if direction == '-':
#     velocity = velocity - speed
# else:
#     velocity = velocity + speed
# print(speed)
# print(velocity)
# m.change_speed_right(velocity)








# #!/usr/bin/python
# # -*- encoding: utf8 -*-
# import sys
# import time
#
# from Adafruit_MCP230xx import Adafruit_MCP230XX
# import RPi.GPIO as GPIO
#
# from Encoder import Encoder
#
# if __name__ == '__main__':
#     # MCP23017
#     mcp = Adafruit_MCP230XX(busnum=1, address=0x20, num_gpios=16)
#
#     def droite(channel):
#         print("droite")
#
#     def gauche(channel):
#         print("Gauche")
#
#     # Objects declaration
#     e = Encoder()
#     # Initialisation
#     e.initEncoder()
#
#     raw_input("Press Enter when ready\n>")
#
#     GPIO.add_event_detect(e.pin_encoder_right, GPIO.BOTH, callback=droite, bouncetime=300)
#     GPIO.add_event_detect(e.pin_encoder_left, GPIO.BOTH, callback=gauche, bouncetime=300)
#
#     try:
#         # Infinite and main loop
#         while True:
#             print("- Main Loop : Start -")
#             # GPIO.wait_for_edge(e.pin_encoder_right, GPIO.FALLING)
#             # GPIO.wait_for_edge(e.pin_encoder_left, GPIO.FALLING)
#             print("- Main Loop : End -")
#             time.sleep(1)
#     except KeyboardInterrupt:
#         GPIO.cleanup()       # clean up GPIO on CTRL+C exit
#         sys.exit(0)
#     GPIO.cleanup()           # clean up GPIO on normal exit





__author__ = 'Kazatchok'
            # print(e.get_count_left())
            # if e.get_count_left() > e.step:
            #     m.fast_motor_stop()
            #     time.sleep(5)
            #     m.stop_motors()
            #     GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            #     sys.exit(0)


            # print(e.get_count_right())
            # if e.get_count_right() > e.step:
            #     m.fast_motor_stop()
            #     time.sleep(5)
            #     m.stop_motors()
            #     GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            #     sys.exit(0)

            # Basic enslavement: PROPORTIONAL

            # if delta > 20:
            #     print('a')
            #     speed_left = speed_left + 1
            #     if speed_left >= 100:
            #         speed_left = 100
            #     m.change_speed_left(speed_left)
            # if delta < -20:
            #     print('b')
            #     speed_left = speed_left - 1
            #     if speed_left <= 0:
            #         speed_left = 0
            #     m.change_speed_left(speed_left)
            # time.sleep(0.25)



            # dist_right = e.get_distance_right()
            # dist_left = e.get_distance_left()
            # delta = dist_right-dist_left
            # if (dist_left >= 100.0) or (dist_right >= 100.0):
            #     print("Delta: "+str(delta))
            #     m.fast_motor_stop()
            #     time.sleep(2)
            #     m.stop_motors()
            #     GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            #     sys.exit(0)
            # time.sleep(1)





            #Final test
            #
            #             print(e.get_count_right())
            # if e.get_count_right() > e.step:
            #     m.fast_motor_stop_right()
            #
            # print(e.get_count_left())
            # if e.get_count_left() > e.step:
            #     m.fast_motor_stop_left()
            #
            # dist_right = e.get_distance_right()
            # dist_left = e.get_distance_left()
            # delta = dist_right-dist_left
            # if (dist_left >= 30.0) or (dist_right >= 30.0):
            #     print("Delta: "+str(delta))
            #     m.fast_motor_stop()
            #     time.sleep(2)
            #     m.stop_motors()
            #     GPIO.cleanup()       # clean up GPIO on CTRL+C exit
            #     sys.exit(0)
            # time.sleep(1)

    #
    # Final test
    #
    #             speed_right = 70
    # speed_left = 30
    # m.move_forward()
    # m.change_speed_right(speed_right)
    # m.change_speed_left(speed_left)
    # print("Right velocity: "+str(speed_right))
    # print("Left velocity: "+str(speed_left))
    # try:
    #     # Infinite and main loop
    #     while True:
    #
    #         dist_right = e.get_distance_right()
    #         dist_left = e.get_distance_left()
    #         if (dist_left >= 100.0000000) or (dist_right >= 100.0000000):
    #             m.fast_motor_stop()
    #             delta = dist_right-dist_left
    #             print("Delta: "+str(delta))
    #             print(e.get_count_right())
    #             print(e.get_count_left())
    #             time.sleep(2)
    #             m.stop_motors()
    #             GPIO.cleanup()       # clean up GPIO on CTRL+C exit
    #             sys.exit(0)
    #
    # except KeyboardInterrupt:
    #     m.vacuum_cleaner_stop()
    #     m.stop_motors()
    #     GPIO.cleanup()       # clean up GPIO on CTRL+C exit
    #     sys.exit(0)
    # GPIO.cleanup()           # clean up GPIO on normal exit