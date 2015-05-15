#!/usr/bin/python
# -*- encoding: utf8 -*-

# File: test
# Author: Pierre

import sys
import time

import RPi.GPIO as GPIO

"""
    Conclusion:
    38 - 53 left
    41 - 53 left
    Unstable, Seems to change in function of the battery level
    Distance covered seems good < 5%
    Need to set a regulation loop to control motors velocity
"""
def test_number_round_separately(m, e, s):

    print("- right: "+str(e.get_distance_right()))
    if e.get_distance_right() > 450:
            m.fast_motor_stop_right()

    print("+ left: "+str(e.get_distance_left()))
    if e.get_distance_left() > 450:
            m.fast_motor_stop_left()

"""
    dist_target = 225.0000000  # Distance to cover
    test_line_distance(m, e, s, dist_target)
    cf EncoderWithoutBounceTime.xlsx
"""
def test_line_distance(m, e, s, dist_target):
    dist_right = e.get_distance_right()
    dist_left = e.get_distance_left()
    my_file = open("line_distance.txt", "a")
    my_file.write(str(dist_right)+" "+str(dist_left)+"\n")
    my_file.close()
    if (dist_left >= dist_target) or (dist_right >= dist_target):
        delta = dist_right-dist_left
        print("Dist right: "+str(dist_right)+" Dist left: "+str(dist_left))
        print("Delta: "+str(delta))
        m.fast_motor_stop()
        m.vacuum_cleaner_stop()
        time.sleep(2)
        m.stop_motors()
        GPIO.cleanup()
        sys.exit(0)
        # Conclusion: It depends on the bouncetime and speed

def test_line_moving(m, e, s, speed_target):
    m.regulation(e, speed_target)

"""
    test_echelon(m, e, s, cycle=sup.cycle)
"""
def test_echelon(m,e,s, cycle):
    if cycle < 30000 or cycle > 200000:
        speed_target = 0.0000000
        test_line_moving(m, e, s, speed_target)
    else:
        speed_target = 70.0000000
        test_line_moving(m, e, s, speed_target)

def test_distance_is_covered(m, e, s):
    dist_right = e.get_distance_right()
    dist_left = e.get_distance_left()
    delta = dist_right-dist_left
    if (dist_left >= 600.0) or (dist_right >= 600.0):
        print("Delta: "+str(delta))
        m.fast_motor_stop()
        time.sleep(2)
        m.stop_motors()
        GPIO.cleanup()
        sys.exit(0)

# Open an file to record the Distance = f(PWM)
def test_linearity_motor_encoder(m,e,s):
    # Test Encoder = f(Motor)
    my_file = open("MotEnc.txt", "a")
    for speed in range(100):
        m.move_forward(speed_left=speed, speed_right=speed)
        time.sleep(0.5)  # Stabilize speed
        speed_right = e.get_speed_right()
        speed_left = e.get_speed_left()
        my_file.write(str(speed)+" "+str(speed_right)+" "+str(speed_left)+"\n")
    my_file.close()
    m.vacuum_cleaner_stop()
    m.stop_motors()
    GPIO.cleanup()
    sys.exit(0)

def test_encoder_reaction_to_speed_variation(m,e,s,cycle):
    val = 22000
    a= 1
    for i in range(1,8):
        if cycle > i*val and cycle < (i+1)*val and a:
            m.move_forward(speed_left=m.get_speed_left()+10,
                           speed_right=m.get_speed_right()+10)
            a = a^1
        if cycle > (i+1)*val and cycle < (i+2)*val and not a:
            m.move_forward(speed_left=28, speed_right=35)
            a = a^1


# import time
# import RPi.GPIO as GPIO
# GPIO.setmode(GPIO.BOARD)
# dc = 100 # Duty cycle
# in1_pin = 11
# in2_pin = 13
# GPIO.setup(in1_pin, GPIO.OUT)
# GPIO.setup(in2_pin, GPIO.OUT)
# p = GPIO.PWM(in1_pin, 500) # channel=4  frequency=500Hz
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