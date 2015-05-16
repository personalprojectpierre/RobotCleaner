#!/usr/bin/python
# -*- encoding: utf8 -*-
import RPi.GPIO as GPIO
import time

class Sonar(object):
    def __init__(self, mcp):
        # MCP3008 component
        # ADC definition
        self.pin_us_front = 0  # pin_us_front
        self.pin_us_left  = 1  # pin_us_left
        self.pin_us_right = 2  # pin_us_right
        self.pin_us_front_e = 8   # pin_us_front_e
        self.pin_us_left_e  = 9   # pin_us_left_e
        self.pin_us_right_e = 10  # pin_us_right_e
        # SPI definition pins
        pin_spi_cs   = 12  # channel 18
        pin_spi_mosi = 16  # channel 23
        pin_spi_miso = 18  # channel 24
        pin_spi_clk  = 22  # channel 25
        #: pin_motor_left_e
        self.pin_spi_cs = pin_spi_cs
        #: pin_motor_left_a
        self.pin_spi_mosi = pin_spi_mosi
        #: pin_motor_left_e
        self.pin_spi_miso = pin_spi_miso
        #: pin_motor_left_a
        self.pin_spi_clk = pin_spi_clk
        self.mcp = mcp

    def initSonar(self):
        self.mcp.config(self.pin_us_front_e,  self.mcp.OUTPUT)
        self.mcp.config(self.pin_us_left_e,   self.mcp.OUTPUT)
        self.mcp.config(self.pin_us_right_e,  self.mcp.OUTPUT)
        GPIO.setmode(GPIO.BOARD)  # != GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin_spi_mosi, GPIO.OUT)
        GPIO.setup(self.pin_spi_miso, GPIO.IN)
        GPIO.setup(self.pin_spi_clk,  GPIO.OUT)
        GPIO.setup(self.pin_spi_cs,   GPIO.OUT)

    """
    ..function:: activate_us(self)
    Activate each ultrasonic sensor
       :rtype: None
       :return: None
    """
    def activate_us(self):
        self.mcp.output(self.pin_us_front_e, True)
        self.mcp.output(self.pin_us_left_e,  True)
        self.mcp.output(self.pin_us_right_e, True)

    """
    ..function:: deactivate_us(self)
    Deactivate each ultrasonic sensor
       :rtype: None
       :return: None
    """
    def unactivate_us(self):
        self.mcp.output(self.pin_us_right_e, False)
        self.mcp.output(self.pin_us_front_e, False)
        self.mcp.output(self.pin_us_left_e,  False)

    """
    ..function:: readADC(self, adcnum)
    SPI reading of the chip MCP3008 of the input adcnum (0 to 7)
    return: value
    rtype: int
    """
    def readADC(self, adcnum):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(self.pin_spi_cs,    True)
        GPIO.output(self.pin_spi_clk, False)  # start clock low
        GPIO.output(self.pin_spi_cs,    False)   # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(self.pin_spi_mosi, True)
                else:
                        GPIO.output(self.pin_spi_mosi, False)
                commandout <<= 1
                GPIO.output(self.pin_spi_clk, True)
                GPIO.output(self.pin_spi_clk, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(self.pin_spi_clk, True)
                GPIO.output(self.pin_spi_clk, False)
                adcout <<= 1
                if (GPIO.input(self.pin_spi_miso)):
                        adcout |= 0x1

        GPIO.output(self.pin_spi_cs, True)

        adcout /= 2       # first bit is 'null' so drop it
        return adcout

    """
    ..function:: activate_us(self)
    Activate each ultrasonic sensor
       :type adcnum: integer 8 bits
       :param adcnum: distance value
       :rtype: integer
       :return: distance in cm
    """
    def get_cm(self, adcnum):
        inch = 3300.0/512.0
        read_adc0 = self.readADC(adcnum)
        mV = read_adc0 * (3300.0 / 1024.0)
        dist0 = mV/inch
        dist1 = dist0 * 2.54
        return dist1

    def getUSFront(self):
        cm = self.get_cm(self.pin_us_front)
        print("Front Obstacle:" +str(cm)+"cm")
        return cm

    def getUSLeft(self):
        cm = self.get_cm(self.pin_us_left)
        print("Left Obstacle:" +str(cm)+"cm")
        return cm

    def getUSRight(self):
        cm = self.get_cm(self.pin_us_right)
        print("Right Obstacle:" +str(cm)+"cm")
        return cm