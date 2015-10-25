#!/usr/bin/python
# -*- encoding: utf8 -*-
import RPi.GPIO as GPIO
import time


##
# @file Sonar.py
# @brief Supervise the sonars
# @author Pierre
# @version 0
# @date 31th july 2015
# Unix (Raspberry Pi Debian)

##
# @class Sonar
# @brief Class to represent the sonar
class Sonar(object):

    def __init__(self, mcp):
        ##
        # @Enum Init the class Sonar
        ## MCP3008 component
        self.pin_us_left  = 0  # pin_us_left
        self.pin_us_front = 1  # pin_us_front
        self.pin_us_right = 2  # pin_us_right
        ## MCP23017
        self.pin_us_e     = 8   # pin_us_enable
        ## SPI definition pins ADC definition
        pin_spi_cs   = 12  # channel 18
        pin_spi_mosi = 16  # channel 23
        pin_spi_miso = 18  # channel 24
        pin_spi_clk  = 22  # channel 25
        ## CS SPI
        self.pin_spi_cs = pin_spi_cs
        ## MOSI SPI
        self.pin_spi_mosi = pin_spi_mosi
        ## MISO SPI
        self.pin_spi_miso = pin_spi_miso
        ## CLK SPI
        self.pin_spi_clk = pin_spi_clk
        ## CLK MCP
        self.mcp = mcp

    def initSonar(self):
        self.mcp.config(self.pin_us_e,  self.mcp.OUTPUT)
        GPIO.setmode(GPIO.BOARD)  # != GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin_spi_mosi, GPIO.OUT)
        GPIO.setup(self.pin_spi_miso, GPIO.IN)
        GPIO.setup(self.pin_spi_clk,  GPIO.OUT)
        GPIO.setup(self.pin_spi_cs,   GPIO.OUT)

    ##
    #  @fn activate_us(self)
    #  @brief Activate each ultrasonic sensor
    #  @param self The object pointer.
    #  @return
    def activate_us(self):
        self.mcp.output(self.pin_us_e, True)

    ##
    #  @fn unactivate_us(self)
    #  @brief Deactivate each ultrasonic sensor
    #  @param self The object pointer.
    #  @return None
    def unactivate_us(self):
        self.mcp.output(self.pin_us_e, False)

    ##
    #  @fn readADC(self, adcnum)
    #  @brief SPI reading of the chip MCP3008 of the input adcnum (0 to 7)
    #  @params self The object pointer.
    #          adcnum
    #  @return value integer
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

    ##
    #  @fn get_cm(self, adcnum)
    #  @brief Read US
    #  @params self The object pointer.
    #          adcnum: integer 8 bits
    #  @return  distance in cm
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