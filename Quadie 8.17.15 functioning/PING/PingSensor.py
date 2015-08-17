#!/usr/bin/python
##############################################################################################
#   author: Alexander Sanchez
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
import Adafruit_BBIO.GPIO as GPIO
import time


TRIG = "P8_7"
ECHO = "P8_9"
GPIO.cleanup()
pulse_start = 0.0
pulse_end = 0.0
counter = 0
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.output(TRIG, GPIO.LOW)
while(True):
    GPIO.output(TRIG,GPIO.HIGH)   
    GPIO.output(TRIG,GPIO.LOW) 
    time.sleep(0.00001)
    GPIO.output(TRIG,GPIO.HIGH)
    while GPIO.input(ECHO) == 0 and counter < 10000:
        pulse_start = time.time()
        counter += 1
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    counter = 0
    pulse_duration = pulse_end - pulse_start
    #print "%f"%pulse_duration
    distance = (pulse_duration/2) / 29.1
    distance = pulse_duration * 17150
    distance = distance *  0.3937
    print "inches =%f "%distance
