#!/usr/bin/env pybricks-micropython

'''
This is team #C44519 Digital Voodo's 2022 Voodoo Ranger API.  This API is a Voodoo Ranger Specific class
inherited from the more general robot class for any robot with tank wheels, right and left line sensors, 
ultrasonic sensor, and gyro. 
'''
from robot import Robot

class VDR2022(Robot):
    def __init__(self):
        super().__init__()
