#!/usr/bin/env pybricks-micropython

'''
Team #C44519 Digital Voodo is using their 2022 Voodoo Ranger API.  This API is a Voodoo Ranger Specific class
inherited from the more general robot class for any robot with tank wheels, right and left line sensors, 
large and medium attachement motors, ultrasonic sensor, and gyro. 
'''
from vdr2022 import VDR2022
'''
Starting with a strategy similar to that of team #44519 Digital Magic in 2021, Digital Voodoo chose missions
based on trade-offs between complexity and points.
'''
from missions import eastern_adventure
from missions import dead_reckon_helicopter
from missions import blade

# creating the Voodoo Ranger 2022 object here
vdr = VDR2022()

# Go Go Voodoo Ranger! 
# vdr.say("Go Go Voodoo Ranger")
vdr.beep()


# creating a 2D list for the Voodoo Ranger menu system
vdr_menu_list = [
    ["Mission1",vdr.beep],
    ["Mission2",vdr.beep],
    ["Mission3",vdr.beep],   
    ["Mission4",vdr.beep],
    ["Mission5",vdr.beep],
    ["Mission6",vdr.beep],
    ["Mission7",vdr.beep],
    ["Mission8",vdr.beep],
    ["Mission9",vdr.beep],
    ["Mission10",vdr.beep],
    ["Mission11",vdr.beep],
    ["Mission12",vdr.beep],
    ["Mission13",vdr.beep],
    ["Mission14",vdr.beep],
    ["Watch Sensors",vdr.watch_sensors]
]

vdr.menu_loop(vdr_menu_list)
