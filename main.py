#!/usr/bin/env pybricks-micropython

# TODO - Eric, Can I really call it an API in this instance? 
'''
Team #C44519 Digital Voodo is using their 2022 Voodoo Ranger API.  This API is a Voodoo Ranger Specific class
inherited from the more general robot class for any robot with tank wheels, right and left line sensors, 
large and medium attachment motors, ultrasonic sensor, and gyro. 
'''
from vdr2022 import VDR2022
'''
Starting with a strategy similar to that of team #44519 Digital Magic in 2021, Digital Voodoo chose missions
based on trade-offs between complexity and points.
'''
from missions import eastern_adventure, blade, dead_reckon_helicopter
from test import *

# creating a Voodoo Ranger 2022 object here
# all the robot specific setting are CONSTANTS in the VDR class
vdr = VDR2022()

# Go Go Voodoo Ranger!
# FIXME - When I want to show off, make the Voodo Ranger speak! 
# vdr.say("Go Go Voodoo Ranger")
vdr.beep(200)


# creating a 2D list for the Voodoo Ranger menu system
# Columns ["Menu Name", Function to call, Parameter]

vdr_menu_list = [
    ["Test Stall",test_stall,vdr],
    ["Gyro Turn OC",test_gyro_turn_on_center,vdr], 
    ["Testing pid",testing_pid,vdr],  
    ["Gyro Straight",test_gyro_straight,vdr],
    ["Mission5",vdr.beep,400],
    ["Mission6",vdr.beep,500],
    ["Mission7",vdr.beep,600],
    ["Watch Sensors",vdr.watch_sensors,"dummy"]
]

vdr.menu_loop(vdr_menu_list)
