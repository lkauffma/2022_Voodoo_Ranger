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
from missions import eastern_adventure
from missions import dead_reckon_helicopter
from missions import blade, test

# creating a Voodoo Ranger 2022 object here
# all the robot specific setting are CONSTANTS in the VDR class
vdr = VDR2022()

# Go Go Voodoo Ranger!
# FIXME - When I want to show off, make the Voodo Ranger speak! 
# vdr.say("Go Go Voodoo Ranger")
vdr.beep(200)

# FIXME - Remove these pre-menu test function calls in the end
test(vdr)


# creating a 2D list for the Voodoo Ranger menu system
# Columns ["Menu Name", Function to call, Parameter]
# TODO - Change Parameters to a list
vdr_menu_list = [
    ["Mission1",vdr.beep,100],
    ["Test",test,vdr],
    ["Mission3",vdr.beep,200],   
    ["Mission4",vdr.beep,300],
    ["Mission5",vdr.beep,400],
    ["Mission6",vdr.beep,500],
    ["Mission7",vdr.beep,600],
    ["Mission8",vdr.beep,700],
    ["Mission9",vdr.beep,800],
    ["Mission10",vdr.beep,900],
    ["Mission11",vdr.beep,200],
    ["Mission12",vdr.beep,200],
    ["Mission13",vdr.beep,200],
    ["Mission14",vdr.beep,200],
    ["Watch Sensors",vdr.watch_sensors,"dummy"]
]

vdr.menu_loop(vdr_menu_list)
