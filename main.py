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


# TO DO developing menu - move this to robot or vdt
print(vdr.wait_for_button("junk1", "junk2", "junk3"))u

# creating a 2D list for the Voodoo Ranger menu system
vdr_menu = [
    ["Mission1","vdr.beep"]
    ["Mission2","vdr.beep"]
    ["Watch Sensors","watch_sensors"]
    ["Mission4","vdr.beep"]
    ["Mission5","vdr.beep"]
    ["Mission6","vdr.beep"]
]

vdr.menu_loop(vdr_menu)


'''
while True:
    # Draw screen based on what run we are on
    if run_number == 0:
        make_screen(ev3,"New Run", " +  -  -  -  -  -  -",  "Right Sensor", "Arm Down","You Got This!","Go Digital Magic!")

    elif run_number == 1:
        make_screen(ev3,"Platooning Trucks", " -  +  -  -  -  -  -", "Load Truck", "Right Sensor","Left Side Ln","Returns Hot")

    elif run_number == 2:
        make_screen(ev3,"Flip Engine", " -  -  +  -  -  -  -", "Right Sensor", "Left Side Line","Fill Blue","Returns Hot")

    elif run_number == 3:
        make_screen(ev3,"Cargo Plane", " -  -  -  +  -  -  -", "Aim for Bar", "Arm Up","Let's Go!","Returns Hot")

    elif run_number == 5:
        make_screen(ev3,"Deliver Cargo", " -  -  -  -  +  -  -", "Aim for Circle", "Dance","Cheer","Returns Hot")

    elif run_number == 6:
        make_screen(ev3,"Innovation Model", " -  -  -  -  -  +  -", "Package!", "Just in Bounds","Three Sides","Rescue?")
    
    elif run_number == 4:
        make_screen(ev3,"Blade"," -  -  -  -  -  -  + ","Bumper 2 Line", "Keep Orange In"," "," ")

    elif run_number == 7:
        make_screen(ev3,"Watch Sensors"," -  -  -  -  -  -  + ","", ""," "," ")

    # Wait for one button to be selected.
    button = wait_for_button(ev3)

    # Now you can do something, based on which button was pressed.
    if button == Button.LEFT:
        if run_number > 0: 
            run_number = run_number - 1
        else:
            run_number = last_run_number

    elif button == Button.RIGHT:
        if run_number < last_run_number: 
            run_number = run_number + 1
        else:
            run_number = 0

    elif button == Button.UP:
        if run_number > 0: 
            run_number = run_number - 1
        else:
            run_number = last_run_number

    elif button == Button.DOWN:
        if run_number < last_run_number: 
            run_number = run_number + 1
        else:
            run_number = 0

    elif button == Button.CENTER:
        if run_number == 0:
            new_run()
            #followline2( 1300, speed = 120, right_or_left_sensor = "left", side_of_line = "left", Kp = 1.0, Ki = 0.0008, Kd =.001)
        elif run_number == 1:
            plattooning_trucks()

        elif run_number == 2:
            flip_engine()

        elif run_number == 3:
            cargo_plane()

        elif run_number == 5:
            connect_cargo()

        elif run_number == 6:
            innovation_model()

        elif run_number == 4:
            blade()

        elif run_number == 7:
            watch_sensors()

        # Move on to next run screen
        if run_number < last_run_number: 
            run_number = run_number + 1
        else:
            run_number = 0 
''' 