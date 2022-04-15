#!/usr/bin/env pybricks-micropython

'''
This is team #C44519 Digital Voodo's 2022 Voodoo Ranger API.  This API is a Voodoo Ranger Specific class
inherited from the more general robot class for any robot with tank wheels, right and left line sensors, 
ultrasonic sensor, and gyro. 
'''
from robot import Robot

# 2022 Voodoo Ranger Settings

WHEEL_DIAMETER = 90         # (dimension: mm) – Diameter of the wheels.
AXLE_TRACK = 127            # (dimension: mm) – Distance between the points where both wheels touch the ground.
LEFT_SENSOR_BLACK = 8       # reading when left sensor is on black line   
LEFT_SENSOR_WHITE = 83      # reading when left sensor is on white line
RIGHT_SENSOR_BLACK = 7      # reading when right sensor is on black line 
RIGHT_SENSOR_WHITE = 97     # reading when right sensor is on white line


class VDR2022(Robot):
    def __init__(self):
        super().__init__(
            wheel_diameter = WHEEL_DIAMETER, 
            axle_track = AXLE_TRACK,
            left_sensor_black = LEFT_SENSOR_BLACK,
            left_sensor_white = LEFT_SENSOR_WHITE,
            right_sensor_black = RIGHT_SENSOR_BLACK,
            right_sensor_white = RIGHT_SENSOR_WHITE
        )

    def test_rear_attachment1(self):
 
        self.medium_attachment_motor.run_angle(1000, 601.2, then=self.Stop.HOLD, wait=True)

    '''
    def test_rear_attachment2(self):
        # SAVE FOR THAT FACTOR
        factor = 1.67
        desired_angle = 360
        calibrated_angle = desired_angle * factor
        print(calibrated_angle)

        Robot.run_angle(speed=-500,rotation_angle=calibrated_angle, wait=True)
        Robot.run_angle(speed=-1000,rotation_angle=calibrated_angle, wait=True)
        Robot.run_angle(speed=1000,rotation_angle=calibrated_angle, wait=True)
        Robot.run_angle(speed=1000,rotation_angle=calibrated_angle, wait=True)
    '''