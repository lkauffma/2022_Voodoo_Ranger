#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color 
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Used in motor_turn.  Working theory is affected by battery level.
#MOTOR_TURN_ANGLE_CALIBRATE = 1.069
#MOTOR_TURN_ANGLE_CALIBRATE = 1.15  # 7.53 volts
MOTOR_TURN_ANGLE_CALIBRATE = 1.06  # 8.29 volts

# Sensor values 
LEFT_SENSOR_BLACK = 8
LEFT_SENSOR_WHITE = 83
RIGHT_SENSOR_BLACK = 7
RIGHT_SENSOR_WHITE = 97


class Robot(): 

    def __init__(self):

        # Initialize the EV3.
        self.ev3 = EV3Brick()

        # Initialize the motors
        self.front_motor = Motor(Port.C)
        self.left_motor = Motor(Port.D)
        self.right_motor = Motor(Port.A)
        self.back_motor = Motor(Port.B)

        # Initialize the sensors
        self.right_line_sensor = ColorSensor(Port.S1)
        self.left_line_sensor = ColorSensor(Port.S4)
        self.gyro_sensor = GyroSensor(Port.S2, Direction.CLOCKWISE)
        self.ultrasonic_sensor=UltrasonicSensor(Port.S3)

        # Initialize the drive base

        # A robotic vehicle with two powered wheels and an optional support wheel or caster.
        # By specifying the dimensions of your robot, this class makes it easy to drive a given distance in millimeters 
        # or turn by a given number of degrees.

        # Positive distances and drive speeds mean driving forward. Negative means backward.

        # Positive angles and turn rates mean turning right. Negative means left. 
        # So when viewed from the top, positive means clockwise and negative means counterclockwise.

        # Parameters:	
        #    left_motor (Motor) – The motor that drives the left wheel.
        #    right_motor (Motor) – The motor that drives the right wheel.
        #    wheel_diameter (dimension: mm) – Diameter of the wheels.
        #    axle_track (dimension: mm) – Distance between the points where both wheels touch the ground.
 
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter=90, axle_track=127)

    def beep(self):
        self.ev3.speaker.beep()

    def motor_straight(self, distance, speed=100, acceleration=1000):

        # Drives straight for a given distance, speed and acceleration, then stops.
        # Uses Pybricks API PID algorithm based on internal motor rotation sensors.

        # distance: mm – distance travelled
        # speed: mm/s – speed of the robot
        # linear acceleration: mm/s/s – acceleration and deceleration of the robot at the start and end of straight().

        self.drive_base.stop()
        #DEFAULT self.drive_base.settings(straight_speed=209, straight_acceleration=837, turn_rate=272, turn_acceleration=1091)
        self.drive_base.settings(straight_speed=speed, straight_acceleration=acceleration, turn_rate=0, turn_acceleration=0)
        self.drive_base.straight(distance)

        #measured_distance, measured_speed, measured_angle, measured_turn_rate = self.drive_base.state()
        #print(distance, " compared to ", measured_distance)

        self.drive_base.stop()

    def motor_turn(self, angle, speed = 80, acceleration = 1000):

        # Turns in place by a given angle, speed, and acceleration and then stops.
        # Uses Pybricks API PID algorithm based on internal motor rotation sensors.

        # angle: degrees – turn angle
        # speed: mm/s – rate of turn
        # linear acceleration: mm/s/s – acceleration and deceleration of the robot at the start and end of turn().

        self.drive_base.stop()
        #DEFAULT self.drive_base.settings(straight_speed=209, straight_acceleration=837, turn_rate=272, turn_acceleration=1091)
        self.drive_base.settings(straight_speed=0, straight_acceleration=0, turn_rate=speed, turn_acceleration = acceleration)
        self.drive_base.turn(angle * MOTOR_TURN_ANGLE_CALIBRATE)

        #print(self.drive_base.state())
        self.drive_base.stop()

    def pid_line_follow(self, distance, speed = 80, right_or_left_sensor = "right", side_of_line = "left", 
                        Proportional_Gain = 0.8, Integral_Gain = 0.0008, Derivative_Gain=.001):
        # PID Line Follower 
        # Mark Lucking says Builder Dude says don't use Integral_Gain if mostly straight line.
        # Seems like the faster you go, the more Derivative_Gain you need


        integral = 0
        derivative = 0
        last_error = 0

        
        if (right_or_left_sensor == "right"):
            sensor = self.right_line_sensor
            target = (RIGHT_SENSOR_WHITE + RIGHT_SENSOR_BLACK) / 2
        else:
            sensor = self.left_line_sensor
            target = (LEFT_SENSOR_WHITE + LEFT_SENSOR_BLACK) / 2

        self.drive_base.reset()
        self.drive_base.stop()

        # PID feedback loop
        while self.drive_base.state()[0] < distance:
            error = sensor.reflection() - target
            integral = integral + error
            derivative = error - last_error
            turn_rate = Proportional_Gain * error + Integral_Gain * integral + Derivative_Gain * derivative
            if side_of_line == "left":
                self.right_motor.run(speed - turn_rate)
                self.left_motor.run(speed + turn_rate)
            else:
                self.right_motor.run(speed + turn_rate)
                self.left_motor.run(speed - turn_rate)
            last_error = error
            wait(10)

        self.drive_base.stop()

    def follow_line_v1(self, distance, speed = 120, right_or_left_sensor = "right", side_of_line = "left", 
                        Kp = 0.8, Ki = 0.0008, Kd =.001):
        """
        Digital Magic's PID Follow Line Function Version 1

        History:
           This version of the function was created by Lily Hill in January 2022 before the state tournament after the team 
           added a second color sensor to the other side of the robot.
          
        Notes:
            You can only change the settings while the robot is stopped. This is either before you begin driving or after you call stop().
            Builder Dude on Youtube said don't bother with the Integral part, but I put it in here anyway so we could try it.

        Parameters:
            straight_speed (int):  Speed of the robot during straight() in millimeters/second.
            straight_acceleration (int): Acceleration and deceleration of the robot at the start and end of straight() in millimeters/second^2.
            turn_rate (int): Turn rate of the robot during turn() in degrees/second.
            turn_acceleration (int): Angular acceleration and deceleration of the robot at the start and end of turn() in degrees/second^2.

        Returns:
            It does not return any values
        """

        integral = 0
        derivative = 0
        last_error = 0


        

        if (right_or_left_sensor == "right"):
            sensor = self.right_line_sensor
            target = (RIGHT_SENSOR_WHITE + RIGHT_SENSOR_BLACK) / 2
        else:
            sensor = self.left_line_sensor
            target = (LEFT_SENSOR_WHITE + LEFT_SENSOR_BLACK) / 2

        f = open('data1.csv', 'w')
        t = open('title.txt','w')
        t.write('speed:' + str(speed) + ' Kp:' + str(Kp) +  ' Ki:' +  str(Ki) + ' Kd:' + str(Kd))
        t.close()
            
        f.write("Reading, Target, Sensor"+"\n")
        

        self.drive_base.reset()
        self.drive_base.stop()

        i=0

        # PID feedback loop
        while self.drive_base.state()[0] < distance:
            i = i + 1
            error = sensor.reflection() - target
            integral = integral + error
            derivative = error - last_error

            magic_adjustment = Kp * error + Ki * integral + Kd * derivative

            f.write(str(i) + ","  + str(target) + "," + str(sensor.reflection())+"\n")

            if side_of_line == "left" or side_of_line == "Left" or side_of_line == "l" or side_of_line == "L":
                self.right_motor.run(speed - magic_adjustment)
                self.left_motor.run(speed + magic_adjustment)
            elif side_of_line == "right" or side_of_line == "Right" or side_of_line == "r" or side_of_line == "R":
                self.right_motor.run(speed + magic_adjustment)
                self.left_motor.run(speed - magic_adjustment)
            else:
                print("ERROR - left or right side of line?  Check case")

            last_error = error
            wait(10)

        f.close()

        self.drive_base.stop()

    def pid_gyro_straight(self, distance, Proportional_Gain = 1, Integral_Gain = 0, Derivative_Gain = 0):
        
        self.gyro_sensor.reset_angle(0)
        
        error = 0
        last_error = 0
        integral = 0
        derivative = 0

        # PID feedback loop
        while self.drive_base.state()[0] < distance:
            angle = self.gyro_sensor.angle()
            error = 0 - angle
            #Holy Crap, we have a pattern here!  I am writing the same code twice. 

    def erickson(self, speed, degrees):
        self.attachment_motor.run_angle(speed, degrees)

    def drive (self, distance = 100, speed = 120, turn_rate = 0):
        # negative speed is reverse

        self.drive_base.reset()  
        self.drive_base.stop()
        
        if speed > 0:
            while self.drive_base.state()[0] < distance:
                print(self.drive_base.state()[0],distance)
                self.drive_base.drive(speed, turn_rate)
        else:
            while (self.drive_base.state()[0] * -1) < distance:
                print(self.drive_base.state()[0],distance)
                self.drive_base.drive(speed, turn_rate) 

        self.drive_base.stop()

    def wait(self,time):
        wait(time)

    def test(self):
        pass

    def watch_sensors(self):
        self.wait(1000)
        self.gyro_sensor.reset_angle(0)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)


        while (self.ev3.buttons.pressed() == []):
            self.ev3.screen.clear()
            self.ev3.screen.draw_text(1, 1, "Gyro:")
            self.ev3.screen.draw_text(100, 1, self.gyro_sensor.angle())
            self.ev3.screen.draw_text(1, 20, "R Line:")
            self.ev3.screen.draw_text(100, 20, self.right_line_sensor.reflection())
            self.ev3.screen.draw_text(1, 40, "L Line:")
            self.ev3.screen.draw_text(100, 40, self.left_line_sensor.reflection())
            self.ev3.screen.draw_text(1, 60, "R Motor:")
            self.ev3.screen.draw_text(100, 60, self.right_motor.angle())
            self.ev3.screen.draw_text(1, 80, "L Motor:")
            self.ev3.screen.draw_text(100, 80, self.left_motor.angle())
            self.ev3.screen.draw_text(1, 100, "V (8231):")
            self.ev3.screen.draw_text(100, 100, self.ev3.battery.voltage())
            wait(100)
    
        # Now wait for the button to be released, from example code.  If you don't do this, the button that ends it executes next loop.
        while any(self.ev3.buttons.pressed()):
            pass           