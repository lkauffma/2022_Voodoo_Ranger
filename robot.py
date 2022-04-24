#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color 
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font

'''
This class can be used for any FLL robot.  Combines the different Pybricks API classes into one robot class.   

    Initially, it is being written for robots with:
        2 large driving motors
        1 large attachment motor
        1 medium attachment motor
        2 light sensors for line tracking
        1 ultrasonic sensor
        1 gyro sensor

    # TODO - The ports are hardcoded in the constructor below.  Eventually, a person would want to initialize this class 
    from child class (motors and sensors are being used at what port). A 2-D List.  The idea is this become totally
    reuable from year to year, robot to robot, without change.  

    Toying with the idea I am creating an abstraction class that is more easy for 4th - 8th grade to use by themselves.  
    One that uses distance traveled and stall detection.  But, mostly this is helping me learn what is most important to
    help coach them better.    
'''

# Used in motor_turn.  Working theory is affected by battery level.
# TODO - Move to Child Class
#MOTOR_TURN_ANGLE_CALIBRATE = 1.069
#MOTOR_TURN_ANGLE_CALIBRATE = 1.15  # 7.53 volts
MOTOR_TURN_ANGLE_CALIBRATE = 1.06  # 8.29 volts

# Default Robot Settings if not specified in child class
WHEEL_DIAMETER = 50
AXLE_TRACK = 100
LEFT_SENSOR_BLACK = 5
LEFT_SENSOR_WHITE = 90
RIGHT_SENSOR_BLACK = 5
RIGHT_SENSOR_WHITE = 90


class Robot(): 

    def __init__(
            self, 
            wheel_diameter = WHEEL_DIAMETER, 
            axle_track = AXLE_TRACK,
            left_sensor_black = LEFT_SENSOR_BLACK,
            left_sensor_white = LEFT_SENSOR_WHITE,
            right_sensor_black = RIGHT_SENSOR_BLACK,
            right_sensor_white = RIGHT_SENSOR_WHITE
        ):

        # Initialize the EV3.
        self.ev3 = EV3Brick()

        # Intitalize the Parameters
        self.Stop = Stop

        # Initialize the light sensor black and white
        self.left_sensor_black = left_sensor_black
        self.left_sensor_white = left_sensor_white
        self.right_sensor_black = right_sensor_black
        self.right_sensor_white = right_sensor_white

        # Initialize the motors
        self.large_attachment_motor = Motor(Port.C)
        self.left_motor = Motor(Port.D)
        self.right_motor = Motor(Port.A)
        self.medium_attachment_motor = Motor(Port.B) 

        # Messed with gear train list parameter but didn't work as expected
        # I might not understand how it supposed to work
        # for now just put a factor on term angle to compensate
        # self.medium_attachment_motor = Motor(Port.B, gears=[12, 20]) 

        # Initialize the sensors
        self.right_line_sensor = ColorSensor(Port.S1)
        self.left_line_sensor = ColorSensor(Port.S4)
        self.gyro_sensor = GyroSensor(Port.S2, Direction.CLOCKWISE)
        self.ultrasonic_sensor=UltrasonicSensor(Port.S3)

        # Initialize the drive base from the Pybricks Documentation:

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
 
        
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter, axle_track)

        # FIXME - Remove debugging / learning code when no longer relevant
        # self.medium_attachment_motor.control.limits(speed=5000)
        print("medium motor max Speed = " + str(self.medium_attachment_motor.control.limits()[0]))
        print("large motor max Speed = " + str(self.large_attachment_motor.control.limits()[0]))

#TODO Legacy Methods Eliminate or rebuild ==========================================================================
    def motor_straight(self, distance, speed=100, acceleration=1000):

        # FIXME - Stops and Stalls
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
        # FIXME - Stops and Stalls

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

    def follow_line_v1(self, distance, speed = 120, right_or_left_sensor = "right", side_of_line = "left", 
                        Kp = 0.8, Ki = 0.0008, Kd =.001):
        """
        Digital Magic's PID Follow Line Function Version 1

        History:
           This version of the function was created by Koen & Coach in January 2022 before the state tournament after the team 
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

        # TODO - Determine which code to keep?
        # FIXME - Stops and Stalls
        # TODO - Move PID to own class

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

    def pid_gyro_straight(self, distance, PID = [0,0,0]):
        
        self.gyro_sensor.reset_angle(0)
        
        error = 0

        # PID feedback loop
        while self.drive_base.state()[0] < distance:
            angle = self.gyro_sensor.angle()
            error = 0 - angle
            #Holy Crap, we have a pattern here!  I am writing the same code twice. 

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

# Custom Driving Methods ===========================================================================================
    def gyro_turn_on_center(self, target_angle=90, speed=100, adjustment=1.0):

        target_angle = target_angle * adjustment
        self.gyro_sensor.reset_angle(0)
        angle = self.gyro_sensor.angle() #take an initial reading from sensor
        print("initial " + str(angle)) #TODO Remove when ready
        
        if target_angle < 0:  
            while angle > target_angle:
                self.left_motor.run(speed=(-1 * speed))
                self.right_motor.run(speed=speed)
                wait(10)
                angle = self.gyro_sensor.angle()  # take another reading from sensor
                print(angle)  #TODO Remove when ready
        else:  
            while angle < target_angle:
                self.left_motor.run(speed=speed)
                self.right_motor.run(speed=(-1 * speed))
                wait(10)
                angle = self.gyro_sensor.angle() # take another reading from sensor
                print(angle)  #TODO Remove when ready

        self.left_motor.brake()
        self.right_motor.brake()


    def gyro_straight(self, pid, distance=100, speed=100):
        pass

# Attachment Motor Methods ==================================================================================
    # NOT SURE I NEED THESE - JUST USE THE PYBRICKS COMMANDS IN Robot Specific CHILD
    # WHAT ABOUT ACTUAL KID CODE? That would be further abastracted.  But still possible? 
    # 
    '''
    def turn_large_attachment_motor(self,degrees,speed):
        pass

    def run_angle(self,rotation_angle,speed,wait=True):
        print("desired speed: " + str(speed))
        print("desired rotation angle: " + str(rotation_angle))

        print("motor angle before: " + str(self.medium_attachment_motor.angle()))
        self.medium_attachment_motor.run_angle(speed, rotation_angle, then=Stop.HOLD, wait=wait)
        print("motor angle after: " + str(self.medium_attachment_motor.angle()))
    '''       

# Non-Driving Methods =======================================================================================

    def beep(self, frequency=500, duration=200):
        '''
        Parameters:	
            frequency (frequency: Hz) – Frequency of the beep. Frequencies below 100 are treated as 100.
            duration (time: ms) – Duration of the beep. If the duration is less than 0, then the method 
            returns immediately and the frequency play continues to play indefinitely
        '''
        self.ev3.speaker.beep(frequency=500, duration=100)

    def say(self, phrase = "hello humans!"):
        self.ev3.speaker.say(phrase)

    def wait(self,time):
        wait(time)

    def watch_sensors(self,dummy):
        self.wait(1000)
        self.gyro_sensor.reset_angle(0)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.medium_attachment_motor.reset_angle(0)
        self.large_attachment_motor.reset_angle(0)

        # TODO - Put line and motors together and add ultrasonic sensor
        while (self.ev3.buttons.pressed() == []):
            self.ev3.screen.clear()
            self.ev3.screen.draw_text(1, 1, "Gyro:")
            self.ev3.screen.draw_text(100, 1, self.gyro_sensor.angle())
            self.ev3.screen.draw_text(1, 20, "M Motor:")
            self.ev3.screen.draw_text(100, 20, self.medium_attachment_motor.angle())
            self.ev3.screen.draw_text(1, 40, "L Motor:")
            self.ev3.screen.draw_text(100, 40, self.large_attachment_motor.angle())
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

    def wait_for_button(self, left, center, right):
    
        '''
        Displays a menu screen 
        Waiting for a button to be pressed
        Returns button pressed 
        '''

        self.ev3.screen.clear()

        ls = self.left_line_sensor
        rs = self.right_line_sensor
        us = self.ultrasonic_sensor
        gy = self.gyro_sensor

        l = 0 # left line sensor value
        r = 0 # right line sendor value 
        u = 0 # middle ultrasonic value
        g = 0 # gyro value
        pl = 0 # previous left line sensor value
        pr = 0 # previous right line sensor value
        pu = 0 # previous ultrasonic value
        pg = 0 # previous gyro value

        pressed = []
        while len(pressed) != 1:

            # read values
            l = ls.reflection()
            r = rs.reflection()
            u = us.distance()
            g = gy.angle()

            if (l != pl) or (r != pr) or (u != pu) or (g != pg):
                self.ev3.screen.clear()
                self.ev3.screen.draw_text(1, 1, l)
                self.ev3.screen.draw_text(70, 1, u)
                self.ev3.screen.draw_text(150, 1, r)
                self.ev3.screen.draw_text(75, 20, "Gyro:")
                self.ev3.screen.draw_text(140, 20, g)

            pl = l
            pr = r
            pu = u
            pg = g

            self.ev3.screen.draw_text(1, 20, "UP")
            
            self.ev3.screen.draw_text(1, 40, "LT=" + left)

            self.ev3.screen.draw_text(1, 60, "CT=" + center)

            self.ev3.screen.draw_text(1, 80, "RT=" + right)
            
            self.ev3.screen.draw_text(1, 100, "DOWN ")


            pressed = self.ev3.buttons.pressed()

            wait(100)

        button = pressed[0]

        # Now wait for the button to be released.
        while any(self.ev3.buttons.pressed()):
            pass

        # Return which button was pressed & released
        return button

    def menu_loop(self, menu_list):
 
        menu_length = len(menu_list)

        p = 0  # starting menu position
        lp = 0 # left button position
        cp = 1 # middle button position
        rp = 2 # right button position

        left = menu_list[lp][0]
        center = menu_list[cp][0]
        right = menu_list[rp][0]

        # Infinite loop for now
        while True:

            # Wait for one button to be selected.
            button = self.wait_for_button(left, center, right)

            # Processing different kinds of selections
            # Selected item always moves to left button
            if button == Button.LEFT:
                menu_list[lp][1](menu_list[lp][2])
                # Don't advance the menu

                left = menu_list[lp][0]
                center = menu_list[cp][0]
                right = menu_list[rp][0]

            elif button == Button.CENTER:
                menu_list[cp][1](menu_list[cp][2])
                # Advance the menu by 1
                lp = cp
                cp = rp
                rp = rp + 1
                if rp == menu_length:
                    rp = 0

                left = menu_list[lp][0]
                center = menu_list[cp][0]
                right = menu_list[rp][0]

            elif button == Button.RIGHT:
                menu_list[rp][1](menu_list[cp][2])
                # Advance the menu by 2
                lp = rp
                cp = rp + 1
                rp = rp + 2
                if rp == menu_length:
                    cp = 0
                    rp = 1

                left = menu_list[lp][0]
                center = menu_list[cp][0]
                right = menu_list[rp][0]

            elif button == Button.DOWN:
                print(button)
                # Advance the menu by 1
                lp = cp
                cp = rp
                rp = rp + 1
                if rp == menu_length:
                    rp = 0

                left = menu_list[lp][0]
                center = menu_list[cp][0]
                right = menu_list[rp][0]

            elif button == Button.UP:
                print(button)
                # Decrease the menu by 1
                rp = cp
                cp = lp
                lp = lp - 1
                if lp < 0:
                    lp = menu_length - 1
                
                left = menu_list[lp][0]
                center = menu_list[cp][0]
                right = menu_list[rp][0]
  
