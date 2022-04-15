def eastern_adventure(robot):
    # Helicopter via line follower

    # pick-up the erickson wand
    robot.erickson(speed=400, degrees=-250)

    # Drive out to pick-up the line
    robot.motor_straight(distance=160)

    # Drive as fast as you can down to the helicopter
    # 170 is fastest so far with P=1.0 an D=6.0
    # 180 works but not reliable with P=1.0 and D=6.0
    ## robot.pid_line_follow(distance=1750, speed = 180, right_or_left_sensor = "right", side_of_line = "left", Proportional_Gain = 1.0, Integral_Gain = 0.000, Derivative_Gain = 6.0)

    # Drive to bridge and put it down
    robot.pid_line_follow(distance=730, speed = 180, right_or_left_sensor = "right", side_of_line = "left", Proportional_Gain = 1.0, Integral_Gain = 0.000, Derivative_Gain = 6.0)
    robot.erickson(speed=400, degrees=160)
    robot.motor_straight(distance=160)
    robot.erickson(speed=400, degrees=-160)
    robot.motor_straight(distance=160)
    robot.erickson(speed=400, degrees=160)
    robot.motor_straight(distance=-100)
    robot.erickson(speed=400, degrees=-260)

    # Catch the line again and go hit helicopter
    robot.motor_turn(10)
    robot.pid_line_follow(distance=800, speed = 120, right_or_left_sensor = "right", side_of_line = "left", Proportional_Gain = 1.0, Integral_Gain = 0.000, Derivative_Gain = 6.0)

    # Back up, spin around and catch line with left sensor
    robot.motor_straight(distance=-100)
    robot.motor_turn(90)
    robot.motor_straight(distance=100)
    # Drive to track and pull it down
    robot.pid_line_follow(distance=250, speed = 100, right_or_left_sensor = "left", side_of_line = "left", Proportional_Gain = 1.0, Integral_Gain = 0.000, Derivative_Gain = 6.0)
    robot.erickson(speed=400, degrees=-360)
    robot.motor_turn(15)
    robot.motor_turn(-15)
    robot.erickson(speed=400, degrees=360)

    #back up to first car of train and push fwd
    robot.motor_straight(distance=-150)
    robot.erickson(speed=400, degrees=-360)
    robot.pid_line_follow(distance=350, speed = 100, right_or_left_sensor = "left", side_of_line = "left", Proportional_Gain = 1.0, Integral_Gain = 0.000, Derivative_Gain = 6.0)
    
    #back up to end of train and push fwd again
    robot.erickson(speed=400, degrees=360)
    robot.motor_straight(distance=-300)
    robot.erickson(speed=400, degrees=-360)
    robot.motor_straight(distance=230)



    ##robot.motor_straight(distance=-200, speed=50, acceleration=1000)
    ##robot.motor_turn(angle=45, speed=80, acceleration=1000)

def dead_reckon_helicopter(robot):
    robot.motor_straight(distance=650, speed=500, acceleration=1000)
    robot.motor_turn(angle=45, speed=80, acceleration=1000)
    robot.motor_straight(distance=1000, speed=500, acceleration=1000)
    robot.motor_turn(angle=-65, speed=80, acceleration=1000)
    robot.motor_straight(distance=300, speed=500, acceleration=1000)

def blade(robot):
    
    robot.drive(distance = 50, speed = 500, turn_rate = 90)
    robot.wait(750)
    robot.drive(distance = 380, speed = 500, turn_rate = 10)
    robot.drive(distance = 500, speed = -500, turn_rate = 0)

def test(robot):
    robot.test_rear_attachment1()
    robot.medium_attachment_motor.run_angle(1000, 601.2, then=robot.Stop.HOLD, wait=True)