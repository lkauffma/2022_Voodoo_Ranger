#!/usr/bin/env pybricks-micropython
from pid import PID

def test_stall(robot):
    robot.test_stall(200)

def testing_pid(robot):
    test_pid_1 = PID()
    test_pid_2 = PID(Kp=2.0)
    print("test_pid_1")
    print(test_pid_1.Kp, test_pid_1.Ki, test_pid_1.Kd)

    print("test_pid_2")
    print(test_pid_2.Kp, test_pid_2.Ki, test_pid_2.Kd)

    print("test_pid_1")
    test_pid_1.update_Kp(100.0)
    test_pid_1.update_Ki(20.0)
    test_pid_1.update_Kd(3.00)
    print(test_pid_1.Kp, test_pid_1.Ki, test_pid_1.Kd)

    print("test_pid_2")
    test_pid_2.update_Kp(.1)
    test_pid_2.update_Ki(0.0)
    test_pid_2.update_Kd(0.0)
    print(test_pid_2.Kp, test_pid_2.Ki, test_pid_2.Kd)

    print("test adjusment")
    print(test_pid_2.adjustment(5))
    print(test_pid_2.adjustment(6))
    print(test_pid_2.adjustment(-5))
    print(test_pid_2.adjustment(2))
    print(test_pid_2.adjustment(0))

def test_gyro_turn_on_center(robot):
    robot.wait(1000)
    print("Hello From Test")
    robot.gyro_turn_on_center(target_angle=-360, speed=200)

def test_gyro_straight(robot):
    test_pid_1 = PID()
    robot.gyro_straight(test_pid_1, distance=300, speed=100)