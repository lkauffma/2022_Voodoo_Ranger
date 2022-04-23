#!/usr/bin/env pybricks-micropython

# This creates an object to use to calculate the adjustment in PID feedback loops

class PID():
    def __init__(pid = [0,0,0]):
        self.Kp = pid[0]
        self.Ki = pid[1]
        self.Kd = pid[2]
        self.last_error = 0

    def adjustment(self, error = 0):
        self.integral = self.integral + error
        self.derivative = error - self.last_error

        pid_adjustment = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative

        self.last_error = error

        return pid_adjustment





