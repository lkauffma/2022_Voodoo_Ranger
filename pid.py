#!/usr/bin/env pybricks-micropython

# This creates an object to use to calculate the adjustment in PID feedback loops

class PID():
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0.0
        self.integral = 0.0

    def adjustment(self, error = 0):
        self.integral = self.integral + error
        self.derivative = error - self.last_error

        pid_adjustment = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative

        self.last_error = error

        return pid_adjustment

    def update_Kp(self, Kp):
        self.Kp = Kp

    def update_Ki(self, Ki):
        self.Ki = Ki

    def update_Kd(self, Kd):
        self.Kd = Kd

    def Kp(self):
        return self.Kp

    def Ki(self):
        return self.Ki

    def Kd(self):
        return self.Kd





