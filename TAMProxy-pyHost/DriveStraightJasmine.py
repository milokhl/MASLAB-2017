# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 21:48:48 2017

@author: arinz
"""

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Motor,Gyro, Encoder
import math, Settings as S

# Cycles a motor back and forth between -255 and 255 PWM every ~5 seconds

class DriveStraight(SyncedSketch):
    # Set me!
    ss_pin = S.ss_pin
    max_speed = 100
    pinsR = S.right_motor_encoder_pins  # pins for right encoders   
    pinsL = S.left_motor_encoder_pins # pins for left encoders
    wheel_r = S.wheel_radius # inches
    len_axis = S.axel_length # inches
    distance = 0 # inches
    
    # old constants
    kP = S.kP#0.6*kU 
    kI = S.kI#tU/2
    kD = S.kD#tU/8.0
    
    angle = 0 #desired angle
    error = 0 #stores the previous error
    integ = 0 # stores the previous integral
    
    
    def setup(self):
        self.motorR = Motor(self.tamp, 23, 22)
        self.motorL = Motor(self.tamp, 21, 20)
        self.motorR.write(1,0)
        self.motorL.write(1,0)
        self.delta = 1
        self.motorvalR = 0
        self.motorvalL = 0
        self.timer = Timer()
        
        
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        self.gyroTimer = Timer()
        
        self.encoderL = Encoder(self.tamp, *self.pinsL, continuous=True)
        self.encoderR = Encoder(self.tamp, *self.pinsR, continuous=True)
        self.encoderTimer = Timer()
    
    def set_speed(self,speedR, speedL):
        self.motorvalR = speedR
        self.motorvalL = speedL        
        
    def loop(self):
        if (self.timer.millis()>10):
            dt = self.timer.millis()
            self.timer.reset()
            g_angle = self.gyro.val
            e = g_angle - self.angle # positive means right wheel needs more spin            
            deriv = (e - self.error)/dt
            self.integ += self.error * dt
            corr = (self.kP * e) + (self.kD * deriv) + (self.kI * self.integ)            
            self.error = e
            
            r_speed = 50 + corr
            l_speed = 50 - corr
            self.set_speed(r_speed, l_speed)
            
            self.distance =  ((self.encoderL.val + self.encoderR.val) / 6400.0) * math.pi * 2 * self.wheel_r
            
#            if (self.distance > 30.0):# drive for 2.5 ft. (30 in.)
#                self.set_speed(0, 0)
            self.motorL.write(self.motorvalL > 0, abs(self.motorvalL))
            self.motorR.write(self.motorvalR > 0, abs(self.motorvalR))
            
        if (self.gyroTimer.millis()>1000):
            self.gyroTimer.reset()
            print self.gyro.val
            print "gyro angle: " + str(self.gyro.val)
            
        if (self.encoderTimer.millis() > 1000):
            self.encoderTimer.reset()
            print "right encoder value: " + str((self.encoderR.val/ 3200.0) * 360.0)
            print "left encoder value: " + str((self.encoderL.val/ 3200.0) * 360.0)
            print "distance:",self.distance

if __name__ == "__main__":
    sketch = DriveStraight(1, -0.00001, 100)
    sketch.run()
