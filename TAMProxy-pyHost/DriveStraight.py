# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 21:48:48 2017

@author: arinz
"""

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Motor,Gyro, Encoder
import math

# Cycles a motor back and forth between -255 and 255 PWM every ~5 seconds
import Settings as S
class DriveStraight(SyncedSketch):
    # Set me!
    left_motor_dir_pin, left_motor_pwm_pin = S.left_motor_dir_pin, S.left_motor_pwm_pin
    right_motor_dir_pin, right_motor_pwm_pin = S.right_motor_dir_pin, S.right_motor_pwm_pin
    #encoder pins
    pinsL = S.left_motor_encoder_pins
    pinsR = S.right_motor_encoder_pins
    #gyroscope pin
    ss_pin = S.ss_pin
    #set the initial motor speed
    init_motor_speed = S.init_motor_speed
    #angle we want to turn to
    desired_theta = S.desired_theta
    #physical constants
    shouldGoAgain = True
    theta_est = S.theta_est
    theta_est_enc = S.theta_est_enc

    last_diff = 0.0
    integral = 0.0
    len_axis = S.axel_length #8.25 distance between two wheels in inches
    wheel_r = S.wheel_radius #1.9375 radius of wheels in inches
    wheel_circumference = S.wheel_circumference #2*wheel_radius*numpy.pi
    counts_per_rotation = S.counts_per_rotation #3200.0 #from the motor ecoder specs
    refresh_rate = S.refresh_rate #50


    kP = S.kP #2#0.6*kU
    kI = S.kI #1e-3#tU/2
    kD = S.kD #110#tU/8.0
    maxCorrection = S.maxCorrection #50
    squareLength = 24 #inches
    numTurns = 0
    

    max_speed = 100
    distance = 0 # inches
    
    
    angle = 0 #desired angle
    error = 0 #stores the previous error
    integ = 0 # stores the previous integral
    
    
    def setup(self):
        self.motorR = Motor(self.tamp, 2, 3)
        self.motorL = Motor(self.tamp, 4, 5)
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
            e = g_angle - angle # positive means right wheel needs more spin            
            deriv = (e - self.error)/dt
            self.integ += self.error * dt
            corr = (self.kP * e) + (self.kD * deriv) + (self.kI * self.integ)            
            self.error = e
            
            r_speed = 50 + corr
            l_speed = 50 - corr
            self.set_speed(r_speed, l_speed)
            
            self.distance =  ((self.encoderL.val + self.encoderR.val) / 6400.0) * math.pi * 2 * self.wheel_r
            
            if (self.distance > 30.0):# drive for 2.5 ft. (30 in.)
                self.set_speed(0, 0)
            self.motorL.write(self.motorvalL < 0, abs(self.motorvalL))
            self.motorR.write(self.motorvalR < 0, abs(self.motorvalR))
            
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



# Prints a quadrature encoder's position
