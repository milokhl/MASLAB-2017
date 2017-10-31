# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 22:13:12 2017

@author: arinze, jasmine
"""

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder
from tamproxy.devices import Gyro
from tamproxy.devices import Motor
import numpy, Settings as S



class DriveSquare(SyncedSketch):
    #motor pins
    left_motor_dir_pin, left_motor_pwm_pin = S.left_motor_dir_pin, S.left_motor_pwm_pin
    right_motor_dir_pin, right_motor_pwm_pin = S.right_motor_dir_pin, S.right_motor_pwm_pin
    #encoder pins
    left_motor_encoder_pins = S.left_motor_encoder_pins
    right_motor_encoder_pins = S.right_motor_encoder_pins
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
    axel_length = S.axel_length #8.25 distance between two wheels in inches
    wheel_radius = S.wheel_radius #1.9375 radius of wheels in inches
    wheel_circumference = S.wheel_circumference #2*wheel_radius*numpy.pi
    counts_per_rotation = S.counts_per_rotation #3200.0 #from the motor ecoder specs
    refresh_rate = S.refresh_rate #50


    kP = S.kP #2#0.6*kU
    kI = S.kI #1e-3#tU/2
    kD = S.kD #110#tU/8.0
    maxCorrection = S.maxCorrection #50
    squareLength = 24 #inches
    numTurns = 0
    bias = 0
    
    def setup(self):
        print "here1"
        #set up timer
        self.timer = Timer()
        self.timer2 = Timer()
        #set up motors
        self.left_motor = Motor(self.tamp, self.left_motor_dir_pin, self.left_motor_pwm_pin)
        self.right_motor = Motor(self.tamp, self.right_motor_dir_pin, self.right_motor_pwm_pin)
        print "motor objects made"

        self.motorval_left = self.init_motor_speed
        self.motorval_right = self.init_motor_speed
        
        #set up encoders
        self.left_motor_encoder = Encoder(self.tamp, *self.left_motor_encoder_pins, continuous=True)
        self.right_motor_encoder = Encoder(self.tamp, *self.right_motor_encoder_pins, continuous=True)
        print "encoder objects made"

        #set up the gyro
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        self.cali_timer = Timer()
        self.calibration = 0.0
        self.calibrated = False

        print "gyro object made"
        
        self.theta_gyro_old = self.gyro.val
        
        #start motors
        self.right_motor.write(1,0)
        self.left_motor.write(1,0)
        print "Finished Setup"


    def loop(self):
        if self.timer.millis() > self.refresh_rate:
            self.dt = self.timer.millis()/1000.0
            self.timer.reset()
            if self.calibrated:
                self.bias = self.init_motor_speed
                distance = (self.left_motor_encoder.val + self.right_motor_encoder.val) / 6400.0 * self.wheel_circumference
                if distance >= self.squareLength * (self.numTurns + 1):
                    self.numTurns +=1
                    self.desired_theta += 90

                if self.numTurns >=4:
                    self.bias = 0
                else:
                    self.bias = self.init_motor_speed

                self.updateThetaEstNoEncoder()
                self.updateThetaEstEncoder()
                self.calculatePower()
                self.updateMotors()
            else:
                self.left_motor.write(1,0)
                self.right_motor.write(1,0)

        # Janky autocalibration scheme
        if not self.calibrated and self.cali_timer.millis() > 3000:
            drift = self.gyro.val / (self.cali_timer.millis() / 1000.0)
            # Arbitrary calibration tolerance of 0.1 deg/sec
            if abs(drift) > 0.1:
                self.calibration += drift
                print "Calibration:", self.calibration
                self.gyro.calibrate_bias(self.calibration)
            else:
                print "Calibration complete:", self.calibration
                self.calibrated = True
            self.gyro.reset_integration()
            self.cali_timer.reset()

        if self.timer2.millis() > 500:
            self.timer2.reset()
            print "desired theta: " + str(self.desired_theta)
            print "actual theta: " + str(self.theta_est)
#            print "difference " + str(self.last_diff)
            print "bias:",self.bias
            print "gyro:",self.theta_est
            print "encoder:",self.theta_est_enc
            print "distance:",(self.left_motor_encoder.val + self.right_motor_encoder.val) / 6400.0 * self.wheel_circumference


    def updateThetaEstNoEncoder(self):
        #Only use gyroscope to estimate current angle
        self.theta_est = self.gyro.val

    def updateThetaEstEncoder(self):

        self.theta_est_enc = float(self.left_motor_encoder.val - self.right_motor_encoder.val)*self.wheel_radius*9.0/(80.0*self.axel_length)


    def calculatePower(self):
        #Use PID and update motors
        diff = self.desired_theta - self.theta_est
        self.integral = self.integral + diff*self.dt
        self.derivative = (diff - self.last_diff)/self.dt
        self.power = self.kP*diff + self.kI*self.integral + self.kD*self.derivative
        self.last_diff = diff

    def updateMotors(self):
        #Prevent a power that will try to set the wheels above or below possible values
        if ((self.bias + self.power)>255) or ((self.bias - self.power)<-255):
            self.power = 255 - abs(self.bias)
        elif ((self.bias + self.power)<-255) or ((self.bias - self.power)>255):
            self.power = -255 + abs(self.bias)

        #Stop power from being too high so the change in wheel speed is not too drastic
        if self.power>self.maxCorrection:
            self.power = self.maxCorrection
        elif self.power<-self.maxCorrection:
            self.power = -self.maxCorrection

        if abs(self.last_diff) >= 10:
            self.bias = 0
        #Set motor values
        self.motorval_left = self.bias + self.power
        self.motorval_right = self.bias - self.power
        self.left_motor.write(self.motorval_left>0, abs(self.motorval_left))
        self.right_motor.write(self.motorval_right>0, abs(self.motorval_right))

if __name__ == "__main__":
    sketch = DriveSquare(1, -0.00001, 100)
    sketch.run()
