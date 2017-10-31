# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 22:13:12 2017

@author: jasmine, arinze
"""

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder,Gyro,Motor
import numpy as np, Settings as S
import math




class DriveToCoord(SyncedSketch):
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
    heading = np.radians(theta_est)
    theta_est_enc = S.theta_est_enc

    axel_length = S.axel_length #8.25 distance between two wheels in inches
    wheel_radius = S.wheel_radius #1.9375 radius of wheels in inches
    wheel_circumference = S.wheel_circumference #2*wheel_radius*numpy.pi
    counts_per_rotation = S.counts_per_rotation #3200.0 #from the motor ecoder specs
    refresh_rate = S.refresh_rate #50
    

    kP = S.kP #2#0.6*kU
    kI = S.kI #1e-3#tU/2
    kD = S.kD #110#tU/8.0
    maxCorrection = 70#S.maxCorrection #50
    last_diff = 0.0
    integral = 0.0
    derivative= 0.0
    power= 0.0

    distTravelled = 0 #inches
    desired_dist = 0 #inches
    oldEncLeft = 0
    oldEncRight = 0
    x = 0.0
    y = 0.0
    
    angle = None
    
    
    dTheta = 10

    driveToCoordState = 0
    coordinates = [[84, 0, None], [132, 72, None],[132, -60, None], [96,24,None], [0,0,0]]
#    coordinates = [[48, 0, None], [0, 0,0]]
    coord_ind = 0
    
    
    def setup(self):
        print "Starting Setup"
        #set up timer
        self.timer = Timer()
        self.statusTimer = Timer()
        self.roundTimer = Timer()
        self.cali_timer = Timer()
        print "timers made"
        
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
        
        #make gyro
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        self.calibration = 0.0
        self.calibrated = False
        print "gyro object made"

        #start motors at 0
        self.right_motor.write(1,0)
        self.left_motor.write(1,0)

        self.bias = 0
        self.lastBlockReading = (True, 0, float("inf"))
        self.lastEncoderReading = 0
        self.distanceTraveled = 0
        print "Finished Setup"

        

    def loop(self):
        if self.timer.millis() > self.refresh_rate:
            self.dt = self.timer.millis() /1000.0
            self.timer.reset()
           
            if self.calibrated:
                try:
                   # print "here"
                    self.drive_to_coord(self.coordinates[self.coord_ind][0],self.coordinates[self.coord_ind][1], self.coordinates[self.coord_ind][2])
                except IndexError:
                    
                    self.left_motor.write(1,0)
                    self.right_motor.write(1,0)
                    self.bias = 0
                   # self.desired_theta = self.theta_est

#                print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
                self.deadReckon()
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
            
        if self.statusTimer.millis()>=1000:
            self.statusTimer.reset()
            print "gyro:",self.theta_est
            print "encoder angle:",self.theta_est_enc
            print "desired_theta:",self.desired_theta
            print "bias:",self.bias
            print "(x,y,heading)",(self.x,self.y,math.degrees(self.heading))
            print "Left Encoder:",self.left_motor_encoder.val,"Right Encoder:",self.right_motor_encoder.val
            print "driveToCoordState",self.driveToCoordState
            print "Destination:",self.coordinates[min(self.coord_ind,len(self.coordinates)-1)]
            print "coord_ind",self.coord_ind
            print "Proportional:",self.kP*self.last_diff
            print "Integral:",self.kI*self.integral
            print "Derivative:",self.kD*self.derivative
            print "Overall:", self.power
            print "------------------------------------"
        

    def updateThetaEstNoEncoder(self):
        #Only use gyroscope to estimate current angle
        self.theta_est = self.gyro.val

    def updateThetaEstEncoder(self):

        self.theta_est_enc = float(self.left_motor_encoder.val - self.right_motor_encoder.val)*self.wheel_radius*9.0/(80.0*self.axel_length)
#        self.theta_est = float(self.left_motor_encoder.val - self.right_motor_encoder.val)*self.wheel_radius*9.0/(80.0*self.axel_length)

    def calculatePower(self):
        #Use PID and update motors
        diff = self.desired_theta - self.theta_est
        integ = self.integral + diff*self.dt
        if integ < 0:
            self.integral = max(integ, - self.maxCorrection/self.kI)
        else:
            self.integral = min(integ, self.maxCorrection/self.kI)
            
        deriv = (diff - self.last_diff)/self.dt
        if deriv < 0:
            self.derivative = max(deriv, - self.maxCorrection/self.kD)
        else:
            self.derivative = min(deriv, self.maxCorrection/self.kD)
        
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
        self.motorval_left = self.bias + self.power + 15.0
        self.motorval_right = self.bias - self.power - 15.0
        self.left_motor.write(self.motorval_left>0, abs(self.motorval_left))
        self.right_motor.write(self.motorval_right>0, abs(self.motorval_right))


    def deadReckon(self):
            leftEnc = self.left_motor_encoder.val
            rightEnc = self.right_motor_encoder.val
            deltaL = (leftEnc - self.oldEncLeft) * self.wheel_circumference/3200.0
            deltaR = (rightEnc - self.oldEncRight) * self.wheel_circumference/3200.0
            if (abs(deltaL - deltaR)) <= 1e-6:
                dx = deltaL * np.cos(self.heading)
                dy = deltaR * np.sin(self.heading)
                self.heading = np.radians(-self.gyro.val)

            else:
                r = self.axel_length * (deltaL + deltaR) / (2 * (deltaR - deltaL))
                wd = (deltaR - deltaL) / self.axel_length
                
                dx = r* np.sin(self.heading + wd) - r*np.sin(self.heading)
                dy = -r* np.cos(self.heading + wd) + r*np.cos(self.heading)
#                self.heading += wd
                self.heading = np.radians(-self.gyro.val)

            self.x = self.x +  dx
            self.y = self.y +  dy
            self.distTravelled += np.sqrt(dy**2 + dx**2)
            self.desired_dist -= np.sqrt(dy**2 + dx**2)
            self.oldEncLeft = leftEnc
            self.oldEncRight = rightEnc
    
                    
    def get_coord_angle(self, x, y):
        ang_rad = math.atan2(y- self.y,x- self.x)
        ang_deg = math.degrees(ang_rad)
        
        
        return -ang_deg
        

    def get_coord_dist(self, x, y):
        dist =  np.sqrt((self.x - x)**2 + (self.y- y)**2)
        return dist

    def drive_to_coord(self, x, y, theta):
        if self.driveToCoordState == 0:
            self.desired_theta = self.get_coord_angle(x, y)
            if abs(self.theta_est - self.desired_theta) < 5:
                self.bias = self.init_motor_speed
            else:
                self.bias = 0
            if self.get_coord_dist(x, y) <= 2:
                self.driveToCoordState = 1
        elif self.driveToCoordState == 1:
            self.bias = 0
            if theta == None:
                self.desired_theta = self.theta_est
            else:
                self.desired_theta = - theta
            if abs(self.theta_est - self.desired_theta) < 5:
                self.coord_ind += 1
                print ">>>>>>>>>>>>>>>>>>>move on"
                self.driveToCoordState = 0
    

if __name__ == "__main__":
    
    sketch = DriveToCoord(1, -0.00001, 100)
    
    print "Sketch Made"
    sketch.run()

    
