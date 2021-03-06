from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder
from tamproxy.devices import Gyro
from tamproxy.devices import Motor
import numpy, Settings as S
import matplotlib.pyplot as plt

#Drives the robot in a straight line

##class MotorWrite(Sketch):
##
##    def setup(self):
##        self.rightMotor = Motor(self.tamp, 2, 3)
##        self.leftMotor = Motor(self.tamp, 4, 5)
##        self.rightMotor.write(1,0)
##        self.leftMotor.write(1,0)
##        self.delta = 1
##        self.motorval = 0
##        self.timer = Timer()
##
##    def loop(self):
##        if (self.timer.millis() > 10):
##            self.timer.reset()
##            if abs(self.motorval) == 255:
##                self.delta = -self.delta
##            self.motorval += self.delta
##            self.leftMotor.write(self.motorval>0, abs(self.motorval))
##            self.rightMotor.write(self.motorval>0, abs(self.motorval))
##
##
##if __name__ == "__main__":
##    sketch = MotorWrite()
##    sketch.run()
##
##class EncoderRead(SyncedSketch):
##
##    pins = 5, 6
##
##    def setup(self):
##        self.encoder = Encoder(self.tamp, *self.pins, continuous=True)
##        self.timer = Timer()
##
##    def loop(self):
##        if self.timer.millis() > 100:
##            self.timer.reset()
##            print self.encoder.val
##
##if __name__ == "__main__":
##    sketch = EncoderRead(1, -0.00001, 100)
##    sketch.run()
##
##class GyroRead(SyncedSketch):
##
##    # Set me!
##    ss_pin = 10
##
##    def setup(self):
##        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
##        self.timer = Timer()
##
##    def loop(self):
##        if self.timer.millis() > 100:
##            self.timer.reset()
##            # Valid gyro status is [0,1], see datasheet on ST1:ST0 bits
##            print self.gyro.val, self.gyro.status
##            
##if __name__ == "__main__":
##    sketch = GyroRead(1, -0.00001, 100)
##    sketch.run()

class DriveStraight(SyncedSketch):
     #angle we want to turn to
    desired_theta = 0
    #physical constants
    shouldGoAgain = True
    last_diff = 0.0
    integral = 0.0
 
    
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
    allAnges = []
    allTimes = []
    shouldPrint = True
    
    def setup(self):
        print "here1"
        #set up timer
        self.timer = Timer()
        self.statusTimer = Timer()
        self.overallTimer = Timer()
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
        print "gyro object made"
        self.theta_gyro_old = self.gyro.val
        #start motors
        self.right_motor.write(1,0)
        self.left_motor.write(1,0)
        print "Set up complete"
        

    def loop(self):
        if self.overallTimer.millis()<=(5000):
            if self.timer.millis() > self.refresh_rate:
                self.dt = self.timer.millis()/1000.0
                self.timer.reset()
                self.bias = self.init_motor_speed
                self.updateThetaEstNoEncoder()
                self.updateThetaEstEncoder()
                self.calculatePower()
                self.updateMotors()
            if self.statusTimer.millis()>=200:
                self.statusTimer.reset()
                print "gyro:",self.theta_est
                print "encoder angle:",self.theta_est_enc
                print "distance:",(self.left_motor_encoder.val + self.right_motor_encoder.val) / 6400.0 * self.wheel_circumference
                print "Left Encoder:",self.left_motor_encoder.val,"Right Encoder:",self.right_motor_encoder.val
                print "Proportional:",self.kP*self.last_diff
                print "Integral:",self.kI*self.integral
                print "Derivative:",self.kD*self.derivative
                print "Overall:", self.power
                self.allAnges.append(self.theta_est)
                self.allTimes.append(self.overallTimer.millis()/1000.0)
                print "------------------------------------"
        else:
            self.left_motor.write(1,0)
            self.right_motor.write(1,0)
            if self.shouldPrint:
                print self.allAnges
                print self.allTimes
                self.shouldPrint = False
                
                plt.plot(self.allTimes, self.allAnges, 'r-o')
#                plt.axis([0, 6, 0, 20])
                plt.show()
    
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
            
        #Set motor values
        self.motorval_left = self.bias + self.power# + 15
        self.motorval_right = self.bias - self.power# - 15
        self.left_motor.write(self.motorval_left>0, abs(self.motorval_left))
        self.right_motor.write(self.motorval_right>0, abs(self.motorval_right))
        
        
if __name__ == "__main__":
    sketch = DriveStraight(1, -0.00001, 100)
    sketch.run()
