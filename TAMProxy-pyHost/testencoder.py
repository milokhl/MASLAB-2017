from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder
from tamproxy.devices import Gyro
from tamproxy.devices import Motor
import numpy, math, Settings as S

class DriveStraight(SyncedSketch):
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
    wheel_radius = S.wheel_radius #1.9375 radius of wheels in inches
    wheel_circumference = S.wheel_circumference #2*wheel_radius*numpy.pi
    counts_per_rotation = S.counts_per_rotation #3200.0 #from the motor ecoder specs
    refresh_rate = S.refresh_rate #50

    def setup(self):
        self.timer = Timer()
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
        self.timerdelay = Timer()
        while self.timerdelay.millis() < 1000:
            pass
        #start motors


    def loop(self):
        if self.timer.millis() > self.refresh_rate:
            self.dt = self.timer.millis()
            self.timer.reset()
            print "L_encoder,r_encoder:", self.left_motor_encoder.val, self.right_motor_encoder.val
            print"L_Distance:",self.left_motor_encoder.val*math.pi*self.wheel_radius/3200.0
            print "R_Distance:",self.right_motor_encoder.val*math.pi*self.wheel_radius/3200.0
            distance = (self.left_motor_encoder.val+self.right_motor_encoder.val)*math.pi*self.wheel_radius/3200.0
            print "Distance:",distance
            desiredEnc = 3200
            try:
                L = (desiredEnc-self.left_motor_encoder.val)/abs((desiredEnc-self.left_motor_encoder.val))
                R = (desiredEnc-self.right_motor_encoder.val)/abs((desiredEnc-self.right_motor_encoder.val))
            except ZeroDivisionError:
                L,R=0,0
            self.motorval_left = max(42,255*abs(desiredEnc-self.left_motor_encoder.val)/3200)
            self.motorval_right = max(42,255*abs(desiredEnc-self.right_motor_encoder.val)/3200)
            self.motorval_left = -self.motorval_left*L
            self.motorval_right = -self.motorval_right*R
            self.left_motor.write(self.motorval_left>0, abs(self.motorval_left))
            self.right_motor.write(self.motorval_right>0, abs(self.motorval_right))
            # if abs(self.left_motor_encoder.val) > 3200:
            #     self.left_motor.write(1,0)
            # else:
            #     self.left_motor.write(self.motorval_left>0, abs(self.motorval_left))
            # if abs(self.right_motor_encoder.val) > 3200:
            #     self.right_motor.write(1,0)
            # else:
            #     self.right_motor.write(self.motorval_right>0, abs(self.motorval_right))



if __name__ == "__main__":
    sketch = DriveStraight(1, -0.00001, 100)
    sketch.run()
