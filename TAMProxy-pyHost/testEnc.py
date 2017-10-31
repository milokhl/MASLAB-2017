from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Motor,Gyro, Encoder
import math, Settings as S

# Cycles a motor back and forth between -255 and 255 PWM every ~5 seconds

class DriveStraight(SyncedSketch):
    # Set me!
    pinsR = S.right_motor_encoder_pins   
    pinsL = S.left_motor_encoder_pins
    wheel_r = S.wheel_radius # inches
    ss_pin = 10
    
    def setup(self):
        self.motorR = Motor(self.tamp, 23, 22)
        self.motorL = Motor(self.tamp, 21, 20)
        self.motorR.write(1,0)
        self.motorL.write(1,0)
        self.motorvalR = 0
        self.motorvalL = 0
        self.timer = Timer()
        

        self.encoderL = Encoder(self.tamp, *self.pinsL, continuous=True)
        self.encoderR = Encoder(self.tamp, *self.pinsR, continuous=True)
        self.encoderTimer = Timer()
        
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
     
        
    def loop(self):
        if (self.timer.millis()>10):
            self.timer.reset()
            desired = 1e6
            if abs(self.encoderL.val)>=desired:
                self.motorvalL = 0
            else:
                self.motorvalL = 50
            if abs(self.encoderR.val)>=desired:
                self.motorvalR = 0
            else:
                self.motorvalR = 50
            self.motorL.write(self.motorvalL < 0, abs(self.motorvalL))
            self.motorR.write(self.motorvalR < 0, abs(self.motorvalR))
            print "Left Encoder",self.encoderL.val
            print "Right Encoder",self.encoderR.val
            print "Gyro:",self.gyro.val
            

if __name__ == "__main__":
    sketch = DriveStraight(1, -0.00001, 100)
    sketch.run()