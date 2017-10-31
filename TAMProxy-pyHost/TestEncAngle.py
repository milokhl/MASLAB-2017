# -*- coding: utf-8 -*-
"""
Created on Fri Jan 13 10:54:37 2017

@author: arinz
"""
from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder,Gyro
import Settings as S
    

class GyroRead(SyncedSketch):

    # Set me!
    ss_pin = S.ss_pin
    pinsL = S.left_motor_encoder_pins
    pinsR = S.right_motor_encoder_pins
    axel_length = S.axel_length #distance between two wheels in inches
    wheel_radius = S.wheel_radius #radius of wheels in inches

    def setup(self):
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        self.left_motor_encoder = Encoder(self.tamp, *self.pinsL, continuous=True)
        self.right_motor_encoder = Encoder(self.tamp, *self.pinsR, continuous=True)
        
        self.timer = Timer()
        self.cali_timer = Timer()
        self.calibration = 0.0
        self.calibrated = True

    def loop(self):
        if self.timer.millis() > 100:
            self.timer.reset()
            print "--------"
            print "Gyro:",self.gyro.val
            print "Encoders",float(self.left_motor_encoder.val - self.right_motor_encoder.val)*self.wheel_radius*9.0/(80.0*self.axel_length)
            print "--------"
            # Valid gyro status is [0,1], see datasheet on ST1:ST0 bits
            #print "{:6f}, raw: 0x{:08x} = 0b{:032b}".format(self.gyro.val, self.gyro.raw, self.gyro.raw)
            
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
            
if __name__ == "__main__":
    sketch = GyroRead(1, -0.00001, 100)
    sketch.run()
