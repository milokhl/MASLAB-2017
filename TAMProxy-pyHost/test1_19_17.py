class testdrive(SyncedSketch):
    pinsL = S.left_motor_encoder_pins
    pinsR = S.right_motor_encoder_pins
    ss_pin = S.ss_pin
    desHeading = 0
    obsHeading = 0 + self.gyro.val
    headingDif = obsHeading - desHeading
    Speed

    def setup(self):
        self.encoderL = Encoder(self.tamp, *self.pinsL, continuous=True)
        self.encoderR = Encoder(self.tamp, *self.pinsR, continuous=True)
        self.gyro = Gyro(self.tamp, self.ss_pin, integrate=True)
        self.rightMotor = Motor(self.tamp, S.right_motor_dir_pin, S.right_motor_pwm_pin)
        self.leftMotor = Motor(self.tamp, S.left_motor_dir_pin, S.left_motor_pwm_pin)
        self.rightMotor.write(1,0)
        self.leftMotor.write(1,0)
        self.motorval = 0
        self.timer = Timer()

    def loop(self):
        if self.timer.millis() > 100:
            self.timer.reset()
            self.leftMotor.write(1,50)
            self.rightMotor.write(1,50)
            print "left encoder: " + str(self.encoderL.val)
            print "right encoder: " + str(self.encoderR.val)
            #print "gyro val, status: " + str(self.gyro.val) + str(self.gyro.status)


if __name__ == "__main__":
    sketch = testdrive(1, -0.00001, 100)
    sketch.run()
