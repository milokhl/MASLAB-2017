
from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Motor
import Settings as S

# Cycles a motor back and forth between -255 and 255 PWM every ~5 seconds

class MotorWrite(Sketch):
#    RIGHTDIR, RIGHTPWM = 2,3
#    LEFTDIR , LEFTPWM  = 4,5
    RIGHTDIR, RIGHTPWM = S.right_motor_dir_pin,S.right_motor_pwm_pin
    LEFTDIR , LEFTPWM  = S.left_motor_dir_pin,S.left_motor_pwm_pin
    def setup(self):
        self.motorR = Motor(self.tamp, self.RIGHTDIR, self.RIGHTPWM)
        self.motorR.write(1,0)
        self.motorvalR = 0
        
        self.motorL = Motor(self.tamp, self.LEFTDIR, self.LEFTPWM)
        self.motorL.write(1,0)
        self.motorvalL = 0
        
        self.delta = 1
        self.timer = Timer()

    def loop(self):
        if (self.timer.millis() > 10):
            self.timer.reset()
            if abs(self.motorvalR) == 100: self.delta = -self.delta
            
            self.motorvalR += self.delta
            self.motorR.write(self.motorvalR>0, abs(self.motorvalR))
            
            self.motorvalL += self.delta
            self.motorL.write(self.motorvalL>0, abs(self.motorvalL))
            print self.motorvalL,self.motorvalR
if __name__ == "__main__":
    sketch = MotorWrite()
    sketch.run()