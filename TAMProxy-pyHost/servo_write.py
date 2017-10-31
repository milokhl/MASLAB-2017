from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Servo


class ServoWrite(Sketch):
    """Cycles a servo back and forth between 0 and 180 degrees. However,
    these degrees are not guaranteed accurate, and each servo's range of valid
    microsecond pulses is different"""

    SERVO_PIN = 3
#    SERVO_PIN2 = 3

    def setup(self):
        self.servo = Servo(self.tamp, self.SERVO_PIN)
        self.servo.write(50)
        self.servoval = 0
#
#        self.servo2 = Servo(self.tamp, self.SERVO_PIN2)
#        self.servo2.write(25)
#        self.servoval2 = 0        
        
#        self.delta = 1
        self.timer = Timer()
#        self.end = False

    def loop(self):
        if (self.timer.millis() > 100):
            self.timer.reset()
#            if self.servoval >= 180: self.delta = -1
#            elif self.servoval <= 0: self.delta = 1
#            self.servoval += self.delta
#            print self.servoval
#            self.servo.write(abs(self.servoval))
            
            try:
                self.servoval = int(raw_input("grabber:"))
            except:
                self.servoval=10
            self.servo.write(abs(self.servoval))
            
#            try:
#                self.servoval2 = int(raw_input("elevator:"))
#            except:
#                self.servoval2 = 50
#            self.servo2.write(abs(self.servoval2))
            
if __name__ == "__main__":
    sketch = ServoWrite()
    sketch.run()
