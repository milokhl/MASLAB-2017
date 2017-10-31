from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Encoder
import math,Settings as S

# Prints a quadrature encoder's position
class EncoderRead(SyncedSketch):
#    pins = 6,7
#    pinsR = 6, 7
#    pinsL = 8, 9
    pinsR = S.right_motor_encoder_pins
    pinsL = S.left_motor_encoder_pins
    WHEELRAD = 3.875/2 #inches
    def setup(self):
        self.encoderR = Encoder(self.tamp, *self.pinsR, continuous=True)
        self.encoderL = Encoder(self.tamp, *self.pinsL, continuous=True)
        self.timer = Timer()

    def loop(self):
        if self.timer.millis() > 100:
            self.timer.reset()
            print "Right Encoder Val:",self.encoderR.val
            print "Right Wheel Distance:",(self.encoderR.val/3200.0)*2*math.pi*self.WHEELRAD
            print "Left Encoder Val:",self.encoderL.val
            print "Left Wheel Distance:",(self.encoderL.val/3200.0)*2*math.pi*self.WHEELRAD
            if self.encoderL.val > 3200:
                self.encoderL.val = 0
if __name__ == "__main__":
    sketch = EncoderRead(1, -0.00001, 100)
    sketch.run()
