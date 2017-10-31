from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Color
import Settings as S

# Prints RGB, clear(C), colorTemp, and lux values read and
# computed from the device. For more details, see the Adafruit_TCS34725
# Arduino library, from which the colorTemp and lux computations are
# used.

# Color sensor should be connected to the I2C ports (SDA and SCL)

class ColorRead(SyncedSketch):


    colorSenserg = S.colorSenserg # red to green ratio
    colorSenserb = S.colorSenserb
    colorSensegr = S.colorSensegr
    colorSensegb = S.colorSensegb
    ourColor = S.ourColor

    def setup(self):
        self.color = Color(self.tamp,
                           integrationTime=Color.INTEGRATION_TIME_101MS,
                           gain=Color.GAIN_1X)
        self.timer = Timer()

    def loop(self):
        if self.timer.millis() > 100:
            self.timer.reset()
            print self.color.r, self.color.g, self.color.b, self.color.c
            print self.color.colorTemp, self.color.lux
            self.checkColor()
    
    def checkColor(self):
        try:
            print "rg:",float(self.color.r)/self.color.g,"rb:",float(self.color.r)/self.color.b
            print "gr:",float(self.color.g)/self.color.r,"gb:",float(self.color.g)/self.color.b
        except ZeroDivisionError:
            pass
        if self.color.r>100 and (self.color.r>self.colorSenserg*self.color.g) and (self.color.r>self.colorSenserb*self.color.b):
            blockColor = "red"
            print "red block"
        elif self.color.g>100 and (self.color.g>self.colorSensegr*self.color.r) and (self.color.g>self.colorSensegb*self.color.b):
            blockColor = "green"
            print "green block"
        else:
            blockColor = "None"
            print "no block"
            
        if blockColor=="None":
            return -1
        elif blockColor == self.ourColor:
            return 0
        else:
            return 1

if __name__ == "__main__":
    sketch = ColorRead(1, -0.00001, 100)
    sketch.run()
