# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 20:54:02 2017

@author: arinz
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 16:10:53 2017

@author: arinz
"""

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Servo


class ServoWrite(Sketch):
    """Cycles a servo back and forth between 0 and 180 degrees. However,
    these degrees are not guaranteed accurate, and each servo's range of valid
    microsecond pulses is different"""

    SERVO1_PIN = 2
    SERVO2_PIN = 3

    def setup(self):
        self.servo1 = Servo(self.tamp, self.SERVO1_PIN)
        self.servo1.write(0)
        self.servo2 = Servo(self.tamp, self.SERVO2_PIN)
        self.servo2.write(0)
        self.queue = [20,20,120,120]
        self.servoQueue = [self.servo1,self.servo2]
        self.queueTimes =[1000,1000,1000,1000,1000]

        self.timer = Timer()
        self.servoTimer = Timer()
        self.index = 0

    def loop(self):
        if (self.timer.millis() > 10):
            self.timer.reset()
            
            if self.servoTimer.millis()>= self.queueTimes[self.index%len(self.queueTimes)]:
                self.servoTimer.reset()
                self.servoQueue[self.index%len(self.servoQueue)].write(self.queue[self.index%len(self.queue)])
                print "timer value",self.queueTimes[self.index%len(self.queueTimes)]
                print "index",self.index
                print "servo number",(self.index%len(self.servoQueue)+1)
                print "value to write",self.queue[self.index%len(self.queue)]
                self.index+=1

#            if self.servoval >= 180: self.delta = -1
#            elif self.servoval <= 0: self.delta = 1
#            self.servoval += self.delta
#            print self.servoval
#            self.servo.write(abs(self.servoval))

if __name__ == "__main__":
    sketch = ServoWrite()
    sketch.run()