from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Servo
import Settings as S


class ServoWrite(SyncedSketch):
    """Cycles a servo back and forth between 0 and 180 degrees. However,
    these degrees are not guaranteed accurate, and each servo's range of valid
    microsecond pulses is different"""

    GRABBER_PIN = S.GRABBER_PIN
    grabber_angles = S.grabber_angles #in and out
    
    ELE_PIN = S.ELE_PIN # TBD
    ele_angles = S.ele_angles # up and down
    
    SKEW_PIN = S.SKEW_PIN
    skew_angles = S.skew_angles #in and out
    
    DOOR_PIN = S.DOOR_PIN
    door_angles = S.door_angles  #closed and open
    
    blockPUS = S.blockPUS #-1 is no block, 0 is our block, 1 is opponent block
    ourBlockPUS = S.ourBlockPUS
    oppBlockPUS = S.oppBlockPUS
    dropState = S.dropState
    
    servoDelayOurs = S.servoDelayOurs #millisec
    servoDelayOpp = S.servoDelayOpp #millisec
    servoDelayDropStack = 1000 #millisec
    
    refresh_rate = S.refresh_rate #S.refresh_rate #50
    
    
    def setup(self):
        self.grabber = Servo(self.tamp, self.GRABBER_PIN)
        self.grabber.write(self.grabber_angles[0]) #grabber starts in

        self.elevator = Servo(self.tamp, self.ELE_PIN)
        self.elevator.write(self.ele_angles[0]) #elevator starts up
        
        self.skewer = Servo(self.tamp, self.SKEW_PIN)
        self.skewer.write(self.skew_angles[1]) #skewer starts out
        self.angleToWrite = self.skew_angles[0]
        
        self.door = Servo(self.tamp, self.DOOR_PIN)
        self.door.write(self.door_angles[0]) #door starts closed
        
        self.timer = Timer()
        self.collectTimer = Timer()
        self.statusTimer = Timer()

    def loop(self):
        if self.timer.millis() > self.refresh_rate:
            self.timer.reset()
            
            #Choose an action if one isn't selected
            if self.blockPUS==-1:
#                print "overall state 0"
                try:
                    self.blockPUS = int(raw_input("Select an action:\n0.Collect our blocks\n1.Collect opponent blocks\n2.Drop our stack\n3.Close door\nYour choice: "))
                    if self.blockPUS>3 or self.blockPUS<-1:
                        print "invalid selection"
                        print "+++++++++++++++++++"
                        self.blockPUS = -1
                    
                except:
                    print "invalid selection"
                    self.blockPUS = -1
                
            if self.blockPUS==0:
                self.pickUpOurs()
                
            elif self.blockPUS==1:
                self.pickUpOpponent()
            
            elif self.blockPUS==2:
                self.dropStack()
            elif self.blockPUS==3:
                self.door.write(self.door_angles[0]) #door starts closed
                self.blockPUS = -1
        
#        #Print information
#        if self.statusTimer.millis()>=500:
#            self.statusTimer.reset()
#            print "state of picking up blocks:",self.blockPUS
#            print "state of picking up our blocks:",self.ourBlockPUS
#            print "state of picking up opponent blocks:",self.oppBlockPUS
#            print "state of dropping stacks",self.dropState
#            print "------------------------------------"

    def pickUpOurs(self):
        if self.collectTimer.millis()>=self.servoDelayOurs:
            self.collectTimer.reset()
            if self.ourBlockPUS==0:
                print "taking grabber out"
                self.grabber.write(self.grabber_angles[1]) #take grabber out
                self.ourBlockPUS=1
                
            elif self.ourBlockPUS==1:
                print "lowering elevator"
                self.elevator.write(self.ele_angles[1]) #drop elevator
                self.ourBlockPUS=2
                
            elif self.ourBlockPUS==2:
                print "inserting grabber"
                self.grabber.write(self.grabber_angles[0]) #put grabber in
                self.ourBlockPUS=3
            
            elif self.ourBlockPUS==3:
                print "raising elevator"
                self.elevator.write(self.ele_angles[0]) #raise elevator
                self.ourBlockPUS=0
                self.blockPUS=-1
    
    def pickUpOpponent(self):
        #Wait for 500ms the first time through to allow skewer to fully insert then only wait 100ms each time.
        if ((self.collectTimer.millis()>=self.servoDelayOpp) and (self.angleToWrite != self.skew_angles[0]))\
            or ((self.collectTimer.millis()>=self.servoDelayOurs) and (self.angleToWrite == self.skew_angles[0])):
            self.collectTimer.reset()
            #Extend to collect block
            if self.oppBlockPUS==0:
                self.skewer.write(self.skew_angles[0])
                self.oppBlockPUS = 1
            
            elif self.oppBlockPUS==1:
                #Slowly increment until it is fully retracted
                if self.angleToWrite < self.skew_angles[1]:
                    self.angleToWrite += 1
                    self.skewer.write(min(self.angleToWrite,self.skew_angles[1]))
                #When fully retracted revert back to waiting for block state
                if self.angleToWrite >= self.skew_angles[1]:
                    self.angleToWrite = self.skew_angles[0]
                    self.blockPUS = -1
                    self.oppBlockPUS = 0
    
    def dropStack(self):
        if self.collectTimer.millis()>=self.servoDelayDropStack:
            self.collectTimer.reset()
            if self.dropState==0:
                self.grabber.write(self.grabber_angles[1]) #take grabber out
                self.dropState=1
                
            elif self.dropState==1:
#                self.elevator.write(self.ele_angles[1]) #drop elevator
                self.door.write(self.door_angles[1]) #open door
                self.dropState=2
                
            elif self.dropState==2:
#                self.elevator.write(self.ele_angles[0]) #raise elevator
                self.dropState=0
                self.blockPUS=-1
                

if __name__ == "__main__":
    sketch = ServoWrite(1, -0.00001, 100)
    sketch.run()
