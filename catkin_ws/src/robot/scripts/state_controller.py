#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings

import roslib
import rospy
import time
from robot.msg import VisionMsg, StateMsg, PickUpMsg, ColorSensorMsg, DriveBusyMsg


class stateController(object):
    """
    SCAN: Rotating slowly in search of nearby blocks.
    NAV_BLOCK: Navigating towards a FOV block using webcam.
    EXTRA: Going a small amount of extra distance to make sure the block gets inside.
    POS_BLOCK: Go forward 2 inches to make sure block is positioned correctly (happens when color sensor first sees a block)
    COLL_OUR: In the process of collecting our color block.
    COLL_OPP: In the process of collecting an opp. block.
    NAV_BUTTON: Navigating towards the button (in view)
    NAV_BASE: Navigating towards our base (with path planning)
    UNSTICK: Trying to get away from a wall that we're stuck on.
    RELEASE: Releasing our stack by opening the door and slowly backing away.
    REVERSE: Backing away from the stack at the end of the match.
    STOP: Doing nothing. We start and end the match in the STOP state.
    """
    def __init__(self):

        # COLOR STUFF
        self.ourColor = Settings.ourColor
        print "$$$ OUR COLOR SET TO %s $$$" % self.ourColor
        self.rg_Ratio = Settings.colorSenserg
        self.rb_Ratio = Settings.colorSenserb
        self.gr_Ratio = Settings.colorSensegr
        self.gb_Ratio = Settings.colorSensegb
        # create a subscriber to the color sensor messages
        self.colorSub = rospy.Subscriber('robot/color_sensor', ColorSensorMsg, self.checkColor)
        self.colorState = -1

        # VISION STUFF
        self.visionSub = rospy.Subscriber('robot/vision', VisionMsg, self.updateVisionState)
        self.blockVisible = False
        self.buttonVisible = False
        self.blockAngle = 0.0
        self.blockDistance = float('inf')

        # PICK UP STUFF
        self.pickUpSub = rospy.Subscriber('robot/pick_up_state', PickUpMsg, self.updatePickUpState)
        self.pickUpBusy = False
        self.numOurBlocks = 0
        self.numOppBlocks = 0
        self.maxOurBlocks = 10
        self.maxOppBlocks = 6

        # DRIVE STUFF
        self.driveSub = rospy.Subscriber('robot/drive_busy', DriveBusyMsg, self.updateDriveBusy)
        self.driveBusy = False

        # SET UP TO PUBLISH ON robot/state
        self.rate = rospy.Rate(20)
        self.statePub = rospy.Publisher('/robot/state', StateMsg, queue_size=10)
        self.stateMsg = StateMsg('SCAN') # don't move at first
        
        # MATCH AND TIMEOUT STUFF
        self.matchStartTime = time.time()
        self.elapsedTime = 0
        self.releaseTime = 160 # stop and release the stack at this point (sec)
        self.lastState = self.stateMsg.state
        self.lastStateTime = time.time()
        self.stateHistory = ['SCAN']

        # for each state, we have an associated timeout
        # after being in this state for the timeout duration, we call a handleTimeout() function
        # which puts us in another state
        self.stateTimeouts = {
            'SCAN':10.0, 
            'NAV_BLOCK':8.0, 
            'UNSTICK':11.0, 
            'NAV_BUTTON':8.0,
            'NAV_BASE':30.0,
            'EXTRA': 5.0, 
            'REVERSE':8.0, 
            'STOP':float('inf'),
            'DROP': 10.0,
            'COLL_OUR':6.0,
            'COLL_OPP':6.0,
            'RELEASE':10.0,
            'POS_BLOCK':4.0,
            'CLEARANCE':5.0
        }

        
        # runs at 20 Hz
        while not rospy.is_shutdown():

            if self.stateMsg.state == 'STOP':
                rospy.sleep(0.5)
                continue

            # get the current duration of the match (sec)
            self.elapsedTime = time.time() - self.matchStartTime

            # IF REACHED TIME LIMIT OR BLOCK LIMIT, ENTER ENDGAME
            if self.elapsedTime > self.releaseTime:

                print "Match timer reached."
                # while pick up action is happening, wait
                while self.pickUpBusy:
                    rospy.sleep(0.1)
                
                print "Pick up not busy, going to clearance"
#----------------------                                             
                # let the pick up node know to make clearance for the stack
                self.stateMsg.state = "CLEARANCE"
                self.statePub.publish(self.stateMsg)

                rospy.sleep(1.0) # wait for pick up to indicate that it's busy

                start = time.time()
                print "Making clearance."
                while self.driveBusy:
                    if (time.time() - start) > 8:
                        break
                    rospy.sleep(0.1)
                print "Done clearing."

                #rospy.sleep(1.0)
                # make sure no block is under the stack before we drop
                if self.colorState == 0 and self.stateMsg.state!="UNSTICK": # our block
                    self.stateMsg.state = "COLL_OUR"
                elif self.colorState == 1 and self.stateMsg.state!="UNSTICK": # opp block
                    self.stateMsg.state = "COLL_OPP"
                self.statePub.publish(self.stateMsg)

                rospy.sleep(0.5)
                # wait until area under stack is cleared
                while self.pickUpBusy:
                    rospy.sleep(0.1)
                rospy.sleep(0.2)

                # let the pick up node know to release the stack
                self.stateMsg.state = "RELEASE"
                self.statePub.publish(self.stateMsg)
                rospy.sleep(1.5) # wait for pick up to indicate that it's busy

                print "Releasing stack."
                while self.pickUpBusy: # wait while the stack is released
                    print "Pick up busy?", self.pickUpBusy
                    rospy.sleep(0.1)
                print "Done releasing."

                rospy.sleep(1.0)
                print "Reversing from stack."
                self.stateMsg.state = "REVERSE"
                self.statePub.publish(self.stateMsg)
                print "Start reversing" 
                                                                
                rospy.sleep(1.0) #wait to give robot time to update busy
                
                while self.driveBusy: # wait while reversing
                    rospy.sleep(0.1)
                print "Finished reversing from stack."

                print "Stopping"
                self.stateMsg.state = "STOP"
                self.statePub.publish(self.stateMsg)


            else: # still time and blocks left

                # check if we've reached the timeout for our current state in the timeout dictionary
                self.stateTimer = (time.time() - self.lastStateTime)

                if (self.stateTimer > self.stateTimeouts[self.stateMsg.state]):
                    self.handleTimeout(self.stateMsg.state)

                # if not in the process of collecting something
                elif not self.pickUpBusy:

                    if self.colorState == 0 and self.stateMsg.state!="UNSTICK": # our block
                        self.stateMsg.state = "COLL_OUR"
                    elif self.colorState == 1 and self.stateMsg.state!="UNSTICK": # opp block
                        self.stateMsg.state = "COLL_OPP"

                    else:
                        print "drive Busy?", self.driveBusy
                        # if we're busy with an important driving task, do not bother
                        if not self.driveBusy:
                            # now check what vision says
                            if self.blockVisible: # if we can see a block
                                if self.blockDistance < 6.0: # close enough for straight shot
                                    self.stateMsg.state = "EXTRA"
                                    # publish the current state
                                    self.statePub.publish(self.stateMsg)
                                    rospy.sleep(0.5)

                                else:
                                    self.stateMsg.state = "NAV_BLOCK"
                                    

                                    if (len(self.stateHistory) >= 4):
                                        print self.stateHistory[len(self.stateHistory)-4:len(self.stateHistory)]
                                        if self.stateHistory[len(self.stateHistory)-4:len(self.stateHistory)] == ['NAV_BLOCK', 'SCAN','NAV_BLOCK','SCAN']:
                                            print "Detected repeat scan nav"
                                            self.stateMsg.state = "UNSTICK"
                                            self.statePub.publish(self.stateMsg)
                                            

                                    self.statePub.publish(self.stateMsg)
                                    rospy.sleep(0.5)


                                    

                            #elif not self.blockVisible and self.stateMsg.state == 'NAV_BLOCK':
                                # self.stateMsg.state = "UNSTICK"
                                # self.statePub.publish(self.stateMsg)
                                # rospy.sleep(0.5)

                            else: # no block in sight
                                self.stateMsg.state = "SCAN"

                else:
                    pass

            

            # check if we changed state, and if so, reset the lastStateTime to now
            print "ELAPSED: %.2f | STATE: %s %.2f | VISION: %s | PICKUP: %s | " \
                % (self.elapsedTime, self.stateMsg.state, self.stateTimer, self.blockVisible, self.pickUpBusy)

            # if we change states, reset the state timer
            if self.stateMsg.state != self.lastState: 
                self.stateHistory.append(self.stateMsg.state)
                self.lastStateTime = time.time()
                self.stateTimer = 0
                self.lastState = self.stateMsg.state

            # publish the current state
            self.statePub.publish(self.stateMsg)

            self.rate.sleep()


    def handleTimeout(self, timeoutState):
        """
        Whenever we hit a timeout, this function is called to determine which
        state we enter next.

        timeoutState: the state that we reached a timeout in
        """
        print "HIT TIMEOUT OF %s" % timeoutState
        if timeoutState == 'SCAN':
            self.stateMsg.state = 'UNSTICK'

        elif timeoutState == 'EXTRA':
            self.stateMsg.state = 'UNSTICK'

        elif timeoutState == 'COLL_OPP' or timeoutState == 'COLL_OUR':
            self.stateMsg.state = 'UNSTICK'

        elif timeoutState == 'NAV_BLOCK':
            self.stateMsg.state = 'UNSTICK'

        elif timeoutState == 'UNSTICK':
            self.stateMsg.state = 'EXTRA'
            # publish the current state
            self.statePub.publish(self.stateMsg)
            rospy.sleep(1.0)

        elif timeoutState == 'REVERSE':
            self.stateMsg.state = 'STOP'

        elif timeoutState == 'NAV_BUTTON':
            self.stateMsg.state = 'UNSTICK'

        elif timeoutState == 'POS_BLOCK':
            self.stateMsg.state = 'UNSTICK'

        else: # by default, go back to scan
            self.stateMsg.state = 'SCAN'

    def reverseFromStack(self):
        # TODO 
        pass


    def updatePickUpState(self, data):
        """
        Whenever the pick_up node or the target_blocks node change the pick up state,
        this function gets called.
        """
        self.pickUpBusy = data.busy # update whether or not the pick up node is doing something
        self.numOurBlocks, self.numOppBlocks = data.num_our_blocks, data.num_opp_blocks


    def updateVisionState(self, data):
        """
        Whenever we get a new vision message, update the local vision variables.
        """
        self.blockVisible = data.block_visible
        self.buttonVisible = data.button_visible
        self.blockAngle = data.block_angle
        self.blockDistance = data.block_distance

    def updateDriveBusy(self, data):
        self.driveBusy = data.busy


    def checkColor(self, data):
        """
        Checks the current color values from the color sensor.
        Determines is our block, opp block, or none is at the sensor.
        Updates the block pick up state accordingly.
        """
        #self.colorState = -1
        #when we're in release mode of reverse mode, don't check the color sensor any more
        if (self.stateMsg.state != 'RELEASE') and (self.stateMsg.state != 'REVERSE') and (self.stateMsg.state != 'STOP'):
            #self.colorState = -1
            if (data.r>self.rg_Ratio*data.g) and (data.r>self.rb_Ratio*data.b):
                blockColor = "red"
                #print "red block"
            elif (data.g>self.gr_Ratio*data.r) and (data.g>self.gb_Ratio*data.b):
                blockColor = "green"
                #print "green block"
            else:
                blockColor = "None"
                #print "no block"
                
            if blockColor == "None":
                self.colorState = -1
            elif blockColor == self.ourColor:
                self.colorState = 0
            else:
                self.colorState=1



if __name__ == '__main__':
    rospy.init_node('state_controller')
    try:
        stateController = stateController()
    except rospy.ROSInterruptException: pass
