#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings

import roslib
import rospy
import time
from robot_stack.msg import ColorSensorMsg, ServoMsg, PickUpMsg, MotorMsg, StateMsg


class pickUpController(object):
    """
    Pick up states:
    -1 : no block
    -2: drive extra to get block
    -3: color sensor seen block, going another 2 inches
    -4: finished unsticking
    0 : collecting our block
    1: collecting opp block
    2 : stack has been dropped
    """
    
    def __init__(self):

        self.rate = rospy.Rate(5)
        self.ourColor = Settings.ourColor
        self.pickUpState = -1 # -1: none, 0: collecting our block, 1: collecting opp block
        self.numOurBlocks = 0
        self.numOppBlocks = 0

        # get ratio values used to check a block's color
        self.rg_Ratio = Settings.colorSenserg
        self.rb_Ratio = Settings.colorSenserb
        self.gr_Ratio = Settings.colorSensegr
        self.gb_Ratio = Settings.colorSensegb
        #self.gr_MaxRatio = Settings.colorSenseMaxgr

        # get preset positions from settings
        self.grabber_in_pos, self.grabber_out_pos = Settings.grabber_angles[0], Settings.grabber_angles[1]
        self.elevator_up_pos, self.elevator_down_pos = Settings.ele_angles[0], Settings.ele_angles[1]
        self.skewer_out_pos, self.skewer_in_pos = Settings.skew_angles[1], Settings.skew_angles[0]
        self.door_closed_pos, self.door_open_pos = Settings.door_angles[0], Settings.door_angles[1]

        # create a subscriber to the color sensor messages and a publisher for the servos
        self.colorSub = rospy.Subscriber('robot/color_sensor', ColorSensorMsg, self.checkColor)
        self.servoPub = rospy.Publisher('robot/servos', ServoMsg, queue_size=10)
        

        # create a publisher for the pick up state
        self.pickUpPub = rospy.Publisher('robot/pick_up_state', PickUpMsg, queue_size=5)
        self.pickUpMsg = PickUpMsg(-1, self.numOurBlocks, self.numOppBlocks)
        self.colorState = -1

        # set defaults
        self.servoMsg = ServoMsg()
        self.servoMsg.grabber_servo_pos = self.grabber_out_pos
        self.servoMsg.elevator_servo_pos = self.elevator_up_pos
        self.servoMsg.skewer_servo_pos = self.skewer_out_pos
        self.servoMsg.door_servo_pos = self.door_closed_pos

        # create a motor publisher for the RELEASE state
        # motor pub sends pwm/dir commands to the motors
        self.stateSub = rospy.Subscriber('robot/state', StateMsg, self.updateState)
        self.state = 'STOP'

        # runs at 5 Hz
        while not rospy.is_shutdown():

            if self.state == 'RELEASE':

                # the self.pickUpState will change to 2, so when this is published it lets the state controller know that the stack
                # is dropped
                self.dropStackAndOpen()
                # after dropped stack, pick up state is 2
                self.pickUpState = 2
                self.pickUpMsg.pick_up_state = self.pickUpState
                self.pickUpPub.publish(self.pickUpMsg)

            else:

                # publish the current pick up state for the state controller to use
                #self.pickUpPub.publish(self.pickUpMsg)

                if self.colorState == 0: # get our block

                    #update the pick up state, go through the routine to pick up our block
                    self.pickUpState = 0
                    self.pickUpMsg.pick_up_state = self.pickUpState
                    self.pickUpPub.publish(self.pickUpMsg)

                    rospy.sleep(1.5) # wait 2 sec
                    # grabber out, elevator down, grabber in, elevator up
                    self.servoMsg.grabber_servo_pos = self.grabber_out_pos
                    self.servoPub.publish(self.servoMsg)
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(0.5) # wait 1 sec
                    self.servoMsg.elevator_servo_pos = self.elevator_down_pos
                    self.servoPub.publish(self.servoMsg)
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(0.5) # wait 1 sec
                    self.servoMsg.grabber_servo_pos = self.grabber_in_pos
                    self.servoPub.publish(self.servoMsg)
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(0.5) # wait 1 sec
                    self.servoMsg.elevator_servo_pos = self.elevator_up_pos
                    self.servoPub.publish(self.servoMsg)
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(2.0)

                    self.numOurBlocks+=1
                    self.pickUpMsg.num_our_blocks = self.numOurBlocks

                    # reset pick up state
                    self.pickUpState = -1
                    self.colorState = -1
                    self.pickUpMsg.pick_up_state = self.pickUpState
                    self.pickUpPub.publish(self.pickUpMsg)

                elif self.colorState == 1: # get opp block

                    # update the pick up state and go through opp. collection routine
                    self.pickUpState = 1
                    self.pickUpMsg.pick_up_state = self.pickUpState
                    self.pickUpPub.publish(self.pickUpMsg)

                    rospy.sleep(1.5) # wait 2 sec
                    self.servoMsg.skewer_servo_pos = self.skewer_in_pos
                    self.servoPub.publish(self.servoMsg)
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(0.5) # wait 1 sec
                    self.servoMsg.skewer_servo_pos = self.skewer_out_pos
                    self.servoPub.publish(self.servoMsg)
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(2.0) # wait 1 sec

                    self.numOppBlocks += 1
                    self.pickUpMsg.num_opp_blocks = self.numOppBlocks

                    # reset pick up state
                    self.pickUpState = -1
                    self.colorState = -1
                    self.pickUpMsg.pick_up_state = self.pickUpState
                    self.pickUpPub.publish(self.pickUpMsg)

            self.rate.sleep()

    def updateState(self, data):
        self.state = data.state

    def checkColor(self, data):
        """
        Checks the current color values from the color sensor.
        Determines is our block, opp block, or none is at the sensor.
        Updates the block pick up state accordingly.
        """
        self.colorState = -1
        # when we're in release mode of reverse mode, don't check the color sensor any more
        if (self.state != 'RELEASE') and (self.state != 'REVERSE') and (self.state != 'STOP'):
            self.colorState = -1
            # if (data.r>self.rg_Ratio*data.g) and (data.r>self.rb_Ratio*data.b):
            #     blockColor = "red"
            #     #print "red block"
            # elif (data.g>self.gr_Ratio*data.r) and (data.g>self.gb_Ratio*data.b):
            #     blockColor = "green"
            #     #print "green block"
            # else:
            #     blockColor = "None"
            #     #print "no block"
                
            # if blockColor == "None":
            #     self.colorState = -1
            # elif blockColor == self.ourColor:
            #     self.colorState = 0
            # else:
            #     self.colorState=1

    
    def dropStackAndOpen(self):
        """
        Drops our stack and opens the front door.
        When finished, sets the pickUpState to 2, which tells the state controller that it's time to reverse.
        """
        rospy.sleep(0.5) # wait 1 sec

        # grabber out
        self.servoMsg.grabber_servo_pos = self.grabber_out_pos
        self.servoPub.publish(self.servoMsg)
        rospy.sleep(2.5) # wait 1 sec

        # open the front door
        self.servoMsg.door_servo_pos = self.door_open_pos
        self.servoPub.publish(self.servoMsg)
        rospy.sleep(2.5)
        

        #print "Finished dropping stack"

    def updateEncoders(self, data):
        self.leftEncoderVal = data.left_encoder_val
        self.rightEncoderVal = data.right_encoder_val

        




if __name__ == '__main__':
    rospy.init_node('block_pick_up_controller')
    try:
        pickUpController = pickUpController()
    except rospy.ROSInterruptException: pass