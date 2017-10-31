#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings

import roslib
import rospy
import time
from robot.msg import ColorSensorMsg, ServoMsg, PickUpMsg, MotorMsg, StateMsg


class pickUpController(object):
    """
    Listens for states from the state controller, and does servo actions accordingly.
    """
    
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.numOurBlocks = 0
        self.numOppBlocks = 0

        # get preset positions from settings
        self.grabber_in_pos, self.grabber_out_pos = Settings.grabber_angles[0], Settings.grabber_angles[1]
        self.elevator_up_pos, self.elevator_down_pos = Settings.ele_angles[0], Settings.ele_angles[1]
        self.skewer_out_pos, self.skewer_in_pos = Settings.skew_angles[1], Settings.skew_angles[0]
        self.door_closed_pos, self.door_open_pos = Settings.door_angles[0], Settings.door_angles[1]

        self.servoPub = rospy.Publisher('robot/servos', ServoMsg, queue_size=10)
        
        # create a publisher for the pick up state
        self.pickUpPub = rospy.Publisher('robot/pick_up_state', PickUpMsg, queue_size=5)
        self.pickUpMsg = PickUpMsg(False, self.numOurBlocks, self.numOppBlocks)

        # set defaults
        self.servoMsg = ServoMsg()
        self.servoMsg.grabber_servo_pos = self.grabber_out_pos
        self.servoMsg.elevator_servo_pos = self.elevator_up_pos
        self.servoMsg.skewer_servo_pos = self.skewer_out_pos
        self.servoMsg.door_servo_pos = self.door_closed_pos

        # create a motor publisher for the RELEASE state
        # motor pub sends pwm/dir commands to the motors
        self.stateSub = rospy.Subscriber('robot/state', StateMsg, self.updateState)
        self.state = 'SCAN'

        # runs at 20 Hz
        while not rospy.is_shutdown():

            if self.state == 'RELEASE':
                self.pickUpMsg.busy = True
                self.pickUpPub.publish(self.pickUpMsg)

                #rospy.sleep(0.5) # wait 1 sec

                self.dropStackAndOpen()

                rospy.sleep(1.0)

                self.pickUpMsg.busy = False
                self.pickUpPub.publish(self.pickUpMsg)
                rospy.sleep(1.0)

            else:
                # publish the current pick up state for the state controller to use
                #self.pickUpPub.publish(self.pickUpMsg)
                if self.state == "COLL_OUR": # get our block

                    #update the pick up state, go through the routine to pick up our block
                    self.pickUpMsg.busy = True
                    self.pickUpPub.publish(self.pickUpMsg)

                    rospy.sleep(1.5)
                    
                    self.pickUpOur()

                    rospy.sleep(1.0)
                    self.numOurBlocks+=1
                    self.pickUpMsg.num_our_blocks = self.numOurBlocks

                    # let the state controller know we're no longer busy
                    self.pickUpMsg.busy = False
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(1.0)

                elif self.state == 'COLL_OPP': # get opp block

                    # tell the state controller we're busy
                    self.pickUpMsg.busy = True
                    self.pickUpPub.publish(self.pickUpMsg)

                    rospy.sleep(1.5) # wait 2 sec
                    
                    self.pickUpOpp()

                    rospy.sleep(1.0) # wait 1 sec

                    self.numOppBlocks += 1
                    self.pickUpMsg.num_opp_blocks = self.numOppBlocks

                    # reset pick up state
                    self.pickUpMsg.busy = False
                    self.pickUpPub.publish(self.pickUpMsg)
                    rospy.sleep(1.0)

            self.rate.sleep()


    def pickUpOur(self):
        # grabber out, elevator down, grabber in, elevator up
        self.servoMsg.grabber_servo_pos = self.grabber_out_pos
        self.servoPub.publish(self.servoMsg)

        rospy.sleep(0.5) # wait 1 sec
        self.servoMsg.elevator_servo_pos = self.elevator_down_pos
        self.servoPub.publish(self.servoMsg)

        rospy.sleep(0.5) # wait 1 sec
        self.servoMsg.grabber_servo_pos = self.grabber_in_pos
        self.servoPub.publish(self.servoMsg)

        rospy.sleep(0.5) # wait 1 sec
        self.servoMsg.elevator_servo_pos = self.elevator_up_pos
        self.servoPub.publish(self.servoMsg)


    def pickUpOpp(self):
        self.servoMsg.skewer_servo_pos = self.skewer_in_pos
        self.servoPub.publish(self.servoMsg)

        rospy.sleep(0.5) # wait 1 sec

        for i in range(2):
            val = self.skewer_in_pos

            # pull out the skewer slowly
            while val < self.skewer_out_pos:
                val += 1
                self.servoMsg.skewer_servo_pos = val
                self.servoPub.publish(self.servoMsg)
                rospy.sleep(0.01)


    def updateState(self, data):
        self.state = data.state


    def dropStackAndOpen(self):
        """
        Drops our stack and opens the front door.
        When finished, sets the pickUpState to 2, which tells the state controller that it's time to reverse.
        """
        # elevator down
        print "In drop stack function"
        self.servoMsg.elevator_servo_pos = self.elevator_down_pos
        self.servoPub.publish(self.servoMsg)
        rospy.sleep(0.9) # wait 1 sec

        # grabber out
        self.servoMsg.grabber_servo_pos = self.grabber_out_pos
        self.servoPub.publish(self.servoMsg)
        rospy.sleep(1.5) # wait 1 sec

        # open the front door
        self.servoMsg.door_servo_pos = self.door_open_pos
        self.servoPub.publish(self.servoMsg)
        print "Finished dropping stack"

    def updateEncoders(self, data):
        self.leftEncoderVal = data.left_encoder_val
        self.rightEncoderVal = data.right_encoder_val

        




if __name__ == '__main__':
    rospy.init_node('block_pick_up_controller')
    try:
        pickUpController = pickUpController()
    except rospy.ROSInterruptException: pass