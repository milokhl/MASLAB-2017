#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings as S

import numpy as np, cv2
import roslib
import rospy
from robot_stack.msg import VisionMsg, DriveMsg, GyroMsg, EncoderMsg, StateMsg, PickUpMsg


class blockTargeter(object):
    """
    Based on the current vision msg, sends drive commands to navigate to a block in the robot's FOV.

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
        self.DriveMsg = DriveMsg()
        self.current_gyro = 0.0
        self.visionSub = rospy.Subscriber('/robot/vision', VisionMsg, self.updateDriveMsg)
        self.drivePub = rospy.Publisher('robot/drive', DriveMsg, queue_size=10)
        self.gyroSub = rospy.Subscriber('robot/gyro', GyroMsg, self.updateGyro)
        self.encoderSub = rospy.Subscriber('robot/encoders', EncoderMsg, self.updateEncoders)
        self.stateSub = rospy.Subscriber('robot/state', StateMsg, self.updateState)
        self.pickUpPub = rospy.Publisher('robot/pick_up_state', PickUpMsg, queue_size=5)
        self.pickUpSub = rospy.Subscriber('robot/pick_up_state', PickUpMsg, self.updatePickUpMsg)
        self.pickUpMsg = PickUpMsg()
        self.leftEncoderVal = 0
        self.rightEncoderVal = 0
        self.approachSpeed = 80
        self.blockDistance = float('inf')
        self.blockAngle = 0
        self.state = "STOP"
        self.unstickDirection = 1 # starts CW
        self.rate = rospy.Rate(20)

        # runs at 20 Hz
        while not rospy.is_shutdown():
            # if we're in STOP or RELEASE, don't drive!
            if (self.state == 'STOP') or (self.state == 'RELEASE'):
                self.drivePub.publish(DriveMsg(self.current_gyro,0))

            # see if we aren't allowed to drive due to the current state
            elif (self.state == 'COLL_OUR') or (self.state == 'COLL_OPP'):
                # stop the robot
                self.drivePub.publish(DriveMsg(self.current_gyro,0))

            elif self.state == 'REVERSE':
                self.reverseFromStack(speed=-45)

                # this pick up state tells the state controller that we're DONE
                self.pickUpMsg.pick_up_state = 10
                self.pickUpPub.publish(self.pickUpMsg)
                

            elif self.state == 'NAV_BLOCK': # allowed to drive

                 # if the block is close enough to get in a straight approach
                if self.blockDistance < 6.0: # and abs(self.blockAngle) < 10.0:
                    self.driveExtra() # go forward another 14 inches straight
            
                else:
                    self.drivePub.publish(self.DriveMsg)

            elif self.state == 'SCAN':
                # have the robot slowly spin CW
                self.DriveMsg.desired_speed = 0
                self.DriveMsg.desired_angle_gyro_cf = self.current_gyro+25
                self.drivePub.publish(self.DriveMsg)

            elif self.state == 'UNSTICK':
                # in the unstick state, we reuse the code for reversing from the stack, but we add in a slight turn
                self.reverseFromStack(speed=-50, turn_amt=20*self.unstickDirection, distance=-8.0)
                print "Finished reverse from stack"
                rospy.sleep(0.5)
                self.unstickDirection *= -1 # reverse unstick directions for the next time
                self.pickUpMsg.pick_up_state = -4
                self.pickUpPub.publish(self.pickUpMsg)
                rospy.sleep(0.5)

            else:
                pass

            self.rate.sleep()

    def driveExtra(self):
        """
        When a block is close/centered enough to get by going straight, goes an extra 14 inches.
        As soon as a block is detected by the color sensor, goes forward another 2 inches.
        """
        # if we're reversing or dropping the stack, don't allow the robot to drive extra
        #print "Calling drive extra"
        # do not allow driving when in REVERSE or RELEASE mode
        if self.state != "REVERSE" and self.state != "RELEASE":

            # tell the state controller that we're in EXTRA
            self.pickUpMsg.pick_up_state = -2
            self.pickUpPub.publish(self.pickUpMsg)

            L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
            dist = 0
            while dist < 12 and self.pickUpMsg.pick_up_state==-2:
                # keep publishing the fact that we're driving the extra dist
                #self.pickUpMsg.pick_up_state = -2
                #self.pickUpPub.publish(self.pickUpMsg)
                # tell the robot to keep driving forward at speed 80 for 14 more inches
                dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
                self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
                self.DriveMsg.desired_speed = 80 # go slow into the block
                self.drivePub.publish(self.DriveMsg)
                rospy.sleep(0.05)

            # we either went 12 inches or got a block near the color sensor
            if self.pickUpMsg.pick_up_state == 0 or self.pickUpMsg.pick_up_state == 1:
                print "Going extra to get block positioned"
                # if the color sensor is near a block, go forward 2 more inches to ensure correct positioning
                L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
                dist = 0

                # tell state controller we're in POS_BLOCK state
                self.pickUpMsg.pick_up_state = -3
                self.pickUpPub.publish(self.pickUpMsg)
                while dist < 2:
                    print "Going extra to get block positioned"
                    dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
                    self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
                    self.DriveMsg.desired_speed = 50 # go slow into the block
                    self.drivePub.publish(self.DriveMsg)
                    rospy.sleep(0.05)


    def updatePickUpMsg(self, data):
        """
        If another node updates the pick up state, we update our local copy here.
        """
        self.pickUpMsg.pick_up_state = data.pick_up_state
        self.pickUpMsg.num_our_blocks, self.pickUpMsg.num_opp_blocks = data.num_our_blocks, data.num_opp_blocks

    def reverseFromStack(self, speed=-50, turn_amt=0, distance=-6.0):
        print "Called reverse from stack"
        rospy.sleep(0.5)
        L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
        dist = 0
        while dist > distance:
            print "Driving to dist in reverse from stack"
            # avg distance travelled by encoders
            dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
            
            # tell robot to keep driving backwards for 6 inches
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro + turn_amt
            self.DriveMsg.desired_speed = speed # go slow into the block
            self.drivePub.publish(self.DriveMsg)
            rospy.sleep(0.05) # rate limit to 20 Hz


    def updateDriveMsg(self, data):
        if data.block_visible == True:
            self.blockAngle = data.block_angle
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro + data.block_angle
            self.DriveMsg.desired_speed = self.approachSpeed
            self.blockDistance = data.block_distance

        # otherwise do nothing
        else:
            self.DriveMsg.desired_speed = 0
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
            self.blockAngle = 0
            self.blockDistance = float('inf')


    def updateEncoders(self, data):
        self.leftEncoderVal = data.left_encoder_val
        self.rightEncoderVal = data.right_encoder_val

    def updateGyro(self, data):
        """
        Every time the gyro reading changes, update the local gyro value
        """
        self.current_gyro = data.angle

    def updateState(self, data):
        """
        Listens for state changes from the state controller.
        """
        self.state = data.state



if __name__ == '__main__':
    rospy.init_node('target_blocks')
    try:
        blockTargeter = blockTargeter()
    except rospy.ROSInterruptException: pass
