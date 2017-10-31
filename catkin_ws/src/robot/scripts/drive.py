#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings as S

import numpy as np, cv2
import random
import roslib
import rospy
from robot.msg import VisionMsg, DriveMsg, GyroMsg, EncoderMsg, StateMsg, DriveBusyMsg


class driveManager(object):
    """
    Based on the current vision msg, sends drive commands to navigate to a block in the robot's FOV.
    """
    def __init__(self):
        
        self.visionSub = rospy.Subscriber('robot/vision', VisionMsg, self.updateVision)
        self.drivePub = rospy.Publisher('robot/drive', DriveMsg, queue_size=10)
        self.gyroSub = rospy.Subscriber('robot/gyro', GyroMsg, self.updateGyro)
        self.encoderSub = rospy.Subscriber('robot/encoders', EncoderMsg, self.updateEncoders)
        self.stateSub = rospy.Subscriber('robot/state', StateMsg, self.updateState)
    
        # vision stuff
        self.blockVisible = False
        self.buttonVisible = False
        self.blockAngle = 0.0
        self.blockDistance = float('inf')

        # driving values
        self.DriveBusyMsg = DriveBusyMsg(False)
        self.driveBusyPub = rospy.Publisher('robot/drive_busy', DriveBusyMsg, queue_size=10)
        self.DriveMsg = DriveMsg()
        self.current_gyro = 0.0
        self.leftEncoderVal = 0
        self.rightEncoderVal = 0
        self.approachSpeed = 80
        self.state = "SCAN"
        self.unstickDirection = 1 # starts CW
        self.rate = rospy.Rate(20)

        # runs at 20 Hz
        while not rospy.is_shutdown():

            # if the servos are busy doing something
            if (self.state == 'COLL_OUR') or (self.state == 'COLL_OPP'):
                self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
                self.DriveMsg.desired_speed = 0
                self.drivePub.publish(self.DriveMsg)
                # let state controller know we're not busy (overridden)
                self.DriveBusyMsg.busy = False
                self.driveBusyPub.publish(self.DriveBusyMsg)
                
            elif self.state == 'NAV_BLOCK': # allowed to drive
                self.DriveMsg.desired_angle_gyro_cf = self.current_gyro + self.blockAngle
                self.DriveMsg.desired_speed = self.approachSpeed #0 if abs(self.blockAngle)>10 else self.approachSpeed
                self.drivePub.publish(self.DriveMsg)

            elif self.state == 'EXTRA':
                print "EXTRA: settings busy to true"
                self.DriveBusyMsg.busy = True
                self.driveBusyPub.publish(self.DriveBusyMsg)
                self.driveExtra()
    

                # let state controller know we're not busy
                self.DriveBusyMsg.busy = False
                self.driveBusyPub.publish(self.DriveBusyMsg)
                rospy.sleep(0.5)

            elif self.state == 'SCAN':
                # have the robot slowly spin CW
                self.DriveMsg.desired_speed = 0
                self.DriveMsg.desired_angle_gyro_cf = self.current_gyro + 35
                self.drivePub.publish(self.DriveMsg)

            elif self.state == 'UNSTICK':
                self.DriveBusyMsg.busy = True
                print "UNSTICK: setting busy to true"
                self.driveBusyPub.publish(self.DriveBusyMsg)
                # choose randomly between 60 and 180
                angle = random.random()*120 + 60
                print "Unsticking at angle %f" % angle
                self.unstick(back_up_speed=-50, forward_speed=50, turn_angle=angle*self.unstickDirection, \
                    forward_distance=6.0)

                self.unstickDirection *= -1 # reverse unstick directions for the next time
                
                # let state controller know we're not busy
                self.DriveBusyMsg.busy = False
                self.driveBusyPub.publish(self.DriveBusyMsg)
                print "Unstick Published that drive not busy"
                rospy.sleep(0.5)
                print "Done sleeping after not busy publish"

            elif self.state == 'REVERSE':
                self.DriveBusyMsg.busy = True
                print "REVERSE: setting busy to true"
                self.driveBusyPub.publish(self.DriveBusyMsg)

                self.reverseFromStack()

                # let state controller know we're not busy
                self.DriveBusyMsg.busy = False
                self.driveBusyPub.publish(self.DriveBusyMsg)
                rospy.sleep(0.5)
                                                                
            elif self.state=='CLEARANCE':
                self.DriveBusyMsg.busy = True
                print "CLEARANCE: setting busy to true"
                self.driveBusyPub.publish(self.DriveBusyMsg)

                self.makeClearForDrop()

                # let state controller know we're not busy
                self.DriveBusyMsg.busy = False
                self.driveBusyPub.publish(self.DriveBusyMsg)
                rospy.sleep(0.5)
            else:
                pass

            self.rate.sleep()

    def driveExtra(self, distance=14.5):
        """
        When a block is close/centered enough to get by going straight, goes an extra 14 inches.
        As soon as a block is detected by the color sensor, goes forward another 2 inches.
        """
        L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
        dist = 0
        while dist < distance:
            if self.state != 'EXTRA':
                break

            dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
            self.DriveMsg.desired_speed = 80 # go slow into the block
            self.drivePub.publish(self.DriveMsg)
            rospy.sleep(0.05)

        # stop when done
        self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
        self.DriveMsg.desired_speed = 0 # go slow into the block
        self.drivePub.publish(self.DriveMsg) 


    def unstick(self, back_up_speed=-50, forward_speed=50, turn_angle=20, back_up_distance=3.0, forward_distance=6.0):
        # back up slightly
        L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
        dist = 0
        print "Starting back up"
        while dist > -back_up_distance:
            if self.state != "UNSTICK":
                self.DriveBusyMsg.busy = False
                self.driveBusyPub.publish(self.DriveBusyMsg)
                break
            # avg distance travelled by encoders
            dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
            
            # tell robot to keep driving backwards for 6 inches
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
            self.DriveMsg.desired_speed = back_up_speed # go slow into the block
            self.drivePub.publish(self.DriveMsg)
            rospy.sleep(0.05) # rate limit to 20 Hz

        print "Finished back up"
        # rotate to given angle
        angle = turn_angle + self.current_gyro
        self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
        while abs(angle-self.current_gyro) > 10:
            if self.state != "UNSTICK":
                break

            if abs(self.DriveMsg.desired_angle_gyro_cf-self.current_gyro)<35:
                self.DriveMsg.desired_angle_gyro_cf += 1
            
            #print abs(turn_angle-self.current_gyro)
            #self.DriveMsg.desired_angle_gyro_cf = min(angle, self.current_gyro+25) if angle>self.current_gyro else max(angle, self.current_gyro-25)
            self.DriveMsg.desired_speed = 0 # go slow into the block
            self.drivePub.publish(self.DriveMsg)
            rospy.sleep(0.05) # rate limit to 20 Hz
        self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
        self.drivePub.publish(self.DriveMsg)
        print "Done turning"
        # now go forward 6 inches
        L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
        dist = 0
        while dist < forward_distance:
            #print "dist:", dist
            if self.state != "UNSTICK":
                break

            # avg distance travelled by encoders
            dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
            
            # tell robot to keep driving backwards for 6 inches
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
            self.DriveMsg.desired_speed = forward_speed # go slow into the block
            self.drivePub.publish(self.DriveMsg)
            rospy.sleep(0.05) # rate limit to 20 Hz

        # stop when done
        self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
        self.DriveMsg.desired_speed = 0 # go slow into the block
        self.drivePub.publish(self.DriveMsg) 
        print "Done going forward"

    def reverseFromStack(self, speed=-50, turn_amt=0, distance=-6.0):
        print "Called reverse from stack."
        rospy.sleep(0.5)
        L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
        dist = 0
        while dist > distance:
            if self.state != "REVERSE":
                break
            print "Driving to dist in reverse from stack"
            # avg distance travelled by encoders
            dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
            
            # tell robot to keep driving backwards for 6 inches
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro + turn_amt
            self.DriveMsg.desired_speed = speed # go slow into the block
            self.drivePub.publish(self.DriveMsg) 
            rospy.sleep(0.05) # rate limit to 20 Hz

        # stop when done
        self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
        self.DriveMsg.desired_speed = 0 # go slow into the block
        self.drivePub.publish(self.DriveMsg) 

    def makeClearForDrop(self, speed=50, turn_amt=0, distance=7.0):
        print "Called reverse from stack."
        rospy.sleep(0.5)
        L_start, R_start = self.leftEncoderVal, self.rightEncoderVal
        dist = 0
        while dist < distance:
            if self.state != "CLEARANCE":
                break
            print "Driving to dist in clearance"
            # avg distance travelled by encoders
            dist = (((self.leftEncoderVal-L_start) + (self.rightEncoderVal-R_start)) / 2) * S.wheel_circumference / S.counts_per_rotation
            
            # tell robot to keep driving backwards for 6 inches
            self.DriveMsg.desired_angle_gyro_cf = self.current_gyro + turn_amt
            self.DriveMsg.desired_speed = speed # go slow into the block
            self.drivePub.publish(self.DriveMsg) 
            
            rospy.sleep(0.05) # rate limit to 20 Hz

        # stop when done
        self.DriveMsg.desired_angle_gyro_cf = self.current_gyro
        self.DriveMsg.desired_speed = 0 # go slow into the block
        self.drivePub.publish(self.DriveMsg) 

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

    def updateVision(self, data):
        """
        Whenever we get a new vision message, update the local vision variables.
        """
        self.blockVisible = data.block_visible
        self.buttonVisible = data.button_visible
        self.blockAngle = data.block_angle
        self.blockDistance = data.block_distance


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
    rospy.init_node('drive_manager')
    try:
        driveManager = driveManager()
    except rospy.ROSInterruptException: pass
