#!/usr/bin/env python

"""
Publishes input from the gamepad as a motor message for the direction and pwm of l/r motors.
"""
# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost')
import Settings
import time
import numpy as np
import math

import roslib
import rospy
from std_msgs.msg import String
from robot_stack.msg import MotorMsg, DriveMsg, GyroMsg, StateMsg, EasyOdomMsg

class driveToCoord():

    def __init__(self):
        
        self.motor_msg = MotorMsg()
        self.rate = rospy.Rate(20.0)
        self.max_pwm = 180 # doesn't allow motor pwm signals to exceed this

        # current_gyro and desired_angle_gyro_cf used to calculate diff
        self.current_gyro = 0 # current gyro reading
        self.desired_angle_gyro_cf = 0
        self.diff = 0

        self.bias = 0
        self.power = 0
        self.integral = 0
        self.derivative = 0
        self.last_diff = 0

        self.left_motor_val = 0
        self.right_motor_val = 0

        #PID gains
        self.kP = 1.6 #Settings.kP
        self.kI = 0.01 #Settings.kI
        self.kD = 0 #Settings.kD

        self.lastTime = time.time()
        self.dt = time.time() - self.lastTime
        

        # drive_sub waits for new drive commands from the path planner node
        self.drivePub = rospy.Publisher('robot/drive', DriveMsg, queue_size=10)

        # gyro sub listens for new gyro data
        self.gyro_sub = rospy.Subscriber('robot/gyro', GyroMsg, self.updateGyro)
        
        # gyro sub listens for new gyro data
        self.odom_sub = rospy.Subscriber('robot/easyOdom', EasyOdomMsg, self.updatePosition)

        
        # create a motor publisher for the RELEASE state
        # motor pub sends pwm/dir commands to the motors
        self.stateSub = rospy.Subscriber('robot/state', StateMsg, self.updateState)
        self.state = 'STOP'
        
        self.shouldDriveToCoord = False
        self.path = [(12,0),(12,12),(0,12),(0,0,0)]

        # main loop runs at 20 Hz
        while not rospy.is_shutdown():
            #connect this state to something in state node
            if self.shouldDriveToCoord:
                #while there are more points to go to
                if len(self.path)>0:
                    #calculate the bias and desired angle
                    if self.drive_to_coord(*self.path[0]):
                        self.path.pop(0)
                    self.drivePub.publish(DriveMsg(self.desired_angle_gyro_cf,self.bias))

            self.rate.sleep()


    def updateGyro(self, data):
        """
        Every time the gyro reading changes, update the local gyro value
        """
        self.current_gyro = data.angle

    def updateState(self, data):
        self.state = data.state

    def updatePosition(self, data):
        self.x = data.x
        self.y = data.y

    def getClosestEquivalentAngle(self,angle):
        #takes an angle and uses robots current position to determine which angle to turn 
        pa = self.theta_est - (self.theta_est%360 - angle%360)
        potentialAngles = [pa-360,pa,pa+360]
        differences = [abs(self.theta_est - potentialAngles[0]), abs(self.theta_est - potentialAngles[1]), abs(self.theta_est - potentialAngles[2])]
        return potentialAngles[differences.index(min(differences))]

#    def drive_to_coord(self, xTarg, yTarg, thetaTarg = None):
#        if self.driveToCoordState == 0:
#            angleToTurnTo = -math.degrees(math.atan2(yTarg - self.y,xTarg - self.x))
#            self.desired_angle_gyro_cf = self.getClosestEquivalentAngle(angleToTurnTo)
#            if abs(self.current_gyro - self.desired_angle_gyro_cf) < 5:
#                self.bias = self.init_motor_speed
#            else:
#                self.bias = 0
#            distanceToBlock = np.sqrt((self.x - xTarg)**2 + (self.y- yTarg)**2)
#            if distanceToBlock <= 2:
#                self.driveToCoordState = 1
#        elif self.driveToCoordState == 1:
#            self.bias = 0
#            if thetaTarg == None:
#                self.desired_angle_gyro_cf = self.current_gyro
#            else:
#                self.desired_angle_gyro_cf = self.getClosestEquivalentAngle(-thetaTarg)
#            if abs(self.current_gyro - self.desired_angle_gyro_cf) < 5:
#                print ">>>>>>>>>>>>>>>>>>>move on"
#                self.driveToCoordState = 0
#                return True
#        return False

    def drive_to_coord(self, xTarg, yTarg, thetaTarg = None):
        angleToTurnTo = -math.degrees(math.atan2(yTarg - self.y,xTarg - self.x))
        self.desired_angle_gyro_cf = self.getClosestEquivalentAngle(angleToTurnTo)
        if abs(self.current_gyro - self.desired_angle_gyro_cf) < 5:
            self.bias = self.init_motor_speed
        else:
            self.bias = 0
        distanceToBlock = np.sqrt((self.x - xTarg)**2 + (self.y- yTarg)**2)
        if distanceToBlock <= 5:
            self.bias = 0
            if thetaTarg == None:
                self.desired_angle_gyro_cf = self.current_gyro
            else:
                self.desired_angle_gyro_cf = self.getClosestEquivalentAngle(-thetaTarg)
            if abs(self.current_gyro - self.desired_angle_gyro_cf) < 5:
                print ">>>>>>>>>>>>>>>>>>>move on"
                self.driveToCoordState = 0
                return True
        return False


if __name__ == '__main__':
    rospy.init_node('drive_pid_controller')
    try:
        driveToCoord = driveToCoord()
    except rospy.ROSInterruptException: pass