#!/usr/bin/env python

"""
Publishes input from the gamepad as a motor message for the direction and pwm of l/r motors.
"""
# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost')
import Settings
import time

import roslib
import rospy
from std_msgs.msg import String
from robot.msg import MotorMsg, DriveMsg, GyroMsg

class drivePIDController():

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
        self.kP = 1.6 #Settings.kP #3.0 #Settings.kP
        self.kI = 0 # Settings.kI #0.05 #Settings.kI
        self.kD = 0 #Settings.kD #0 #Settings.kD

        self.lastTime = time.time()
        self.dt = time.time() - self.lastTime

        # drive_sub waits for new drive commands from the path planner node
        drive_sub = rospy.Subscriber('robot/drive', DriveMsg, self.updateDesiredAngleAndSpeed)

        # gyro sub listens for new gyro data
        gyro_sub = rospy.Subscriber('robot/gyro', GyroMsg, self.updateGyro)

        # motor pub sends pwm/dir commands to the motors
        motor_pub = rospy.Publisher('robot/motors', MotorMsg, queue_size=10)


        # main loop runs at 20 Hz
        while not rospy.is_shutdown():
            # calculate power for each motor
            self.calculatePower()


            # publish servo and motor messages
            motor_pub.publish(self.motor_msg)

            self.rate.sleep()

    def calculatePower(self):
        self.dt = time.time() - self.lastTime
        self.lastTime = time.time()

        # main loop runs at 20 Hz
        self.diff = self.desired_angle_gyro_cf - self.current_gyro
        #Use PID and update motors
        self.integral += self.diff*self.dt
        self.derivative = (self.diff - self.last_diff)/self.dt
        self.power = self.kP*self.diff + self.kI*self.integral + self.kD*self.derivative
        self.last_diff = self.diff

        # update motor values
        self.left_motor_val = self.bias + self.power
        self.right_motor_val = self.bias - self.power

        #print "Bias:", self.bias, "L:", self.left_motor_val, "R:", self.right_motor_val

        # update the motor message commands
        self.motor_msg.left_motor_dir = self.left_motor_val > 0
        self.motor_msg.left_motor_pwm = min(abs(self.left_motor_val), self.max_pwm) # don't allow motors to exceed max power
        self.motor_msg.right_motor_dir = self.right_motor_val > 0
        self.motor_msg.right_motor_pwm = min(abs(self.right_motor_val), self.max_pwm)

    def updateGyro(self, data):
        """
        Every time the gyro reading changes, update the local gyro value
        """
        self.current_gyro = data.angle


    def updateDesiredAngleAndSpeed(self, data):
        """
        Every time we receive a new drive command, the local drive commands are updated.
        """
        if self.bias != data.desired_speed: # if we're changing speed, reset the integral to avoid suddent jumps
            self.integral = 0

        self.bias = data.desired_speed # the bias is more or less equivalent to the forward speed of the robot
        self.desired_angle_gyro_cf = data.desired_angle_gyro_cf # desired angle in the GYRO COORDINATE FRAME!


if __name__ == '__main__':
    rospy.init_node('drive_pid_controller')
    try:
        drivePIDController = drivePIDController()
    except rospy.ROSInterruptException: pass