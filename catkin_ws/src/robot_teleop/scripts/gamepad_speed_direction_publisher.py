#!/usr/bin/env python

"""
Publishes input from the gamepad as a motor message for the direction and pwm of l/r motors.
"""
# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost')
import Settings

import roslib
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from robot_stack.msg import DriveMsg, ServoMsg, GyroMsg
from math import atan, radians, degrees


class gamepadSpeedDirectionPublisher():
    def __init__(self):
        self.drive_msg = DriveMsg()
        self.servo_msg = ServoMsg()
        self.current_gyro = 0

        self.rate = rospy.Rate(20.0)
        self.quit_button = False

        # define the positions for important actions
        self.grabber_in_pos = Settings.grabber_angles[0]
        self.grabber_out_pos = Settings.grabber_angles[1]
        self.elevator_down_pos = Settings.ele_angles[1]
        self.elevator_up_pos = Settings.ele_angles[0]
        self.skewer_in_pos = Settings.skew_angles[0]
        self.skewer_out_pos = Settings.skew_angles[1]

        # make the servo msg the same as the initialized position of all servos
        self.servo_msg.grabber_servo_pos = Settings.grabber_angles[0] #grabber starts in
        self.servo_msg.elevator_servo_pos = Settings.ele_angles[0] #elevator starts up
        self.servo_msg.skewer_servo_pos = Settings.skew_angles[1] #skewer starts out

        joy_sub = rospy.Subscriber("/joy", Joy, self.updateCommands)
        gyro_sub = rospy.Subscriber("/robot/gyro", GyroMsg, self.updateGyro)

        drive_pub = rospy.Publisher('robot/drive', DriveMsg, queue_size=10)
        servo_pub = rospy.Publisher('robot/servos', ServoMsg, queue_size=10)

        while not rospy.is_shutdown():
            # publish servo and motor messages
            drive_pub.publish(self.drive_msg)
            servo_pub.publish(self.servo_msg)

            if self.quit_button == True:
                break

            self.rate.sleep()

    def updateGyro(self, data):
        self.current_gyro = data.angle

    def updateCommands(self, data):
        """
        axes = [Lx, Ly, Rx, Ry, PadX, PadR]
        buttons = [x, a, b, y, LB, RB, LT, RT, BACK, START, L_click, R_click]

        Note: Positive X is left, Positive Y is up
        """
        if data.buttons[8]==1.0:
            print "Quitting gamepad motor publisher"
            self.quit_button=True

        # update the motor commands
        self.drive_msg.desired_speed = int(255 * data.axes[1])

        if abs(data.axes[2]) > 0.6 or abs(data.axes[3]) > 0.6: # ignore small commands
            # to avoid divide by zero error

            if data.axes[3]>0 and data.axes[2]<0:
                theta = degrees(atan(-data.axes[2] / data.axes[3])) if data.axes[3] != 0 else self.current_gyro

            elif data.axes[3]<0 and data.axes[2]<0:
                theta = 90 + degrees(atan(data.axes[3] / data.axes[2])) if data.axes[2] != 0 else self.current_gyro

            elif data.axes[3]<0 and data.axes[2]>0:
                theta = 180 + degrees(atan(data.axes[2] / -data.axes[3])) if data.axes[3] != 0 else self.current_gyro

            elif data.axes[2]==0:
                theta = 0 if data.axes[3] >= 0 else 180

            elif data.axes[3]==0:
                theta = 90 if data.axes[2] <= 0 else 270
            else:
                theta = 360 - degrees(atan(data.axes[2] / data.axes[3])) if data.axes[3] != 0 else self.current_gyro

            self.drive_msg.desired_angle_gyro_cf = theta

        else:
            self.drive_msg.desired_angle_gyro_cf = self.current_gyro

        # update the servo commands
        if data.buttons[0] == 1: # x
            self.servo_msg.grabber_servo_pos = self.grabber_in_pos
        if data.buttons[2] == 1: # b
            self.servo_msg.grabber_servo_pos = self.grabber_out_pos
        if data.buttons[1] == 1: # a
            self.servo_msg.elevator_servo_pos = self.elevator_down_pos
        if data.buttons[3] == 1: # y
            self.servo_msg.elevator_servo_pos = self.elevator_up_pos
        if data.buttons[4] == 1: # LB: skewer out
            self.servo_msg.skewer_servo_pos = self.skewer_out_pos
        if data.buttons[5] == 1: # RB: skewer in
            self.servo_msg.skewer_servo_pos = self.skewer_in_pos


if __name__ == '__main__':
    rospy.init_node('gamepad_motor_publisher')
    try:
        gamepadSpdDirPublisher = gamepadSpeedDirectionPublisher()
    except rospy.ROSInterruptException: pass