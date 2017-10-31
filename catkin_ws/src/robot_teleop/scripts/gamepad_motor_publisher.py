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
from robot_stack.msg import MotorMsg, ServoMsg


class gamepadMotorPublisher():
    def __init__(self):
        self.motor_msg = MotorMsg()
        self.servo_msg = ServoMsg()

        self.rate = rospy.Rate(20.0)
        self.quit_button = False
        self.pwm_limit = 150

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

        rospy.Subscriber("/joy", Joy, self.updateMotors)

        motor_pub = rospy.Publisher('robot/motors', MotorMsg, queue_size=10)
        servo_pub = rospy.Publisher('robot/servos', ServoMsg, queue_size=10)

        while not rospy.is_shutdown():
            # publish servo and motor messages
            motor_pub.publish(self.motor_msg)
            servo_pub.publish(self.servo_msg)

            if self.quit_button == True:
                break

            self.rate.sleep()


    def updateMotors(self, data):
        """
        axes = [Lx, Ly, Rx, Ry, PadX, PadR]
        buttons = [x, a, b, y, LB, RB, LT, RT, BACK, START, L_click, R_click]

        Note: Positive X is left, Positive Y is up
        """
        if data.buttons[8]==1.0:
            print "Quitting gamepad motor publisher"
            self.quit_button=True

        # update the motor commands
        self.motor_msg.left_motor_dir = data.axes[1] > 0
        self.motor_msg.left_motor_pwm = int(abs(data.axes[1])*self.pwm_limit) # scale between 0 and 255
        self.motor_msg.right_motor_dir = data.axes[3] > 0
        self.motor_msg.right_motor_pwm = int(abs(data.axes[3])*self.pwm_limit) # scale between 0 and 255

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
        gamepadMotorPublisher = gamepadMotorPublisher()
    except rospy.ROSInterruptException: pass