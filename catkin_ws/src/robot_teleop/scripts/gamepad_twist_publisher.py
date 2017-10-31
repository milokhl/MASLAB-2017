#!/usr/bin/env python

"""
Publishes input from the gamepad as a twist message for linear and angular velocity of the robot.
"""

import roslib
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class gamepadTwistPublisher():
    def __init__(self):
        self.msg = Twist()
        self.rate = rospy.Rate(10.0)

        rospy.Subscriber("/joy", Joy, self.updateTwist)

        pub = rospy.Publisher('teleop_cmd_vel', Twist, queue_size=10)

        while not rospy.is_shutdown():
            pub.publish(self.msg)
            self.rate.sleep()


    def updateTwist(self, data):
        """
        axes = [Lx, Ly, Rx, Ry, PadX, PadR]
        buttons = [x, a, b, y, LB, RB, LT, RT, BACK, START, L_click, R_click]

        Note: Positive X is left, Positive Y is up
        """
        self.msg.linear.x = data.axes[1]
        self.msg.angular.z = data.axes[2]


if __name__ == '__main__':
    rospy.init_node('joystick_twist_publisher')
    try:
        gamepadTwistPublisher = gamepadTwistPublisher()
    except rospy.ROSInterruptException: pass