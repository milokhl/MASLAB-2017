#!/usr/bin/env python

"""
Publishes an odometry tf to the tf tree. Uses input from a Logitech gamepad to simulate odometry values.
"""
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from sensor_msgs.msg import Joy
from math import degrees, radians, cos, sin

# class Quat(object):
#     def __init__(self):
#         self.w = 0
#         self.x = 0
#         self.y = 0
#         self.z = 0


# def eulerToQuat(pitch, roll, yaw):
#     """
#     pitch, roll, yaw are float angles (radians)
#     return a Quat object
#     """
#     t0 = cos(yaw * 0.5)
#     t1 = sin(yaw * 0.5)
#     t2 = cos(roll * 0.5)
#     t3 = sin(roll * 0.5)
#     t4 = cos(pitch * 0.5)
#     t5 = sin(pitch * 0.5)

#     q = Quat()
#     q.w = t0 * t2 * t4 + t1 * t3 * t5
#     q.x = t0 * t3 * t4 - t1 * t2 * t5
#     q.y = t0 * t2 * t5 + t1 * t3 * t4
#     q.z = t1 * t2 * t4 - t0 * t3 * t5

#     return q


class gamepadOdometryPublisher():
    def __init__(self):
        self.yawAngle = 0 # radians

        self.msg = Odometry()
        self.rate = rospy.Rate(10.0)
        self.lastUpdateTime = rospy.get_time() # float secs
        self.lastJoyData = Joy()
        self.lastJoyData.axes = [0 for i in range(6)]
        self.lastJoyData.buttons = [0 for i in range(12)]

        rospy.Subscriber("/joy", Joy, self.updateOdometry)

        odom_pub = rospy.Publisher('joy_odometry', Odometry, queue_size=10)

        baselink_in_odom_publisher = tf.TransformBroadcaster()
        
        # publish the odometry at 10 Hz
        while not rospy.is_shutdown():
            # update the odometry even if we haven't received a new Joy msg
            self.updateOdometry(self.lastJoyData)

            # publish the nav_msgs/Odometry message
            odom_pub.publish(self.msg)

            # send the baselink in odometry tf
            baselink_in_odom_publisher.sendTransform((self.msg.pose.pose.position.x, self.msg.pose.pose.position.y, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, self.yawAngle),
                         rospy.Time.now(),
                         "base_link",
                         "odom")

            # print out where the odometry says the robot is
            self.printState()

            self.rate.sleep()

    def printState(self):
        pose = self.msg.pose.pose.position
        print "X:%f   Y:%f   Z:%f    Ang:%f" % (pose.x, pose.y, pose.z, degrees(self.yawAngle))


    def updateOdometry(self, data):
        """
        axes = [Lx, Ly, Rx, Ry, PadX, PadR]
        buttons = [x, a, b, y, LB, RB, LT, RT, BACK, START, L_click, R_click]

        Note: Positive X is left, Positive Y is up

        Right stick controls yaw rotation.
        Left stick controls X and Y coordinates (not completely accurate to the movement of robot)
        """
        # set the latest joy data
        self.lastJoyData = data

        #twist update
        self.msg.twist.twist.linear.x = data.axes[1] # X is forward
        self.msg.twist.twist.linear.y = data.axes[0] # Y is left
        self.msg.twist.twist.angular.z = data.axes[2] # Rstick L/R

        # integrate twist to update the pose
        updateTime = rospy.get_time()
        dt = updateTime - self.lastUpdateTime
        self.lastUpdateTime = updateTime

        # pose update
        self.msg.pose.pose.position.x += self.msg.twist.twist.linear.x * dt
        self.msg.pose.pose.position.y += self.msg.twist.twist.linear.y * dt
        self.msg.pose.pose.position.z = 0 # always 0

        self.yawAngle += self.msg.twist.twist.angular.z * dt # radians/sec * sec
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yawAngle)
        self.msg.pose.pose.orientation = quat
        # self.msg.pose.pose.orientation.x = quat.x
        # self.msg.pose.pose.orientation.y = quat.y
        # self.msg.pose.pose.orientation.z = quat.z

if __name__ == '__main__':
    rospy.init_node('joystick_odometry_publisher')
    try:
        gamepadTwistPublisher = gamepadOdometryPublisher()
    except rospy.ROSInterruptException: pass

