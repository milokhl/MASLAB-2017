#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost')
import Settings
import numpy
from math import degrees, radians

import roslib
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from robot.msg import GyroMsg, EncoderMsg


class robotOdometryPublisher():
    """
    Publishes a nav_msgs/Odometry msg called "robot/odometry"
    Note: this odometry message only includes yaw angular velocity in the Twist
    """
    def __init__(self):
        self.yawAngle = 0 # degrees
        self.yawAngularVelocity = 0 # deg / sec
        self.oldEncLeft = 0
        self.oldEncRight = 0
        self.distTravelled = 0

        self.msg = Odometry()
        self.rate = rospy.Rate(20.0)

        # subscribe the the gyro and encoders
        rospy.Subscriber("/robot/gyro", GyroMsg, self.gyroUpdate)
        rospy.Subscriber("/robot/encoders", EncoderMsg, self.updateOdometry)

        # create a publisher for the odometry, and one for the tf 
        odom_pub = rospy.Publisher('robot/odometry', Odometry, queue_size=10)
        baselink_in_odom_publisher = tf.TransformBroadcaster()
        
        # publish the odometry at 10 Hz
        while not rospy.is_shutdown():
            # publish the nav_msgs/Odometry message
            odom_pub.publish(self.msg)

            # send the baselink in odometry tf
            baselink_in_odom_publisher.sendTransform((self.msg.pose.pose.position.x, self.msg.pose.pose.position.y, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, radians(self.yawAngle)),
                         rospy.Time.now(),
                         "base_link",
                         "odom")

            # print out where the odometry says the robot is
            #self.printState()

            self.rate.sleep()

    def gyroUpdate(self,data):
        """
        Whenever the gyro value changes, this function gets called.
        """
        self.yawAngle = -data.angle
        self.yawAngularVelocity = -data.angular_vel

        # set the current orientation of the robot in the pose
        self.msg.twist.twist.angular.z = self.yawAngularVelocity
        self.msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, radians(self.yawAngle))

    def printState(self):
        pose = self.msg.pose.pose.position
        print "X:%f   Y:%f   Z:%f    Ang:%f" % (pose.x, pose.y, pose.z, self.yawAngle)


    def updateOdometry(self, data):
        """
        Whenever encoders change, this function gets called.
        """
        deltaL = (data.left_encoder_val - self.oldEncLeft) * Settings.wheel_circumference_meters/3200.0
        deltaR = (data.right_encoder_val - self.oldEncRight) * Settings.wheel_circumference_meters/3200.0
        if (abs(deltaL - deltaR)) <= 1e-6:
            dx = deltaL * numpy.cos(radians(self.yawAngle))
            dy = deltaR * numpy.sin(radians(self.yawAngle))

        else:
            r = Settings.axel_length_meters * (deltaL + deltaR) / (2 * (deltaR - deltaL))
            wd = (deltaR - deltaL) / Settings.axel_length_meters
            
            dx = r * numpy.sin(radians(self.yawAngle) + wd) - r*numpy.sin(radians(self.yawAngle))
            dy = -r * numpy.cos(radians(self.yawAngle) + wd) + r*numpy.cos(radians(self.yawAngle))
            #self.heading += wd
  

        self.distTravelled += numpy.sqrt(dy**2 + dx**2)
        self.oldEncLeft = data.left_encoder_val
        self.oldEncRight = data.right_encoder_val

        # pose update
        self.msg.pose.pose.position.x += dx
        self.msg.pose.pose.position.y += dy
        self.msg.pose.pose.position.z = 0 # always 0


if __name__ == '__main__':
    rospy.init_node('robot_odometry_publisher')
    try:
        robotOdomoetryPublisher = robotOdometryPublisher()
    except rospy.ROSInterruptException: pass
