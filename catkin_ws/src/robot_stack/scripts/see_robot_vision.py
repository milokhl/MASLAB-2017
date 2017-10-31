#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings as S

import numpy as np, cv2
import roslib
import rospy
from robot_stack.msg import VisionMsg, DriveMsg, GyroMsg, EncoderMsg, StateMsg, PickUpMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class visionDisplay(object):
    """
    Based on the current vision msg, sends drive commands to navigate to a block in the robot's FOV.
    """
    def __init__(self):
        self.frameSub = rospy.Subscriber('robot/image_raw', Image, self.updateFrame)
        self.redFrameSub = rospy.Subscriber('robot/image_red', Image, self.updateRedFrame)
        self.greenFrameSub = rospy.Subscriber('robot/image_green', Image, self.updateGreenFrame)
        self.height = 480
        self.width = 640
        self.frame = np.zeros([self.height,self.width,3],np.uint8)
        self.redFrame = np.zeros([self.height,self.width,3],np.uint8)
        self.greenFrame = np.zeros([self.height,self.width,3],np.uint8)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()

        # runs at 20 Hz
        while not rospy.is_shutdown():
            #show image
            cv2.imshow('Original Image', self.frame)
            cv2.imshow('Red Image', self.redFrame)
            cv2.imshow('Green Image', self.greenFrame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            self.rate.sleep()
        cv2.destroyAllWindows()

    def updateFrame(self,data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
    
    def updateRedFrame(self,data):
        try:
            self.redFrame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
    def updateGreenFrame(self,data):
        try:
            self.greenFrame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)



if __name__ == '__main__':
    rospy.init_node('vision_display')
    try:
        visionDisplay = visionDisplay()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()