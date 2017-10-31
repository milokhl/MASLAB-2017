#!/usr/bin/env python

# import settings
import sys
sys.path.insert(0, '/home/maslab/Code/team1/TAMProxy-pyHost') # path to folder containing Settings.py
import Settings as S

import numpy as np, cv2
import roslib
import rospy
from robot_stack.msg import VisionMsg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class visionController(object):

    def __init__(self):

        self.cap = cv2.VideoCapture(0)
        ret, self.frame = self.cap.read()

        # wait until the webcam is connected
        while ret==False: 
            print "No camera found at index 0"
            pass

        # get frame information that will be useful for object position estimation
        self.height, self.width, channels = self.frame.shape
        self.rate = rospy.Rate(20)
        self.minFraction = 0.002 #percent of pixels needed to be considered a block
        self.target = None # this will contain a "target" contour that the vision is navigating us towards
        self.rg = S.rg
        self.rb = S.rb
        self.gr = S.gr
        self.gb = S.gb
        self.redF = np.zeros((self.height,self.width,3),np.uint8)
        self.greenF = np.zeros((self.height,self.width,3),np.uint8)

        # create a VisionMsg publisher
        self.VisionMsg = VisionMsg()
        self.visionPub = rospy.Publisher('/robot/vision', VisionMsg, queue_size=10)
        self.imageRawPub = rospy.Publisher('/robot/image_raw', Image, queue_size=1)
        self.imageRedPub = rospy.Publisher('/robot/image_red', Image, queue_size=1)
        self.imageGreenPub = rospy.Publisher('/robot/image_green', Image, queue_size=1)

        self.bridge = CvBridge()


        # runs this loop at 30Hz
        while not rospy.is_shutdown():
            
            ret, self.frame = self.cap.read()

            # if a new frame was received
            if ret:
                #print "getting frame"
                # extract a red frame and green frame (filtered)
                self.redF, self.greenF = self.filter_color(self.frame, self.height, self.width)

                # get contours in the red and green frames
                cntRed = self.filter_contours(self.redF,self.height,self.width,self.minFraction)
                cntGreen = self.filter_contours(self.greenF,self.height,self.width,self.minFraction)

                allCnt = cntRed
                [allCnt.append(i) for i in cntGreen]

                #print allCnt
                allCnt = sorted(allCnt, key=cv2.contourArea, reverse=True)
            
                # if a red/green contour was found
                allCntNoButtons = []
                if len(allCnt) > 0:
                    for cnt in allCnt:
                        if not self.is_button(cnt):
                            allCntNoButtons.append(cnt)
                        else: # is a button
                            #print "removing button"
                            pass

                if len(allCntNoButtons)>0:
                    #print "has target"
                    # target the largest cnt in our field of view (closest block)
                    self.target = allCntNoButtons[0]

                    # compute the centroid of the target cnt
                    Cx, Cy = self.get_center_of_mass(self.target)

                    self.VisionMsg.block_visible = True
                    self.VisionMsg.block_angle = self.get_angle(Cx, self.width)
                    self.VisionMsg.block_distance = self.get_distance(self.target, self.height, self.width)

                else: # no blocks visible
                    self.VisionMsg.block_visible = False
                    self.VisionMsg.block_angle = 0.0
                    self.VisionMsg.block_distance = 0.0

                # publish the updated vision msg (if we got a new frame from the cam)
            
                self.visionPub.publish(self.VisionMsg)

            # publish the raw, red, and green image topics
            self.imageRawPub.publish(self.bridge.cv2_to_imgmsg(self.frame))
            self.imageRedPub.publish(self.bridge.cv2_to_imgmsg(self.redF))
            self.imageGreenPub.publish(self.bridge.cv2_to_imgmsg(self.greenF))

            # delay to set the frame rate
            self.rate.sleep()

        # make sure to release the camera when quitting
        self.cap.release()

    #def is_button(self, c, threshold = 1):
        #return False
        # # initialize the shape name and approximate the contour
        # shape = "unidentified"
        # peri = cv2.arcLength(c, True)
        # approx = cv2.approxPolyDP(c, 0.005 * peri, True)
        # print "approx", len(approx)
        # if len(approx) <= 13:
        # 	# compute the bounding box of the contour and use the
        # 	# bounding box to compute the aspect ratio
        #     # a square will have an aspect ratio that is approximately
        #     # equal to one, otherwise, the shape is a rectangle
        #     shape = False
        #     # otherwise, we assume the shape is a circle
        # else:
        #     shape = True
        #     print "Found Button!"
        #     # return the name of the shape
        # return shape

    def is_button(self, cnt, threshold=1):
        area = cv2.contourArea(cnt)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print "area/approx:",(100*area/cv2.contourArea(box))**2
        if (100*area/cv2.contourArea(box))**2<6000:
            print "Sees button"
            return False
        return False

    def get_center_of_mass(self, cnt):
        """
        takes in a contour and finds the center of mass enclosed in the contour
        returns Cx, Cy
        to color it black: frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
        """
        M = cv2.moments(cnt)

        if M['m00']!=0:
            Cx = int(M['m10']/M['m00'])
            Cy = int(M['m01']/M['m00'])
            return Cx, Cy
        else:
            return 0,0

    def get_angle(self, Cx, width):
        """
        input: x value of center of mass, width of the frame
        return the angle in front of the camera, 0 is in the middle, positive to the right, and negative to the left
        """
        return (60.0*float(Cx)/width - 30.0)


    def get_distance(self, cnt, height, width):
        
        v = 100*cv2.contourArea(cnt)/(height*width)
        v=1/v
        dist = -0.1582*(v**2) + 3.9643*v + 5.3521
        return dist


    def filter_contours(self, frame, height, width, ratio = 0.01):
        
        imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,50,255,0)
        if ret==False:
            return []

        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        minArea = ratio*height*width
        realContours = []
        for cnt in contours:
            cntArea = cv2.contourArea(cnt)
            if cntArea>=minArea:
                realContours.append(cnt)
            else:
                frame = cv2.drawContours(frame,[cnt],0,[0,0,0],-1)

        # sort the contours by area   
        #sortedCnts = sorted(realContours,key=cv2.contourArea, reverse=True)
        return realContours
        

    def filter_color(self, frame, height, width):
        
        filteredFrameRed = np.zeros((height,width,3),np.uint8)
        filteredFrameGreen = np.zeros((height,width,3),np.uint8)

        b,g,r = cv2.split(frame)
        filteredFrameRed[((r>(self.rg*g)) & (r>(self.rb*b)))] = [0,0,255]
        filteredFrameGreen[((g>(self.gr*r)) & (g>(self.gb*b)))] = [0,255,0]
        
        return filteredFrameRed,filteredFrameGreen



if __name__ == '__main__':
    rospy.init_node('vision')
    try:
        visionController = visionController()
    except rospy.ROSInterruptException:
        visionController.cap.release()