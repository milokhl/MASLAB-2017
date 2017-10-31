# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 13:28:43 2017

@author: arinz
"""
import numpy as np
import cv2
import time
import copy

cap = cv2.VideoCapture(0)
#cap.set(3,100)
#cap.set(4,60)
#cap.set(15, -4.0)

count = 0
start = time.time()

rg = 1.7
rb = 1.7
gr = 1.15
gb = 1.15


def calculateDistanceOfBlock(cnt,height):
    blockHeight = 2.0 #inches
    sensorHeight = 2.0 #inces
    f = 3.0#1.5748 #focal length inches #actual is 4mm
    x,y,w,h = cv2.boundingRect(cnt) #pixels
    distance = f * blockHeight*height/(h*sensorHeight)
    return cv2.contourArea(cnt)

def get_center_of_mass(cnt):
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

def get_angle(Cx, width):
    """
    input: x value of center of mass, width of the frame
    return the angle in front of the camera, 0 is in the middle, positive to the right, and negative to the left
    """
    return (60.0*Cx/width - 30)

def is_button(cnt, threshold):
    """
    cnt: a contour, threshold: a float, for the minimum ratio needed for a contour to be a circle
    return: True if the contour is a circle, False otherwise
    """
    areaOld = cv2.contourArea(cnt)
    epsilon= 0.04 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(biggestCnt, epsilon, True)
    a = cv2.contourArea(approx)
    
    ratio = float(biggestArea)/a
    return ratio > threshold

while(True):
    if (time.time()-start)>=3:
        
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()

#    frame2 = copy.deepcopy(frame)
    frame2 = copy.copy(frame)

    b,g,r = cv2.split(frame2)
    frame2[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    frame2[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    frame2[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
    frame3 = copy.copy(frame2)
    imgray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,50,255,0)
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    realContours = []
    height, width, channels = frame2.shape
    minArea  = 0.01*height*width
#    minArea = 0
    biggestCnt = None
    biggestArea = 0.0

    for cnt in contours:
        cntArea = cv2.contourArea(cnt)
        if cntArea>=minArea:
            if cntArea>biggestArea:
                biggestCnt = cnt
                biggestArea = cntArea
            realContours.append(cnt)
            Cx,Cy= get_center_of_mass(cnt)
            print Cx-width/2,Cy
            print "angle:", (60.0*Cx/width - 30)
            frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
#            distance = calculateDistanceOfBlock(cnt,height)
#            print "Distance:",distance
        else:
            frame2 = cv2.drawContours(frame2,[cnt],0,[0,0,0],-1)
#            pass
    
#    shapes = []
#    for cnt in realContours:
#        epsilon= 0.05 * cv2.arcLength(cnt, True)
#        approx = cv2.approxPolyDP(cnt, epsilon, True)
#        shapes.append(approx)
    if biggestArea > 0:
        epsilon= 0.01 * cv2.arcLength(biggestCnt, True)
        approx = cv2.approxPolyDP(biggestCnt, epsilon, True)
        a = cv2.contourArea(approx)
        p = cv2.arcLength(approx,True)
        
        #print "ratio", float(biggestArea)/((p/4) **2)
        print "ratio", float(a)/((p/4) **2)
    
    frame2 = cv2.drawContours(frame2, [biggestCnt], -1, (255,0,255), 3)
#    frame2 = cv2.drawContours(frame2, realContours, -1, (255,0,255), 3)
    frame3 = cv2.drawContours(frame3, [approx], -1, (255, 0, 255), 3)
    M = cv2.moments(approx)
    if M['m00']!=0:
        Cx = int(M['m10']/M['m00'])
        Cy = int(M['m01']/M['m00'])
    else:
        Cx,Cy=0,0
    frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
    
    if biggestArea != 0.0:
#        distance = calculateDistanceOfBlock(biggestCnt,height)
        v = 100*biggestArea/(height*width)
        v=1/v
        dist = -25.452*(v**2) + 51.157*v - 0.7276
        #print "Distance:",dist

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow("Original", frame)
    cv2.imshow("Filtered", frame2)
    cv2.imshow("approximated", frame3)
    
    #cv2.imshow('frame',frame2)
    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


