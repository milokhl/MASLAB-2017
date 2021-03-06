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
            M = cv2.moments(cnt)
            if M['m00']!=0:
                Cx = int(M['m10']/M['m00'])
                Cy = int(M['m01']/M['m00'])
            else:
                Cx,Cy=0,0
            print Cx-width/2,Cy
            print "angle:", (60.0*Cx/width - 30)
            frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
#            distance = calculateDistanceOfBlock(cnt,height)
#            print "Distance:",distance
        else:
            frame2 = cv2.drawContours(frame2,[cnt],0,[0,0,0],-1)
#            pass
            
    frame2 = cv2.drawContours(frame2, realContours, -1, (255,0,255), 3)

    if biggestArea != 0.0:
#        distance = calculateDistanceOfBlock(biggestCnt,height)
        v = 100*biggestArea/(height*width)
        v=1/v
        dist = -25.452*(v**2) + 51.157*v - 0.7276
        print "Distance:",dist

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow("Original", frame)
    cv2.imshow("Filtered", frame2)
    
    #cv2.imshow('frame',frame2)
    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


