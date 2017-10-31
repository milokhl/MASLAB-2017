# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 13:28:43 2017

@author: Alec
"""
import numpy as np
import cv2
import time
import copy

cap = cv2.VideoCapture(0)
#cap.set(3,100)
#cap.set(4,60)

count = 0
start = time.time()

rg = 1.7
rb = 1.7
gr = 1.4
gb = 1.1

while(True):
    if (time.time()-start)>=3:
        
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()

    frame2 = copy.copy(frame)

    b,g,r = cv2.split(frame2)
    frame2[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    frame2[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    frame2[((b>(rb*r)) & (b>(gb*g)))] = [255,0,0]
    frame2[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b)))|((b>(rb*r)) & (b>(gb*g))))] = [0,0,0]

    imgray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,25,255,0)
    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    realContours = []
    height, width, channels = frame2.shape
    minArea  = 0.007*height*width
    for cnt in contours:
        if cv2.contourArea(cnt)>=minArea:
            realContours.append(cnt)
            M = cv2.moments(cnt)
            Cx = int(M['m10']/M['m00'])
            Cy = int(M['m01']/M['m00'])
            #print Cx-width/2,Cy
            #print "angle:", (60.0*Cx/width - 30)
            frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,255]
            if frame2[cnt[0][0][1],cnt[0][0][0]][0] == 255:
                for pt in cnt:
#                    for y in range(0,pt[0][1]):
#                        frame2[y,pt[0][0]] = [0,0,0]
                    frame2[0:pt[0][1],pt[0][0]] = [0,0,0]
                    
        else:
            frame2 = cv2.drawContours(frame2,[cnt],0,[0,0,0],-1)
            
    frame2 = cv2.drawContours(frame2, realContours, -1, (255,0,255), 3)
    
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
