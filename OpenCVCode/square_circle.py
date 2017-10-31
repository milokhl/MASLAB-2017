# -*- coding: utf-8 -*-
"""
Created on Mon Jan 23 21:11:38 2017
contains functions for:
getting center of mass
getting angle
detecting buttons

@author: arinz
"""


import numpy as np
import cv2
import time
import copy

rg = 1.7
rb = 1.7
gr = 1.15
gb = 1.0

fileNames= ['green_button.jpg','red_button.jpg','green_block.jpg','red_block.jpg', 'blocks_2.jpg', 'blocks_3.jpg','blocks_4.jpg', 'blocks_4_1.jpg']
#fileNames = ['green_button.jpg','red_button.jpg','red_block.jpg','blocks_2.jpg','blocks_4_1.jpg']

frames = {}

for fileName in fileNames:
        frame = cv2.imread(fileName)
        frames[fileName] = frame
#frame = cv2.imread()
#frame = cv2.imread()
#frame = cv2.imread('green_button.jpg')
#frame = cv2.imread('red_block.jpg')
#frame = cv2.imread('green_block.jpg')

def is_button(cnt, threshold = 1.3):
    """
    cnt: a contour, threshold: a float, for the minimum ratio needed for a contour to be a circle
    return: True if the contour is a circle, False otherwise
    """
    areaOld = cv2.contourArea(cnt)
    epsilon= 0.04 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    a = cv2.contourArea(approx)
    try:
        ratio = float(areaOld)/a
    except ZeroDivisionError:
        return True
#    print ratio
    return ratio > threshold

def is_button_shape(cnt, threshold = 0.04):
    """
    cnt: a contour, threshold: a float, the maximum value for the cnt to be a button
    use shape matching in opencv to determine if contour is a block
    return True if the contour matches a circle, False otherwise
    """
    with np.load('button_contour.npz') as X:
        match = [X[i] for i in ('cnt',)]
    ret = cv2.matchShapes(cnt,match[0],1,0.0)
    print ret
    return ret < threshold

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
    
def get_distance(cnt,height,width):
    
    v = 100*cv2.contourArea(cnt)/(height*width)
    v=1/v
    dist = -25.452*(v**2) + 51.157*v - 0.7276
    return dist

def filter_contours(frame,height,width,ratio = 0.01):
    
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
            
    sortedCnts = sorted(realContours,key=cv2.contourArea)
    sortedCnts.reverse()
    return sortedCnts
    #sort realcontours by area

def filter_color(frame,height,width):
    
    filteredFrameRed = np.zeros((height,width,3),np.uint8)
    filteredFrameGreen = np.zeros((height,width,3),np.uint8)

    b,g,r = cv2.split(frame)
    filteredFrameRed[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    filteredFrameGreen[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    
    return filteredFrameRed,filteredFrameGreen
    
for fileName in fileNames:
    print "-------------------------"
    print fileName
    frame = frames[fileName]
    height, width, channels = frame.shape
    redF,greenF = filter_color(frame,height,width)
    cntRed = filter_contours(redF,height,width)
    cntGreen = filter_contours(greenF,height,width)
    
    allCnt = np.append(cntRed,cntGreen)
    
    redF = cv2.drawContours(redF, cntRed, -1, (255,0,255), 3)
    greenF = cv2.drawContours(greenF, cntGreen, -1, (255,0,255), 3)
    
    for cnt in cntRed:
#        if is_button_shape(cnt):
#            print "is red button shape"
        if is_button(cnt):
            print "is red button"
            epsilon= 0.04 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            redF =  cv2.drawContours(redF,[approx],-1, (255, 255, 0), 3)
        else:
            print "is red block"
            
    for cnt in cntGreen:
#        if is_button_shape(cnt):
#            print "is green button shape"
        if is_button(cnt):
            print "is green button"
        else:
            print "is green block"
    
    # Display the resulting frame
    cv2.imshow("Original", frame)
    cv2.imshow("Red Filtered", redF)
    cv2.imshow("Green Filtered", greenF)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    


#frame2 = copy.copy(frame)
#
#b,g,r = cv2.split(frame2)
#frame2[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
#frame2[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
#frame2[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
#frame3 = copy.copy(frame2)
#imgray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
#ret,thresh = cv2.threshold(imgray,50,255,0)
#image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#
#realContours = []
#height, width, channels = frame2.shape
#minArea  = 0.01*height*width
##    minArea = 0
#biggestCnt = None
#biggestArea = 0.0
#
#for cnt in contours:
#    cntArea = cv2.contourArea(cnt)
#    if cntArea>=minArea:
#        if cntArea>biggestArea:
#            biggestCnt = cnt
#            biggestArea = cntArea
#        realContours.append(cnt)
##        M = cv2.moments(cnt)
##        if M['m00']!=0:
##            Cx = int(M['m10']/M['m00'])
##            Cy = int(M['m01']/M['m00'])
##        else:
##            Cx,Cy=0,0
##        print Cx-width/2,Cy
##        print "angle:", (60.0*Cx/width - 30)
##        frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
##            distance = calculateDistanceOfBlock(cnt,height)
##            print "Distance:",distance
#    else:
#        frame2 = cv2.drawContours(frame2,[cnt],0,[0,0,0],-1)
##            pass
#
##    shapes = []
##    for cnt in realContours:
##        epsilon= 0.05 * cv2.arcLength(cnt, True)
##        approx = cv2.approxPolyDP(cnt, epsilon, True)
##        shapes.append(approx)
#print "area: ", biggestArea
#if biggestArea > 0:
#
#    #if is_button(biggestCnt):
#    if is_button_shape(biggestCnt):
#        print "There is a button in front of you"
#    else:
#        print "There is a block"
#
#frame2 = cv2.drawContours(frame2, [biggestCnt], -1, (255,0,255), 3)
##    frame2 = cv2.drawContours(frame2, realContours, -1, (255,0,255), 3)
##frame3 = cv2.drawContours(frame3, [approx], -1, (255, 0, 255), 3)
#Cx, Cy =  get_center_of_mass(biggestCnt)
#print "center of mass:", Cx, Cy
#if get_angle(Cx, width) < 0:
#    direction = "left "
#else:
#    direction = "right "
#print "turn " + direction + str(abs(get_angle(Cx, width))) + "degrees."
#frame2[Cy-7:Cy+7,Cx-7:Cx+7] = [255,0,0]
#
#
#if biggestArea != 0.0:
##        distance = calculateDistanceOfBlock(biggestCnt,height)
#    v = 100*biggestArea/(height*width)
#    v=1/v
#    dist = -25.452*(v**2) + 51.157*v - 0.7276
#    #print "Distance:",dist
#
## Our operations on the frame come here
##gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#
## Display the resulting frame
#cv2.imshow("Original", frame)
#cv2.imshow("Filtered", frame2)
#cv2.imshow("approximated", frame3)
#
##if cv2.waitKey(1) & 0xFF == ord('q'):
##    # When everything done, release the capture
##    cv2.destroyAllWindows()
#cv2.waitKey(0)
#cv2.destroyAllWindows()
