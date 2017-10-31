# -*- coding: utf-8 -*-
"""
Created on Mon Jan 23 21:42:00 2017

@author: arinz
"""

import cv2
import numpy as np
import copy

rg = 1.7
rb = 1.7
gr = 1.15
gb = 1.0

buttonImages = ['button1.jpg','button2.jpg','button3.jpg','button4.jpg','button5.jpg','button6.jpg','button7.jpg','button8.jpg','button9.jpg','button10.jpg','button11.jpg']
#buttonImages = ['button1.jpg','button2.jpg','button3.jpg','button4.jpg','button7.jpg','button9.jpg']
blockImages = ['block1.jpg','block2.jpg','block3.jpg','block4.jpg','block5.jpg','block6.jpg','block7.jpg','block8.jpg','block9.jpg','block10.jpg','block11.jpg','block12.jpg']
buttonContours = []
blockContours = []

def detect(c):
    # initialize the shape name and approximate the contour
    shape = "unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.005 * peri, True)
    print "approx", len(approx)
    if len(approx) <= 13:
    	# compute the bounding box of the contour and use the
    	# bounding box to compute the aspect ratio
        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        shape = False
        # otherwise, we assume the shape is a circle
    else:
        shape = True

        # return the name of the shape
    return shape

def is_button4(cnt):
    #perimeter = cv2.arcLength(cnt,True)
    area = cv2.contourArea(cnt)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = float(area)/hull_area
    print area,hull_area
    print "solidity",solidity
    if (solidity > .94 and solidity < .99):
        return True
    return False

for fn in buttonImages:
    img1 = cv2.imread(fn)
    b,g,r = cv2.split(img1)
    img1[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    img1[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    img1[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
    img2 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img2, 50, 255,0)
    image, contours,hierarchy = cv2.findContours(thresh,2,1)
    sortedCnts = sorted(contours,key=cv2.contourArea)
    sortedCnts.reverse()
    biggestCnt = sortedCnts[0]
    buttonContours.append(biggestCnt)
    cv2.drawContours(img1, [biggestCnt], -1, (255,0,255), 3)
    isButton = detect(biggestCnt)
    print fn,isButton
    print
#    cv2.imshow(fn, img1)
#    cv2.waitKey(0)
#    cv2.destroyAllWindows()

for fn in blockImages:
    img1 = cv2.imread(fn)
    b,g,r = cv2.split(img1)
    frameR = copy.copy(img1)
    frameR[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    frameR[~(((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
    frameG = copy.copy(img1)
    frameG[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    frameG[~(((g>(gr*r)) & (g>(gb*b))))] = [0,0,0]
    # img1[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
    # img1[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
    # img1[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
    img2 = cv2.cvtColor(frameG,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img2, 50, 255,0)
    image, contours,hierarchy = cv2.findContours(thresh,2,1)
    sortedCnts = sorted(contours,key=cv2.contourArea)
    sortedCnts.reverse()
    try:
        biggestCnt = sortedCnts[0]
    except IndexError:
        continue
    blockContours.append(biggestCnt)
    cv2.drawContours(img1, [biggestCnt], -1, (255,0,255), 3)
    isButton = detect(biggestCnt)
    print fn,isButton
    print
#    cv2.imshow(fn, img1)
#    cv2.waitKey(0)
#    cv2.destroyAllWindows()

#np.savez("button_contours", buttonCnt = buttonContours)
#np.savez("block_contours", blockCnt = blockContours)
inputs = np.zeros([15,4])
index = 0
outputs = np.zeros([15,1])
print "buttons"
for cnt in buttonContours:
    numberOfPoints = len(cnt)
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt,True)

#    rect = cv2.minAreaRect(cnt)
#    box = cv2.boxPoints(rect)
#    box = np.int0(box)
#    boxArea = cv2.contourArea(box)
#    inputs [index][:] = [numberOfPoints,area,perimeter,boxArea]
#    outputs[index] = 1
#    index += 1

#    (x,y),radius = cv2.minEnclosingCircle(cnt)
#    print area/(np.pi*radius*radius)

    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    print "area/approx:",(100*area/cv2.contourArea(box))**2


#    print "numberOfPoints:",numberOfPoints
#    print "area",area
#    print "perimeter",perimeter

#    epsilon = 0.05*cv2.arcLength(cnt,True)
#    approx = cv2.approxPolyDP(cnt,epsilon,True)
#    print "approx area",cv2.contourArea(approx)
#    try:
#        print "area/approx:",area/cv2.contourArea(approx)
#    except ZeroDivisionError:
#        print "ZDE"

#    areaOld = cv2.contourArea(cnt)
#    epsilon= 0.05 * cv2.arcLength(cnt, True)
#    approx = cv2.approxPolyDP(cnt, epsilon, True)
#    a = cv2.contourArea(approx)
#    try:
#        ratio = float(areaOld)/a
#        print ratio
#    except ZeroDivisionError:
#        print "ZDE"


print "blocks"
for cnt in blockContours:
    numberOfPoints = len(cnt)
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt,True)

#    rect = cv2.minAreaRect(cnt)
#    box = cv2.boxPoints(rect)
#    box = np.int0(box)
#    boxArea = cv2.contourArea(box)
#    inputs [index][:] = [numberOfPoints,area,perimeter,boxArea]
#    outputs[index] = 0
#    index += 1

#    (x,y),radius = cv2.minEnclosingCircle(cnt)
#    print area/(np.pi*radius*radius)

    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    print "area/approx:",(100*area/cv2.contourArea(box))**2

#    print "numberOfPoints:",numberOfPoints
#    print "area",area
#    print "perimeter",perimeter


#    epsilon = 0.05*cv2.arcLength(cnt,True)
#    approx = cv2.approxPolyDP(cnt,epsilon,True)
#    print "approx area",cv2.contourArea(approx)
#    try:
#        print "area/approx:",area/cv2.contourArea(approx)
#    except ZeroDivisionError:
#        print "ZDE"

#    areaOld = cv2.contourArea(cnt)
#    epsilon= 0.05 * cv2.arcLength(cnt, True)
#    approx = cv2.approxPolyDP(cnt, epsilon, True)
#    a = cv2.contourArea(approx)
#    try:
#        ratio = float(areaOld)/a
#        print ratio
#    except ZeroDivisionError:
#        print "ZDE"

#print inputs
#print outputs
