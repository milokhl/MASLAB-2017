# -*- coding: utf-8 -*-
"""
Created on Tue Jan 24 14:40:14 2017

@author: arinz
"""
import numpy as np
import cv2
import time


# Load previously saved data
with np.load("xbutton_contours.npz") as X:
    buttonContours = X['buttonCnt']
    
# Load previously saved data
with np.load('xblock_contours.npz') as X:
    blockContours = X['blockCnt']
    
def is_button7(c):
    # initialize the shape name and approximate the contour
    shape = "unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.005 * peri, True)
    approx2 = cv2.approxPolyDP(c, 0.01 * peri, True)
    approx3 = cv2.approxPolyDP(c, 0.005 * peri, True)
    approx4 = cv2.approxPolyDP(c, 0.001 * peri, True)
    approx5 = cv2.approxPolyDP(c, 0.0005 * peri, True)
    print "approx", len(approx)
    print "approx2", len(approx2)
    print "approx3", len(approx3)
    print "approx4", len(approx4)
    print "approx5", len(approx5)
    if len(approx) <= 13:
    	# compute the bounding box of the contour and use the
    	# bounding box to compute the aspect ratio
        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        shape = False
        print "Found Block!"
        # otherwise, we assume the shape is a circle
    else:
        shape = True
        print "Found Button!"
        # return the name of the shape
    return shape

def is_button6(cnt):
    epsilon= 0.02 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    a = cv2.contourArea(approx)
    rect= cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    ratio = (a/cv2.contourArea(box))**2 * 100
    print "ratio:", ratio
    if ratio > 10:
        return False
    else:
        return True
    

def is_button5(cnt):
    area = cv2.contourArea(cnt)
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    ratio = (100*area/cv2.contourArea(box))**2
    print "area/approx:",ratio
    if (100*area/cv2.contourArea(box))**2<6150:
        return True
    return False

def is_button4(cnt):
    #perimeter = cv2.arcLength(cnt,True)
    area = cv2.contourArea(cnt)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = float(area)/hull_area
    print area,hull_area
    print "solidity",solidity
    if not(solidity > .94 and solidity < .99):
        return True
    return False

def is_button3(cnt):
    approx = cv2.approxPolyDP(cnt,0.03*cv2.arcLength(cnt,True),True)
    area = cv2.contourArea(cnt)
    print approx,area
    if ((len(approx) > 8) & (len(approx) < 23) & (area > 30) ):
        return True
    return False
    
def is_button2(cnt1):
    blockScores = []
    buttonScores = []
    for i in range(len(blockContours)):
        cnt2 = blockContours[i]
        ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
#        print "block score:",ret,"file:",(i+1)
        blockScores.append(ret)
        
    for i in range(len(buttonContours)):
        cnt2 = buttonContours[i]
        ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
        buttonScores.append(ret)
#        print "button score:",ret,"file:",(i+1)
        
    bestBlock= np.mean(blockScores)
    bestButton = np.mean(buttonScores)
    print bestButton
    if bestBlock<bestButton:
#        print "Block Found"
        return False
    else:
#        print "Button Found"
        return True

def is_button(cnt, threshold = 1.1):
    """
    cnt: a contour, threshold: a float, for the minimum ratio needed for a contour to be a circle
    return: True if the contour is a circle, False otherwise
    """
    areaOld = cv2.contourArea(cnt)
    epsilon= 0.05 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    a = cv2.contourArea(approx)
    
    try:
        ratio = float(areaOld)/a
    except ZeroDivisionError:
        print "ZDE"
        return True
    # print ratio
    print ratio
    return ratio > threshold


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
    
    v = (height*width)/(10*cv2.contourArea(cnt))
    dist = -0.0429*(v**2) + 2.3863*v + 12.291
    return dist

def filter_contours(frame,height,width,ratio = 0.01):
    """
    returns real contours, sorted with the biggest contour as the first element
    """
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

rg = 1.2
rb = 1.7
gr = 1.15
gb = 1.05

cap = cv2.VideoCapture(0)
#cap.set(3,100)
#cap.set(4,60)
#cap.set(15, -4.0)

count = 0
start = time.time()

while True:
    if (time.time()-start)>=3:
        
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print "No camera found"
        continue

    height, width, channels = frame.shape
    redF,greenF = filter_color(frame,height,width)
    cntRed = filter_contours(redF,height,width,ratio = 0.005)
    cntGreen = filter_contours(greenF,height,width,ratio = 0.005)
    
    allCnt = []
    for cnt in cntRed:
        allCnt.append(cnt)
    for cnt in cntGreen:
        allCnt.append(cnt)
    allCnt = sorted(allCnt,key = cv2.contourArea)
    allCnt.reverse()
    
    redF = cv2.drawContours(redF, cntRed, -1, (255,0,255), 3)
    greenF = cv2.drawContours(greenF, cntGreen, -1, (255,0,255), 3)
    try:
        cntRed = [cntRed[0]]
    except:
        cntRed = []
    try:
        cntGreen = [cntGreen[0]]
    except:
        cntGreen = []
        
    for cnt in cntRed:
#        if is_button_shape(cnt):
#            print "is red button shape"
#        print is_button2(cnt)
        if is_button5(cnt):
#            pass
            print "is red button"
        else:
            print "is red block"
         #   print "distance:",get_distance(cnt,height,width),"inches"
         #   print "angle:",get_angle(get_center_of_mass(cnt)[0], width),"degrees"
            
    for cnt in cntGreen:
#        if is_button_shape(cnt):
#            print "is green button shape"
#        print is_button2(cnt)
        if is_button5(cnt):
#            pass
            print "is green button"
        else:
            print "is green block"
            print "distance:",get_distance(cnt,height,width),"inches"
            print "angle:",get_angle(get_center_of_mass(cnt)[0], width),"degrees"
    if len(allCnt)>0:
        print "-----------------------"
    
    # Display the resulting frame
    cv2.imshow("Original", frame)
    cv2.imshow("Red Filtered", redF)
    cv2.imshow("Green Filtered", greenF)

    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
