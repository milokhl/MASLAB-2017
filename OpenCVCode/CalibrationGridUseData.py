# -*- coding: utf-8 -*-
"""
Created on Sat Jan 21 06:39:02 2017

@author: arinz
"""

#import numpy as np
#import cv2
#import glob
### termination criteria
##criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
### prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
##objp = np.zeros((9*6,3), np.float32)
##objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
### Arrays to store object points and image points from all the images.
##objpoints = [] # 3d point in real world space
##imgpoints = [] # 2d points in image plane.
###images = glob.glob('*.jpg')
###for fname in images:
#cap = cv2.VideoCapture(-1)
#
#while(True):
#    ret, frame = cap.read()
#    if ret:
#        cv2.imshow('original',frame)
#
##    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
##    # Find the chess board corners
##    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
##    # If found, add object points, image points (after refining them)
##    if ret == True:
##        objpoints.append(objp)
##        corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
##        imgpoints.append(corners)
##        # Draw and display the corners
##        cv2.drawChessboardCorners(img, (9,6), corners2, ret)
##        cv2.imshow('img', img)
###        cv2.waitKey(5000)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
#cv2.destroyAllWindows()

import numpy as np
import cv2
import time
import copy

cap = cv2.VideoCapture(0)
#cap.set(3,100)
#cap.set(4,60)

count = 0
start = time.time()

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Load previously saved data
with np.load('webcam_calibration_ouput.npz') as X:
    mtx, dist, rvecs, tvecs = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

def draw(img, corners, imgpts):
    x = np.array([[ 174.78126526,  449.86920166]], dtype=np.float32)
#    corner = tuple(corners[0].ravel())
    corner = tuple(x.ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
corners2 = np.array([[[ 174.78126526,  449.86920166]],

       [[ 180.68838501,  411.61248779]],

       [[ 186.4058075 ,  375.30682373]],

       [[ 192.25161743,  340.79598999]],

       [[ 197.33950806,  308.27828979]],

       [[ 202.1191864 ,  276.70681763]],

       [[ 206.54852295,  246.63311768]],

       [[ 210.68284607,  217.9733429 ]],

       [[ 214.64544678,  190.56147766]],

       [[ 209.19444275,  451.006073  ]],

       [[ 215.20358276,  412.57870483]],

       [[ 220.1778717 ,  375.75537109]],

       [[ 225.06884766,  340.81341553]],

       [[ 229.42881775,  307.76757812]],

       [[ 233.70202637,  276.46643066]],

       [[ 237.6933136 ,  246.41503906]],

       [[ 241.43412781,  217.83070374]],

       [[ 244.78172302,  190.21517944]],

       [[ 244.33444214,  451.88696289]],

       [[ 249.51321411,  412.88757324]],

       [[ 254.32150269,  376.28033447]],

       [[ 258.38687134,  340.77438354]],

       [[ 262.26409912,  307.63122559]],

       [[ 265.71853638,  276.05856323]],

       [[ 269.15875244,  245.63729858]],

       [[ 271.9671936 ,  216.58605957]],

       [[ 274.3225708 ,  187.7052002 ]],

       [[ 280.54129028,  452.85510254]],

       [[ 285.05737305,  413.50820923]],

       [[ 288.8973999 ,  376.18432617]],

       [[ 292.46829224,  340.73657227]],

       [[ 295.52041626,  307.25415039]],

       [[ 298.5050354 ,  275.38238525]],

       [[ 301.1763916 ,  244.75505066]],

       [[ 303.31243896,  215.33203125]],

       [[ 305.11956787,  186.42483521]],

       [[ 318.12060547,  454.19696045]],

       [[ 321.41619873,  413.99008179]],

       [[ 324.35238647,  376.32821655]],

       [[ 326.92938232,  340.38751221]],

       [[ 329.38226318,  306.58981323]],

       [[ 331.47247314,  274.46914673]],

       [[ 333.43676758,  243.51921082]],

       [[ 335.07156372,  213.86709595]],

       [[ 336.60073853,  185.26542664]],

       [[ 356.6774292 ,  455.45166016]],

       [[ 358.8260498 ,  414.55752563]],

       [[ 360.65103149,  376.13134766]],

       [[ 362.25476074,  339.51034546]],

       [[ 363.66763306,  305.19927979]],

       [[ 364.91516113,  272.58056641]],

       [[ 366.26617432,  241.47494507]],

       [[ 367.45495605,  212.38356018]],

       [[ 368.39984131,  183.56632996]]], dtype=np.float32)

while(True):
    if (time.time()-start)>=3:
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame2 = copy.copy(frame)
    # project 3D points to image plane
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
    frame2 = draw(frame2,corners2,imgpts)

#    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv2.imshow("Original", frame)
    cv2.imshow("Grayscale", frame2)
    
    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
