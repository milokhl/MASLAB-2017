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
shouldSave = False
while(True):
    if (time.time()-start)>=3:
        
        print count/(time.time()-start)
        count=0
        start = time.time()
        
    # Capture frame-by-frame
    ret, frame = cap.read()

    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(frame2, (9,6), None)
    frame3 = copy.copy(frame)
    print ret
    if ret == True:
        objpoints.append(objp)
        corners2=cv2.cornerSubPix(frame2, corners, (11,11), (-1,-1), criteria)
        print corners2
        print type(corners2)
        print corners2[0]
        break
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(frame3, (9,6), corners2, ret)


        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame2.shape[::-1], None, None)
        if shouldSave:
            np.savez("webcam_calibration_ouput.txt", ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
            shouldSave = False
#        h,  w = frame.shape[:2]
#        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
#        
#        # undistort
#        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
#        
##        # undistort
##        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
##        dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
#        
#        # crop the image
#        x, y, w, h = roi
#        dst = dst[y:y+h, x:x+w]
#        cv2.imshow('Distort', dst)
        
    # Display the resulting frame
    cv2.imshow("Original", frame)
    cv2.imshow("Grayscale", frame2)
    cv2.imshow('Found Points', frame3)
    
    count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
