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


img1 = cv2.imread('red_button.jpg')

b,g,r = cv2.split(img1)
img1[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
img1[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
img1[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)


img2 = cv2.imread('green_button.jpg')
b,g,r = cv2.split(img2)
img2[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
img2[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
img2[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)


ret, thresh = cv2.threshold(img1, 50, 255,0)
ret, thresh2 = cv2.threshold(img2, 50, 255,0)
image, contours,hierarchy = cv2.findContours(thresh,2,1)

biggestCnt = contours[0]
biggestArea = cv2.contourArea(contours[0])

for cnt in contours:
    if cv2.contourArea(cnt)>biggestArea:
        biggestArea = cv2.contourArea(cnt)
        biggestCnt = cnt

cnt1 = biggestCnt
np.savez("button_contour", cnt = cnt1)



img1 = cv2.drawContours(img1, [biggestCnt], -1, (255,0,255), 3)

image, contours1,hierarchy = cv2.findContours(thresh2,2,1)


biggestCnt1 = contours1[0]
biggestArea1 = cv2.contourArea(contours1[0])

for cnt in contours1:
    if cv2.contourArea(cnt)>biggestArea1:
        biggestArea1 = cv2.contourArea(cnt)
        biggestCnt1 = cnt

cnt2 = biggestCnt1
img2 = cv2.drawContours(img2, [biggestCnt1], -1, (255,0,255), 3)
ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
print ret




cv2.imshow("red button", img1)
cv2.imshow("green button", img2)

#if cv2.waitKey(1) & 0xFF == ord('q'):
#    # When everything done, release the capture
#    cv2.destroyAllWindows()
cv2.waitKey(0)
cv2.destroyAllWindows()
