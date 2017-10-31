# -*- coding: utf-8 -*-
"""
Created on Mon Jan 23 20:48:41 2017

@author: arinz
"""

import cv2
import numpy as np
import copy

rg = 1.7
rb = 1.7
gr = 1.15
gb = 1.0

img2 = cv2.imread('blocks_3.jpg')
img2a = copy.copy(img2)
b,g,r = cv2.split(img2)
img2a[((r>(rg*g)) & (r>(rb*b)))] = [0,0,255]
img2a[((g>(gr*r)) & (g>(gb*b)))] = [0,255,0]
img2a[~(((g>(gr*r)) & (g>(gb*b)))|((r>(rg*g)) & (r>(rb*b))))] = [0,0,0]
#img2a = cv2.cvtColor(img2a,cv2.COLOR_BGR2GRAY)

cv2.imshow("original", img2)
cv2.imshow("filtered", img2a)

#if cv2.waitKey(1) & 0xFF == ord('q'):
#    # When everything done, release the capture
#    cv2.destroyAllWindows()
cv2.waitKey(0)
cv2.destroyAllWindows()