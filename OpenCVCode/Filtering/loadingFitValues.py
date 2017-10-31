# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 15:27:50 2017

@author: arinz
"""
import numpy as np
import cv2

# Load previously saved data
with np.load('button_contours.npz') as X:
    buttonContours = [X[i] for i in ('buttonCnt')]
    
# Load previously saved data
with np.load('block_contours.npz') as X:
    blockContours = [X[i] for i in ('blockCnt')]
    
    
def is_button(cnt1):
    blockScores = []
    buttonScores = []
    
    for cnt2 in blockContours:
        ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
        blockScores.append(ret)
        
    for cnt2 in buttonContours:
        ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
        buttonScores.append(ret)
        
    bestBlock= min(blockScores)
    bestButton = min(buttonScores)
    
    if bestBlock<bestButton:
        print "Block Found"
        return False
    else:
        print "Button Found"
        return True
    
    