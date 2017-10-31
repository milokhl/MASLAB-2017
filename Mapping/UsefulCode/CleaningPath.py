# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 13:28:12 2017

@author: arinz
"""

def cleanPath(path,allSegments):
    start = 0
    end = 2
    path.reverse()
    while end<len(path)-1: 
        p1 = path[start]
        p2 = path[end]
        seg = (p1[0],p1[1],p2[0],p2[1])
        if isClearPath(seg,allSegments):
            path.pop(end-1)
        else:
            start += 1
            end += 1
    path.reverse()
    return path
        
def isClearPath(SEG, allSegments, tolerance = tol):
    for segment in allSegments:
        if SD.segments_distance(SEG, segment)<= tolerance:
            return False
    return True
    
def getPathLength(path):
    #a path is a list of points
    distance = 0.0
    for i in range(len(path)-1):
        distance += calculatHeuristic(path[i],path[i+1])
    return distance