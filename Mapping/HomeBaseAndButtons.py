# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 02:16:28 2017

@author: arinz
"""
import numpy as np
import vectors2d as vec
import math
# determine if a point is inside a given polygon or not
# Polygon is a list of (x,y) pairs.

def point_inside_polygon(x,y,poly):
    #Include points on the boundary
    if (x,y) in poly:
        return True
    
    n = len(poly)
    inside =False
    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside
    



def getButtonCoord(A,B,distance):
    #Get the point at that distance from the midpoint
    A = np.array(A)
    B = np.array(B)
    M = (A+B)/2.0
    p = (A-B)
    n = np.array([-p[1],p[0]])
    norm_length = int(np.sqrt((n[0] * n[0]) + (n[1] * n[1])))
    n[0] /= norm_length
    n[1] /= norm_length

    result = (M + (distance * n))
#    print result
    #get the angle from the point to the line segment
    
    ang_rad = math.atan2(M[1] - result[1],M[0] - result[0])
    ang_deg = math.degrees(ang_rad)
    return result,ang_deg

#The two points beside the buttons and the angle they need to face the button
A = np.array((6,9))
B = np.array((6,8))
C = np.array((6,7))
D = np.array((6,6))
distanceFromButton = 1
possiblePointR1,angR1 = getButtonCoord(A,B,distanceFromButton)
possiblePointR2,angR2 = getButtonCoord(A,B,-distanceFromButton)
possiblePointG1,angG1 = getButtonCoord(C,D,distanceFromButton)
possiblePointG2,angG2 = getButtonCoord(C,D,-distanceFromButton)
print possiblePointR1,angR1
print possiblePointR2,angR2
print possiblePointG1,angG1
print possiblePointG2,angG2

#Getting points in the home base
homebase = [(6,5),(7,5),(7,4),(7,3),(6,3),(5,3),(5,4)]
grid = np.zeros([10,10])
for y in range(10):
    for x in range(10):
        if (point_inside_polygon(x,y,homebase)):
            grid[9-y][x]=1
print grid
#must perform and with the not of coords in the boundary grid to get free squares in base