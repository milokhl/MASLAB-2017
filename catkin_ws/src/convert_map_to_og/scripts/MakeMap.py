#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Sun Jan 29 11:29:38 2017

@author: Arinze Jasmine
^^^ Thanks guys!!!
"""

	
#from vectors2d import *
import numpy as np, vectors2d as vec
import time
import cv2
    
def makeMap(mapFileName,factor=1.0,clearance = 1.0):
    """
    returns a map enlarged by factor (should be 24)
    """    
    walls,greenDispensors,redDispensor,homeBase,stacks,startingPosition, maxX, maxY = parseFile(mapFileName,factor)
    
    #Calculate max x and y from walls and dispensors
#    maxX = 9 * factor
#    maxY = 10 * factor
#    def transform(x):
#        
#        return x * factor
#    
    #use walls and dispensors (and blocks maybe) to input obstacles in list of lines
    boundaries = walls + [greenDispensors] + [redDispensor]
    dispensors = [greenDispensors] + [redDispensor]
#    print boundaries

        
    print "grid size:", maxX, maxY
    
    boundaryGrid = np.zeros([maxY,maxX])
    dispensorGrid = np.zeros([maxY,maxX])
    baseGrid = np.zeros([maxY,maxX])
    stackGrid = np.zeros([maxY,maxX])
    
#   fill in grids
    for x in range(maxX):
        for y in range(maxY):
            for line in boundaries:
                if vec.pnt2line([x,y], line[0], line[1])[0]<=clearance:
                    boundaryGrid[maxY-1-y][x] = 1
                    break
                
    for x in range(maxX):
        for y in range(maxY):
            for line in dispensors:
                if vec.pnt2line([x,y], line[0], line[1])[0]<=clearance:
                    dispensorGrid[maxY-1-y][x] = 1
                    break
    
    for x in range(maxX):
        for y in range(maxY):
            for hb in range(len(homeBase)):
                try:
                    isClear = (vec.pnt2line([x,y], homeBase[hb], homeBase[hb+1])[0]<=clearance)
                except IndexError:
                    isClear = (vec.pnt2line([x,y], homeBase[hb], homeBase[0])[0]<=clearance)
                if isClear:
                    baseGrid[maxY-1-y][x] = 1
                    break
                
    for x in range(maxX):
        for y in range(maxY):
            for sk in range(len(stacks)):
                if vec.pnt2line([x,y], stacks[sk], stacks[sk])[0]<=clearance:
                    stackGrid[maxY-1-y][x] = 1
                    break
    
                
    print boundaryGrid
    print "---------------------"
    print dispensorGrid
    print "---------------------"
    print baseGrid
    print "---------------------"
    print stackGrid
    print "---------------------"
    return boundaryGrid,dispensorGrid,baseGrid,stackGrid,walls,greenDispensors,redDispensor,homeBase,stacks,startingPosition, maxX, maxY

def parseFile(mapFileName,scale=1):
    """
    parses a mapfile, type(mapFileName) == str
    for walls and home base, return a list of line segments, which are lists of two coordinates, [x,y]
    for stacks,return a list of coordinates, [x, y]
    for startPos, return [x, y, heading], heading is a string 'N', 'E', 'S', 'W'
    for dispensers, return two coordinates[x, y]
    
    """
    f = open(mapFileName, 'r') 

    allXValues = []
    allYValues = []
    
    walls = []
    dispenser_g = []
    dispenser_r = []
    homeBase = []
    stacks = []
    startPos = []
    
    for line in f:
    
        item = line.split(',')
        print line
        
        if item[0] == 'W':
            coordinate = []
            x1, y1 = scale*int(item[1]), scale*int(item[2])
            x2, y2 = scale*int(item[3]), scale*int(item[4][0])
            coordinate.append((x1, y1))
            coordinate.append((x2, y2))
            allXValues.append(x1)
            allXValues.append(x2)
            allYValues.append(y1)
            allYValues.append(y2)
    
            walls.append(coordinate)
        elif item[0] == 'D':
            x1, y1 = scale*int(item[1]), scale*int(item[2])
            x2, y2 = scale*int(item[3]), scale*int(item[4])

            allXValues.append(x1)
            allXValues.append(x2)
            allYValues.append(y1)
            allYValues.append(y2)
#            coordinate.append([x1, y1])
#            coordinate.append([x2, y2])
            if item[len(item) -1][0] == 'R':
                dispenser_r.append((x1, y1))
                dispenser_r.append((x2, y2))
            else:
                dispenser_g.append((x1, y1))
                dispenser_g.append((x2, y2))
        elif item[0] == 'H':
            for i in range(2, len(item),2):
                x1, y1 = scale*int(item[i]), scale*int(item[i +1])
                allXValues.append(x1)
                allYValues.append(y1)
                homeBase.append((x1, y1))
        elif item[0] == 'S':
            x1, y1 = scale*int(item[1]), scale*int(item[2])
            allXValues.append(x1)
            allYValues.append(y1)
            stacks.append((x1, y1))
    
        elif item[0] == 'L':
            x1,y1 = scale*int(item[1]),scale*int(item[2])
            startPos.append((x1,y1))
            startPos.append(item[3][0])        

            allXValues.append(x1)
            allYValues.append(y1)
            
    minX = int(min(allXValues)*0.9)
    minY = int(min(allYValues)*0.9)
    maxX = int(max(allXValues) * 1.1)
    maxY = int(max(allYValues) * 1.1)
    maxX = maxX-minX
    maxY = maxY-minY

    f.close()
    
#    print "walls:", walls
#    print "dispensor_green:", dispenser_g
#    print "dispensor_red:", dispenser_r
#    print "home base:", homeBase
#    print "stacks:" , stacks
#    print "starting position:", startPos

    walls, dispenser_g, dispenser_r, homeBase, stacks, startPos = shiftPoints(walls, dispenser_g, dispenser_r, homeBase, stacks, startPos,minX,minY)

#    print "walls:", walls
#    print "dispensor_green:", dispenser_g
#    print "dispensor_red:", dispenser_r
#    print "home base:", homeBase
#    print "stacks:" , stacks
#    print "starting position:", startPos    
    
    return walls, dispenser_g, dispenser_r, homeBase, stacks, startPos, maxX, maxY
    
   
def shiftPoints(walls, dispenser_g, dispenser_r, homeBase, stacks, startPos,minX,minY):
    
    walls2 = []
    for w in walls:
        p1 = (w[0][0]-minX,w[0][1]-minY)
        p2 = (w[1][0]-minX,w[1][1]-minY)
        walls2.append([p1,p2])
    
    dispenser_g2 = []
    dispenser_r2 = []
    
    for p in dispenser_g:
        dispenser_g2.append((p[0]-minX,p[1]-minY))
    for p in dispenser_r:
        dispenser_r2.append((p[0]-minX,p[1]-minY))

    homeBase2 = []
    for p in homeBase:
        homeBase2.append((p[0]-minX,p[1]-minY))
        
    stacks2 = []
    for p in stacks:
        stacks2.append((p[0]-minX,p[1]-minY))
        
    startPos2 = [(startPos[0][0]-minX,startPos[0][1]-minY),startPos[1]]
    
    return  walls2, dispenser_g2, dispenser_r2, homeBase2, stacks2, startPos2

# def displayMap(boundaryGrid,dispensorGrid,baseGrid,stackGrid,walls,greenDispensors,redDispensor,homeBase,stacks,startingPosition, maxX, maxY):
    
#     mapImage = 255*np.ones([maxY,maxX,3])
#     mapImage[boundaryGrid==1] = [255,0,0]
#     mapImage[dispensorGrid==1] = [0,255,255]
#     mapImage[baseGrid==1] = [255,0,255]
#     mapImage[stackGrid==1] = [0,0,255]
#     cv2.imshow("map",mapImage)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    

# if __name__ == "__main__":
#     mapFileName = "PracticeMap.txt"
#     factor = 24.0
#     clearance = 2.0
#     boundaryGrid,dispensorGrid,baseGrid,stackGrid,walls,greenDispensors,redDispensor,homeBase,stacks,startingPosition, maxX, maxY = makeMap(mapFileName, factor, clearance)
#     displayMap(boundaryGrid,dispensorGrid,baseGrid,stackGrid,walls,greenDispensors,redDispensor,homeBase,stacks,startingPosition, maxX, maxY)