# -*- coding: utf-8 -*-
"""
Created on Sun Jan 29 11:29:38 2017

@author: Arinze Jasmine
"""

	
from vectors2d import *
import numpy as np
import time
  

    
def makeMap(mapFileName):
    walls,greenDispensors,redDispensor,homeBase,stacks,startingPosition = parseFile(mapFileName)
    
    clearance = 1.0
    #Calculate max x and y from walls and dispensors
    maxX = 0
    maxY = 0
    
    #use walls and dispensors (and blocks maybe) to input obstacles in list of lines
    obstacles = walls + greenDispensors+redDispensor
    for ob in obstacles:
        xMax = max(xMax,ob[0][0],ob[1][0])
        yMax = max(yMax,ob[0][1],ob[1][1])
    
    for coord in homeBase:
        xMax = max(xMax,ob[0][0],ob[1][0])
        yMax = max(yMax,ob[0][1],ob[1][1])
        
    for s in stacks:
        obstacles.append([s,s])
        xMax = max(xMax,ob[0],ob[1][0])
        yMax = max(yMax,ob[1])
        

    

#    startcoord = [2,3,0]
#    endcoord = [13,12,0]
#    startcoord1 = [2,3,0]
#    endcoord1 = [13,12,0]
#    lines = [[startcoord,endcoord],[startcoord1,endcoord1]]

    #use obstacles to fill in grid
    grid = np.zeros([maxY,maxX])
    for x in range(maxX):
        for y in range(maxY):
            for line in obstacles:
                if pnt2line([x,y], line[0:2]), line[2:4])[0]<=clearance:
                    grid[maxY-1-y][x] = 1
                    break
                
#    print grid
    return gridWithBlocks, gridNoBlocks

def parseFile(mapFileName):
    """
    parses a mapfile, type(mapFileName) == str
    for walls and home base, return a list of line segments, which are lists of two coordinates, [x,y]
    for stacks,return a list of coordinates, [x, y]
    for startPos, return [x, y, heading], heading is a string 'N', 'E', 'S', 'W'
    for dispensers, return two coordinates[x, y]
    
    """
    f = open(mapFileName, 'r')
    
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
            x1, y1 = int(item[1]), int(item[2])
            x2, y2 = int(item[3]), int(item[4][0])
            coordinate.append([x1, y1])
            coordinate.append([x2, y2])
    
            walls.append(coordinate)
        elif item[0] == 'D':
            x1, y1 = int(item[1]), int(item[2])
            x2, y2 = int(item[3]), int(item[4])
#            coordinate.append([x1, y1])
#            coordinate.append([x2, y2])
            if item[len(item) -1][0] == 'R':
                dispenser_r.append([x1, y1])
                dispenser_r.append([x2, y2])
            else:
                dispenser_g.append([x1, y1])
                dispenser_g.append([x2, y2])
        elif item[0] == 'H':
            for i in range(1, len(item)-3, 2):
                coord = []
                x1, y1 = int(item[i]), int(item[i +1])
                x2, y2 = int(item[i + 2]), int(item[i + 3][0])
                coord.append([x1, y1])
                coord.append([x2, y2])
                homeBase.append(coord)
        elif item[0] == 'S':
            x1, y1 = int(item[1]), int(item[2])
            stacks.append([x1, y1])
    
        elif item[0] == 'L':
            startPos.append(int(item[1]))
            startPos.append(int(item[2]))
            startPos.append(item[3][0])            
                
#    print "walls:", walls
#    print "dispensor_green:", dispenser_g
#    print "dispensor_red:", dispenser_r
#    print "home base:", homeBase
#    print "stacks:" , stacks
#    print "starting position:", startPos
    
    return walls, dispenser_g, dispenser_r, homeBase, stacks, startPos
    
    f.close()


#if __name__ == "__main__":
#    mapFileName = "PracticeMap.txt"
#    makeMap(mapFileName) 
