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
#        print line
        
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

        
    
parseFile('PracticeMap.txt')