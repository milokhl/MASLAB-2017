import math, time, cv2
import numpy as np, vectors2d as vec
"""
mapping
reads files
finds path

"""

class Node(object):
    def __init__(self, x, y, f = 0, g = 0, h = 0):
        self.x = int(x)
        self.y = int(y)
        self.f = float(f)
        self.g = float(g)
        self.h = float(h)
        self.parent = None

    def set_parent(self, parent):
        self.parent = parent    

    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"
    def compare(self, other):
        """
        returns True if other has the same coordinates but lower f
        """
        return (self.x == other.x and self.y == other.y and self.f >= other.f)
    def get_coord(self):
        return (self.x, self.y)
    
    def get_dist(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y -  other.y)**2)

    def is_obstacle(self, grid, maxY):
        #return if the node is on an obstacle -- if grid[yMax - 1 - self.y][self.x] == 1
		return grid[maxY -1 - self.y][self.x] == 1


def generate_successors(parent_node):
    node = parent_node.get_coord()
    successors = [] # a list of successor coordinates
    successors.append((node[0],node[1] + 1))
    successors.append((node[0] + 1,node[1]))
    successors.append((node[0],node[1] - 1))
    successors.append((node[0] - 1,node[1]))
    successors.append((node[0] + 1,node[1] + 1))
    successors.append((node[0] + 1,node[1] - 1))
    successors.append((node[0] - 1,node[1] + 1))
    successors.append((node[0] - 1,node[1] - 1))

    succ_nodes = []
   # print successors
    for succ in successors:
        succ_node = Node(*succ) #make the nodes
        succ_node.set_parent(parent_node) # set the parent node to the parent node
        succ_nodes.append(succ_node) # add to the list of successors
    return succ_nodes


def find_path(start, goal, grid, maxY):
    """
    start, and goal are tuples (x, y)
    grid is a grid with obstacles
    return a list of nodes that are explored in the map
    """
    stopSearch = False
    begin = time.time()
    startNode = Node(*start)
    goalNode = Node(*goal)
    openNodes = [startNode]
    closedNodes = []
##    path = []
    while len(openNodes) > 0 and not stopSearch:
        #find the node with the least f on the open list call it q
        if time.time() - begin >= 60:
            stopSearch = True
            print "time out, path not found"
        ind = 0
        min_f = openNodes[0].f
        for i in range(len(openNodes)):
            if min_f >= openNodes[i].f:
                min_f = openNodes[i].f
                ind = i
        q = openNodes.pop(ind)
        
        """
        generate q's 8 successors and set their parents to q
        for each successor
            if successor is the goal, stop the search
            successor.g = q.g + distance between successor and q
            successor.h = distance from goal to successor
            successor.f = successor.g + successor.h
        """
        successors = generate_successors(q)
        for successor in successors:
            if successor.x == goalNode.x and successor.y == goalNode.y:
                stopSearch = True
                print "path found"
                break
            successor.g = q.g + successor.get_dist(q)
            successor.h = successor.get_dist(goalNode)
            successor.f = successor.g + successor.h

            """
                if a node with the same position as successor is in the OPEN list \
                    which has a lower f than successor, skip this successor
                if a node with the same position as successor is in the CLOSED list \ 
                    which has a lower f than successor, skip this successor
                otherwise, add the node to the open list
            end
            push q on the closed list
            """
            skip = False
            for node in openNodes:
                if successor.compare(node):
                    skip = True
            for node in closedNodes:
                if successor.compare(node):
                    skip = True
            if not skip and not successor.is_obstacle(grid, maxY):
                openNodes.append(successor)

        closedNodes.append(q)
##        path.append(q.get_coord())

    closedNodes.append(goalNode)
##    path.append(goalNode.get_coord())
    
    return closedNodes
  

def draw_path(path, maxX, maxY, clearance = 1.0):
    pathGrid = np.zeros([maxY,maxX])
#    for x in range(maxX):
#        for y in range(maxY):
#            for p in range(len(path)):
#                try:
#                    isClear = (vec.pnt2line([x,y], path[p], path[p+1])[0]<=clearance)
#                except IndexError:
#                    isClear = (vec.pnt2line([x,y], path[p], path[0])[0]<=clearance)
#                if isClear:
#                    pathGrid[maxY-1-y][x] = 1
#                    break
    for step in path:
        x =maxY- step[1] -1
        y = step[0]
        pathGrid[x][y] = 1.0
    return pathGrid


def find_path_1(closedNodes):
    """
    return a list of tuples, that connect a path from explored nodes
    """
    end0 = closedNodes[len(closedNodes) - 1]
    end_coord0 = end0.get_coord()
    end_point =  closedNodes[len(closedNodes) - 2]
    start_point = closedNodes[0]
    end_coord = end_point.get_coord()
    start_coord = start_point.get_coord()

    path = [end_coord0,end_coord]
    while end_point.parent != None and (end_coord[0] != start_coord[0] or end_coord[1] != start_coord[1]):
        path.append(end_point.parent.get_coord())
        end_point =  end_point.parent
    return path

def draw_path_1(path, maxX, maxY, clearance = 1.0):
    """
    draw the actual path in a grid
    """
    pathGrid = np.zeros([maxY,maxX])
    for x in range(maxX):
        for y in range(maxY):
            for p in range(len(path) - 1):
                try:
                    isClear = (vec.pnt2line([x,y], path[p], path[p+1])[0]<=clearance)
                except IndexError:
                    isClear = (vec.pnt2line([x,y], path[p], path[0])[0]<=clearance)
                if isClear:
                    pathGrid[maxY-1-y][x] = 1
                    break
    return pathGrid


def cleanPath(path,allSegments, tolerance = 5.0):
    start = 0
    end = 2
    #path.reverse()
    while end<len(path)-1: 
        p1 = path[start]
        p2 = path[end]
        seg = [(p1[0],p1[1]),(p2[0],p2[1])]
        if isClearPath(seg,allSegments, tolerance):
            path.pop(end-1)
        else:
            start += 1
            end += 1
    path.reverse()
    return path
        
def isClearPath(SEG, allSegments, tolerance = 5.0):
    for segment in allSegments:
        if segments_distance(SEG, segment)<= tolerance:
            return False
    return True

def segments_distance(SEG1, SEG2):
  """ distance between two segments in the plane:
      one segment is (SEG1[0], SEG1[1]) to (SEG1[2], SEG1[3])
      the other is   (SEG2[0], SEG2[1]) to (SEG2[2], SEG2[3])
  """
  A = SEG1[0]
  B = SEG1[1]
  C = SEG2[0]
  D = SEG2[1]
  
  if segments_intersect(SEG1,SEG2): return 0
  # try each of the 4 vertices w/the other segment
  distances = []
  distances.append(point_segment_distance(A,SEG2))
  distances.append(point_segment_distance(B,SEG2))
  distances.append(point_segment_distance(C,SEG1))
  distances.append(point_segment_distance(D,SEG1))
  return min(distances)


def segments_intersect(SEG1,SEG2):
  """ whether two segments in the plane intersect:
      one segment is (SEG1[0], SEG1[1]) to (SEG1[2], SEG1[3])
      the other is   (SEG2[0], SEG2[1]) to (SEG2[2], SEG2[3])
  """
  x11 = float(SEG1[0][0])
  y11 = float(SEG1[0][1])
  x12 = float(SEG1[1][0])
  y12 = float(SEG1[1][1])
  x21 = float(SEG2[0][0])
  y21 = float(SEG2[0][1])
  x22 = float(SEG2[1][0])
  y22 = float(SEG2[1][1])

  
  dx1 = x12 - x11
  dy1 = y12 - y11
  dx2 = x22 - x21
  dy2 = y22 - y21
  delta = dx2 * dy1 - dy2 * dx1
  if delta == 0: return False  # parallel segments
  s = (dx1 * (y21 - y11) + dy1 * (x11 - x21)) / delta
  t = (dx2 * (y11 - y21) + dy2 * (x21 - x11)) / (-delta)
  return (0 <= s <= 1) and (0 <= t <= 1)

def point_segment_distance(P,SEG):
  """
      returns the distance between a point (P[0],P[1])
      and a line segment (SEG[0], SEG[1]) to (SEG[2], SEG[3])
  """
  px = float(P[0])
  py = float(P[1])
  x1 = float(SEG[0][0]) 
  y1 = float(SEG[0][1])
  x2 = float(SEG[1][0])
  y2 = float(SEG[1][1])


  dx = x2 - x1
  dy = y2 - y1
  if dx == dy == 0:  # the segment's just a point
    return math.hypot(px - x1, py - y1)

  # Calculate the t that minimizes the distance.
  t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

  # See if this represents one of the segment's
  # end points or a point in the middle.
  if t < 0:
    dx = px - x1
    dy = py - y1
  elif t > 1:
    dx = px - x2
    dy = py - y2
  else:
    near_x = x1 + t * dx
    near_y = y1 + t * dy
    dx = px - near_x
    dy = py - near_y

  return math.hypot(dx, dy)


def getButtonCoord(A,B,distance):
    """
    A and B are tuples
    returns a tuple and angle in degrees
    """
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

    coord = tuple(result)
    return coord,ang_deg

def makeMap(mapFileName,factor=1.0,clearance = 1.0):
    """
    returns a map enlarged by factor (should be 24)
    """    
    
    walls,dispenser,homeBase,stacks,startingPosition, maxX, maxY = parseFile(mapFileName,factor)
    
    #Calculate max x and y from walls and dispensors
#    maxX = 9 * factor
#    maxY = 10 * factor
#    def transform(x):
#        
#        return x * factor
#    
    #use walls and dispensors (and blocks maybe) to input obstacles in list of lines
    boundaries = walls + [dispenser]
    dispensors = [dispenser]
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
                dist, a = vec.pnt2line([x,y], line[0], line[1])
                if dist<=clearance:
                   boundaryGrid[maxY-1-y][x] = 1
                   break
                
    for x in range(maxX):
        for y in range(maxY):
            for line in dispensors:
                if vec.pnt2line([x,y], line[0], line[1])[0]<=clearance:
                    dispensorGrid[maxY-1-y][x] = 1
                    break

    homeX = []
    homeY = []
    for x in range(maxX):
        for y in range(maxY):
            if point_inside_polygon(x, y, homeBase):
                baseGrid[maxY-1-y][x] = 1
                homeX.append(x)
                homeY.append(y)
##            for hb in range(len(homeBase)):
##                try:
##                    isClear = (vec.pnt2line([x,y], homeBase[hb], homeBase[hb+1])[0]<=clearance)
##                except IndexError:
##                    isClear = (vec.pnt2line([x,y], homeBase[hb], homeBase[0])[0]<=clearance)
##                if isClear:
##                    baseGrid[maxY-1-y][x] = 1
##                    break
    homePos = (int(sum(homeX)/float(len(homeX))), int(sum(homeY)/float(len(homeY))))
    
    for x in range(maxX):
        for y in range(maxY):
            for sk in range(len(stacks)):
                if vec.pnt2line([x,y], stacks[sk], stacks[sk])[0]<=clearance:
                    stackGrid[maxY-1-y][x] = 1
                    break
    
                
#    print boundaryGrid
#    print "---------------------"
#    print dispensorGrid
#    print "---------------------"
#    print baseGrid
#    print "---------------------"
#    print stackGrid
#    print "---------------------"
    return boundaryGrid,dispensorGrid,baseGrid,stackGrid,walls,dispenser,homeBase,stacks,startingPosition, homePos, maxX, maxY

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


def get_final_heading(homePos, boundaries):
    bound = boundaries[0]
    dist = point_segment_distance(homePos,bound)
    for b in boundaries:
        if point_segment_distance(homePos,b) < dist:
            bound = b
            dist = point_segment_distance(homePos,b)
    A = np.array(bound[0])
    B = np.array(bound[1])
    M = (A+B)/2.0
    
    homep = np.array(homePos)
    ang_rad = math.atan2(M[1] - homep[1],M[0] - homep[0])
    ang_deg = math.degrees(ang_rad)
    return ang_deg


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
    dispenser = []
    homeBase = []
    stacks = []
    startPos = []
    
    for line in f:
    
        item = line.split(',')
    #    print line
        
        if item[0] == 'W':
            coordinate = []
            x1, y1 = scale*int(item[1]), scale*int(item[2])
            x2, y2 = scale*int(item[3]), scale*int(item[4])
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
##            if item[len(item) -1][0] == 'R':
            dispenser.append((x1, y1))
            dispenser.append((x2, y2))

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

    walls, dispenser, homeBase, stacks, startPos = shiftPoints(walls, dispenser, homeBase, stacks, startPos,minX,minY)

#    print "walls:", walls
#    print "dispensor_green:", dispenser_g
#    print "dispensor_red:", dispenser_r
#    print "home base:", homeBase
#    print "stacks:" , stacks
#    print "starting position:", startPos    
    
    return walls, dispenser, homeBase, stacks, startPos, maxX, maxY
    
   
def shiftPoints(walls, dispenser, homeBase, stacks, startPos,minX,minY):
    
    walls2 = []
    for w in walls:
        p1 = (w[0][0]-minX,w[0][1]-minY)
        p2 = (w[1][0]-minX,w[1][1]-minY)
        walls2.append([p1,p2])
    
##    dispenser_g2 = []
    dispenser_2 = []
    
##    for p in dispenser_g:
##        dispenser_g2.append((p[0]-minX,p[1]-minY))
    for p in dispenser:
        dispenser_2.append((p[0]-minX,p[1]-minY))

    homeBase2 = []
    for p in homeBase:
        homeBase2.append((p[0]-minX,p[1]-minY))
        
    stacks2 = []
    for p in stacks:
        stacks2.append((p[0]-minX,p[1]-minY))
        
    startPos2 = [(startPos[0][0]-minX,startPos[0][1]-minY),startPos[1]]
    
    return  walls2, dispenser_2, homeBase2, stacks2, startPos2



def displayMap(boundaryGrid,dispensorGrid,baseGrid,stackGrid,pathGrid, homePos, maxX, maxY):
    """
    draws the map
    """
    mapImage = 255*np.ones([maxY,maxX,3])
    mapImage[boundaryGrid==1] = [255,0,0]
    mapImage[dispensorGrid==1] = [0,255,255]
    mapImage[baseGrid==1] = [255,0,255]
    mapImage[stackGrid==1] = [0,0,255]
    mapImage[pathGrid==1] = [0,0,0]
    x = homePos[0]
    y = homePos[1]
    mapImage[maxY-1-y][x] = [0,255,0]
    cv2.imshow("map",mapImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def displayPath(pathGrid, maxX, maxY):
    """
    displays all the explored points in the grid
    """
    
    mapImage = 255*np.ones([maxY,maxX,3])
    mapImage[pathGrid==1] = [0,0,0]
    cv2.imshow("map",mapImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    mapFileName = "tournamentMapHarvard.txt"
#    mapFileName = "seedingMapMIT.txt"
##    mapFileName = "PracticeMap2.txt"
    factor = 24
    clearance = 9
    t1 = time.time()
    boundaryGrid,dispensorGrid,baseGrid,stackGrid,walls,dispensor,homeBase,stacks,startingPosition, homePos, maxX, maxY = makeMap(mapFileName, factor, clearance)
    start = startingPosition[0]#(2, 9)
##    goal = stacks[3] #bug at stack[3], stack[4]
    button_coord, button_heading = getButtonCoord(dispensor[0],dispensor[1], 10)
    goals = [start, button_coord, stacks[0], stacks[1], stacks[2], stacks[3], homePos]
#    goals = [start, stacks[0], stacks[1], stacks[2], stacks[3], homePos]
    #goals = stacks
    bounds = walls + [dispensor]
    s_ind = 0
    e_ind = 1
    t2 = time.time()
    path_total = [start]
    for i in range(len(goals) - 1):
        cNodes = find_path(goals[i], goals[i + 1], boundaryGrid, maxY)
        path = find_path_1(cNodes)
        path_clean = cleanPath(path, bounds,clearance)
        path_total.extend(path_clean[2:])
        print "completed step # ", i

    start_headings = ['E', 'N', 'W', 'S']
    init_heading = math.radians(start_headings.index(startingPosition[1]) * 90)    
    final_heading = get_final_heading(homePos, bounds)
    print "finished driving, robot heading " + str(final_heading) + " degrees"
    pathGrid1 = draw_path_1(path_total, maxX, maxY)
    t3 = time.time()
    
    
    np.savez(mapFileName + "output", init_heading=init_heading, path_total = np.array(path_total), final_heading = final_heading)
    
##  #  pathGrid = draw_path(path, maxX, maxY)
##    displayMap(boundaryGrid,dispensorGrid,baseGrid,stackGrid,pathGrid1, walls,dispensor,homeBase,stacks,startingPosition, homePos, maxX, maxY)
    displayMap(boundaryGrid,dispensorGrid,baseGrid,stackGrid,pathGrid1, homePos, maxX, maxY)
    #displayPath(pathGrid, maxX, maxY)
    t4 = time.time()
    
    print t2-t1,"seconds for grid creation"
    print t3-t2,"seconds for path planning"
    print t4-t3,"seconds for displaying the map"
