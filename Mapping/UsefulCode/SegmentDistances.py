# -*- coding: utf-8 -*-
"""
Created on Tue Feb 09 23:44:29 2016

@author: arinz
"""
import math

def segments_distance(SEG1, SEG2):
  """ distance between two segments in the plane:
      one segment is (SEG1[0], SEG1[1]) to (SEG1[2], SEG1[3])
      the other is   (SEG2[0], SEG2[1]) to (SEG2[2], SEG2[3])
  """
  A = [SEG1[0],SEG1[1]]
  B = [SEG1[2],SEG1[3]]
  C = [SEG2[0],SEG2[1]]
  D = [SEG2[2],SEG2[3]]
  
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
  x11 = float(SEG1[0])
  y11 = float(SEG1[1])
  x12 = float(SEG1[2])
  y12 = float(SEG1[3])
  x21 = float(SEG2[0])
  y21 = float(SEG2[1])
  x22 = float(SEG2[2])
  y22 = float(SEG2[3])

  
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
  x1 = float(SEG[0]) 
  y1 = float(SEG[1])
  x2 = float(SEG[2])
  y2 = float(SEG[3])


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
  


#SEG1 = (0,0,1,0)
#SEG2 = (2,1,2,2)
#
#print segments_distance(SEG1,SEG2)

#SEG = (0.,12.,0.,8.)
#P1 = (0.,15.)
#P2 = (0.,12.)
#P3 = (0.,10.)
#P4 = (0.,8.)
#P5 = (0.,4.)
#P6 = (1.,10.)
#points = [P1,P2,P3,P4,P5,P6]
#
#for point in points:
#    print point_segment_distance(point,SEG)