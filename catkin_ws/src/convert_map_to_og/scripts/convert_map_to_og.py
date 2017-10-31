#!/usr/bin/env python

import rospy
import roslib
from MakeMap import makeMap
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose



class staticMapPublisher(object):

    def __init__(self):
        self.map_file = '/home/maslab/Code/team1/catkin_ws/src/convert_map_to_og/maps/map1.txt'
        
        boundaryGrid,dispensorGrid,baseGrid,stackGrid,\
        walls,greenDispensors,redDispensor,homeBase,stacks,\
        startingPosition, maxX, maxY = makeMap(self.map_file, 48, 1) #filename, points per unit, linewidth

        # initalize the occupancy grid msg
        self.metaData = MapMetaData()
        self.metaData.resolution = 0.02 # each grid unit is 2 cm
        self.metaData.width = int(10 / self.metaData.resolution) # 10 m
        self.metaData.height = int(10 / self.metaData.resolution) # 10 m
        self.metaData.origin = Pose([0,0,0],[0,0,0,1]) # defines the map CF
        self.occGrid = OccupancyGrid()
        self.occGrid.info = self.metaData
        self.occGrid.data = [] # list of points (row major order) with a value -1 for unknown, [0,100] for occ. prob.

        # each map unit is 0.5in (1.27cm), but occgrid units are 2cm
        for row in range(len(boundaryGrid)):
            for column in range(len(boundaryGrid[0])):
                occGridIndex = int(row*self.metaData.width*(1.27/2) + column*(1.27/2))
                self.occGrid.data[occGridIndex] = 100

        print "Successfully converted %s to a nav_msgs/OccupancyGrid" % self.map_file

        self.ogPub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=1)
        self.rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            self.ogPub.publish(self.occGrid)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('static_map_publisher')
    try:
        staticMapPub = staticMapPublisher()
    except rospy.ROSInterruptException: pass
    