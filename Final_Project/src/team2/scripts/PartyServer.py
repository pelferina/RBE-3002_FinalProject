#!/usr/bin/env python
import rospy
import roslib

#roslib.load_manifest('Final_Project')

import time
import math

from numpy import *

#Message Type Imports
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent


class Global:
    def __init__(self):
        self.staticMapGrid = OccupancyGrid()
        self.costMapGrid = OccupancyGrid()
        self.threshold = 50
        self.groupCount = 0
        self.allGroups = []
        self.largeGroups = []
        self.xPos = 0
        self.yPos = 0
        self.theta = 0
    
    #evaluatePoints: can be used to find points containing obstacles in the global costmap
    def evaluatePoints(self):
        mapWidth = self.staticMapGrid.info.width
        mapHeight = self.staticMapGrid.info.height
        mapOriginX = int(math.floor(self.costMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(self.costMapGrid.info.origin.position.y * 20))
        potentialPoints = []
        for i in range(0, mapWidth):
            for j in range(0, mapHeight):
                if self.costMapGrid.data[(i * mapWidth) + j] > self.threshold:
                    staticValue = 0
                    for k in range(-3, 4):
                        for l in range(-3, 4):
                            staticValue += self.staticMapGrid.data[((i + k) * mapWidth) + j + k]
                    if staticValue <= 0:
                        potentialPoints.append(Cell(j + mapOriginX, i + mapOriginY))
        return potentialPoints
    
    #evaluateGroups:
    def evaluateGroups(potentialPoints):
        
        mapWidth = costMapGrid.info.width
        mapHeight = costMapGrid.info.height
        mapOriginX = int(math.floor(costMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(costMapGrid.info.origin.position.y * 20))
        for i in range(0, len(potentialPoints)):
            #If there are no groups, create one
            if len(allGroups) == 0:
                allGroups.append(Group(self.groupCount))
                allGroups[0].addCelltoGroup(potentialPoints[i])
                self.groupCount += 1
            else:
                inclusive = 0
                #check if the cell is in a group already
                for j in range(0, len(allGroups)):
                    for k in range(0, len(allGroups[j].cells)):
                        if potentialPoints[i].__eq__(allGroups[j].cells[k]):
                            inclusive += 1
                #if not add it to the appropriate group
                if inclusive == 0:
                    notCloseEnough = 1 
                    for l in range(0, len(allGroups)):
                        for m in range(0, len(allGroups[l].cells)):
                            #if the cell is close enough to a group, add the cell to that group and move to the next cell
                            if ((potentialPoints[i].x < allGroups[l].cells[m].x + 2 and potentialPoints[i].x > allGroups[l].cells[m].x - 2) and
                                (potentialPoints[i].y < allGroups[l].cells[m].x + 2 and potentialPoints[i].y > allGroups[l].cells[m].y - 2)):
                                allGroups[l].addCelltoGroup(potentialPoints[i])
                                notCloseEnough = 0
                                m = len(allGroups[l].cells)
                                l = len(allGroups)
                    #if not, add the cell to its own group and move to the next cell
                    if notCloseEnough == 1:
                        allGroups.append(Group(self.groupCount))
                        allGroups[len(allGroups) - 1].addCelltoGroup(potentialPoints[i])
                        self.groupCount += 1
        #calculate the center of each of the groups
        for n in range(0, len(allGroups)):
            for o in range(0, len(allGroups[n].cells)):
                if costMapGrid.data[((allGroups[n].cells[len(allGroups[n].cells) - 1 - o].y + mapOriginY) * mapWidth) + 
                                    allGroups[n].cells[len(allGroups[n].cells) - 1 - o].x + mapOriginX] < self.threshold:
                    del allGroups[n].cells[len(allGroups[n].cells) - 1 - o]
            allGroups[n].computeCenterofGroup()

global g

#Group class: for storing cells in a given group
class Group:
    def __init__(self, num):
        self.index = num
        self.cells = []
        self.centerX = 0
        self.centerY = 0
    
    def addCelltoGroup(self, cell):
        self.cells.append(cell)
    
    def computeCenterofGroup(self):
        sumXpos = 0
        sumYpos = 0
        for i in range(0, len(self.cells)):
            sumXpos += self.cells[i].x
            sumYpos += self.cells[i].y
        self.centerX = sumXpos / len(self.cells)
        self.centerY = sumYpos / len(self.cells)

class Subscriber:
    
    def __init__(self):
        self.getOdomData()
        self.getStaticMapData()
        self.getGlobalCostmapData()
    #Subscriber Functions
    def getOdomData(data):
        sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    
    def getStaticMapData(data):
        sub = rospy.Subscriber("/map", OccupancyGrid, staticMapCallBack)
        
    def getGlobalCostmapData(data):
        sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapCallBack)
    
    

class Publisher:
    #Publisher Functions
    #publishGoal: publishes the x position, y position and angle for the robot to move to
    def publishGoal(xPos, yPos, angle):
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        goal = PoseStamped()
        goal.pose.position.x = xPos
        goal.pose.position.y = yPos
        #determine theta eventually
        pub.publish(goal)
    
    def publishCells(self, listOfCells):
        pub = rospy.Publisher('/team2/cells_with_obstructions', GridCells)
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = 0.05
        cells.cell_height = 0.05
        for i in range(0, len(listOfCells)):
            point = Point()
            point.x = (listOfCells[i].x) * 0.05 + 0.025
            point.y = (listOfCells[i].y) * 0.05 + 0.025
            point.z = 0
            cells.cells.append(point)
        pub.publish(cells)
        rospy.sleep(0.1)     

#Subscriber Callback functions
def staticMapCallBack(data):
    global g
    g.staticMapGrid = data
def globalCostmapCallBack(data):
    global g
    g.costMapGrid = data
def odomCallback(data):
    global g
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    g.xPos = px
    g.yPos = py
    g.theta = yaw * 180.0 / math.pi

#Cell class: for storing position data of a given cell
class Cell:
    def __init__(self, xPos, yPos):
        self.x = xPos
        self.y = yPos
    
    def __eq__(self, cell):
        if self.x == cell.x and self.y == cell.y:
            return True
        else:
            return False   

if __name__ == '__main__':
    rospy.init_node('PartyServer', anonymous=True)
    global g
    g = Global()
    s = Subscriber()
    p = Publisher()
    while(len(g.staticMapGrid.data) == 0 or
          len(g.costMapGrid.data) == 0):
        rand  = 0
    print len(g.staticMapGrid.data)
    print g.staticMapGrid.info.width
    while(1):
        cells = g.evaluatePoints()
        p.publishCells(cells)
        for i in range(0, len(cells)):
            print '[', cells[i].x, ' ', cells[i].y, ']'
