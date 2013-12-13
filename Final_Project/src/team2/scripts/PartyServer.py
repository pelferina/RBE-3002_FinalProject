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

staticMapGrid = OccupancyGrid()
localCostMapGrid = OccupancyGrid()
globalCostMapGrid = OccupancyGrid()


class Global:
    def __init__(self):
        self.threshold = 75
        self.groupCount = 0
        self.allGroups = []
        self.largeGroups = []
        self.xPos = 0
        self.yPos = 0
        self.theta = 0
    
    #evaluatePoints: can be used to find points containing obstacles in the global costmap
    def evaluatePoints(self):
        mapWidth = staticMapGrid.info.width
        mapHeight = staticMapGrid.info.height
        mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        localMapWidth = localCostMapGrid.info.width
        localMapHeight = localCostMapGrid.info.height
        localMapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        localMapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        potentialPoints = []
        
        for i in range(0, localMapWidth):
            for j in range(0, localMapHeight):
                if localCostMapGrid.data[(i * localMapWidth) + j] > self.threshold:
                    staticValue = 0
                    for k in range(-7, 8):
                        for l in range(-7, 8):
                            if staticMapGrid.data[((i - localMapOriginX + k) * mapWidth) + j - localMapOriginY + l] > 0:
                                staticValue = 1
                                break
                        else:
                            continue
                        break
                    if staticValue == 0:
                        potentialPoints.append(Cell(j + localMapOriginX, i + localMapOriginY))
        print len(potentialPoints)
        return potentialPoints
    
    #evaluateGroups:
    def evaluateGroups(self, potentialPoints):
        
        mapWidth = globalCostMapGrid.info.width
        mapHeight = globalCostMapGrid.info.height
        mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        localMapWidth = localCostMapGrid.info.width
        localMapHeight = localCostMapGrid.info.height
        localMapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        localMapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        for i in range(0, len(potentialPoints)):
            #If there are no groups, create one
            if len(self.allGroups) == 0:
                self.allGroups.append(Group(self.groupCount))
                self.allGroups[0].addCelltoGroup(potentialPoints[i])
                self.groupCount += 1
            else:
                inclusive = 0
                #check if the cell is in a group already
                for j in range(0, len(self.allGroups)):
                    for k in range(0, len(self.allGroups[j].cells)):
                        if potentialPoints[i].__eq__(self.allGroups[j].cells[k]):
                            inclusive = 1
                            break
                    else:
                        continue
                    break
                #if not add it to the appropriate group
                if inclusive == 0:
                    notCloseEnough = 1 
                    for l in range(0, len(self.allGroups)):
                        for m in range(0, len(self.allGroups[l].cells)):
                            #if the cell is close enough to a group, add the cell to that group and move to the next cell
                            if ((potentialPoints[i].x < self.allGroups[l].cells[m].x + 2 and potentialPoints[i].x > self.allGroups[l].cells[m].x - 2) and
                                (potentialPoints[i].y < self.allGroups[l].cells[m].x + 2 and potentialPoints[i].y > self.allGroups[l].cells[m].y - 2)):
                                self.allGroups[l].addCelltoGroup(potentialPoints[i])
                                notCloseEnough = 0
                                break
                        else:
                            continue
                        break
                    
                    #if not, add the cell to its own group and move to the next cell
                    if notCloseEnough == 1:
                        self.allGroups.append(Group(self.groupCount))
                        self.allGroups[len(self.allGroups) - 1].addCelltoGroup(potentialPoints[i])
                        self.groupCount += 1
        
        #calculate the center of each of the groups
        for n in range(0, len(self.allGroups)):
            indices = []
            for o in xrange(len(self.allGroups[n].cells) - 1, -1, -1):
                if localCostMapGrid.data[((self.allGroups[n].cells[o].y - localMapOriginY) * localMapWidth) + 
                                    self.allGroups[n].cells[o].x - localMapOriginX] < self.threshold:
                    del self.allGroups[n].cells[o]
        
            #self.allGroups[n].computeCenterofGroup()

g = Global()

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

#class Subscriber:
    
   # def __init__(self):
   #     self.getOdomData()
   #     self.getStaticMapData()
   #     self.getGlobalCostmapData()
    #Subscriber Functions
def getOdomData():
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    
def getStaticMapData():
    sub = rospy.Subscriber("/map", OccupancyGrid, staticMapCallBack)
        
def getGlobalCostmapData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapCallBack)

def getLocalCostmapData():
    sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, localCostmapCallBack)
    
#class Publisher:
    #Publisher Functions
    #publishGoal: publishes the x position, y position and angle for the robot to move to
def publishGoal(xPos, yPos, angle):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    goal = PoseStamped()
    goal.pose.position.x = xPos
    goal.pose.position.y = yPos
        #determine theta eventually
    pub.publish(goal)
    
def publishCells(listOfCells):
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

#Subscriber Callback functions
def staticMapCallBack(data):
    global staticMapGrid
    staticMapGrid = data
    
def localCostmapCallBack(data):
    global localCostMapGrid
    localCostMapGrid = data
    
def globalCostmapCallBack(data):
    global globalCostMapGrid
    globalCostMapGrid = data
    
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
    getOdomData()
    getStaticMapData()
    getLocalCostmapData()
    getGlobalCostmapData()
    #p = Publisher()
    while(len(staticMapGrid.data) == 0 or
          len(globalCostMapGrid.data) == 0):
        rand  = 0
    #print len(staticMapGrid.data)
    #print staticMapGrid.info.width
    while(1):
        cells = g.evaluatePoints()
        publishCells(cells)
        #for i in range(0, len(cells)):
        #    print '[', cells[i].x, ' ', cells[i].y, ']'
        print len(cells)
        oldData = localCostMapGrid.data
        g.evaluateGroups(cells)
        print len(g.allGroups)
        while oldData == localCostMapGrid.data:
            rand = 1
            
            
        print 'Change'
        
