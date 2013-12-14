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

from map_msgs.msg import OccupancyGridUpdate

from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

staticMapGrid = OccupancyGrid()
localCostMapGrid = OccupancyGrid()
globalCostMapGrid = OccupancyGrid()
globalCostMapUpdate = OccupancyGridUpdate()

updateCount = 0


class Global:
    def __init__(self):
        self.threshold = 75
        self.groupCount = 0
        self.allGroups = []
        self.largeGroups = []
        self.xPos = 0
        self.yPos = 0
        self.theta = 0
        self.groupThreshold = 15
    
    #evaluatePoints: can be used to find points containing obstacles in the global costmap
    def evaluatePoints(self):
        mapWidth = staticMapGrid.info.width
        mapHeight = staticMapGrid.info.height
        mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
        
        potentialPoints = []
        
        for y in range(0, mapHeight):
            for x in range(0, mapWidth):
                if globalCostMapGrid.data[(y * mapWidth) + x] > self.threshold:
                    staticValue = 0
                    for yRange in range(-4, 5):
                        for xRange in range(-4, 5):
                            if staticMapGrid.data[((y + yRange) * mapWidth) + x + xRange] > 0:
                                staticValue = 1
                                break
                        else:
                            continue
                        break
                    if staticValue == 0:
                        potentialPoints.append(Cell(x + mapOriginX, y + mapOriginY))
        print 'Points:', len(potentialPoints)
        return potentialPoints
    
    #evaluateGroups:
    def evaluateGroups(self, potentialPoints):
        
        mapWidth = globalCostMapGrid.info.width
        mapHeight = globalCostMapGrid.info.height
        mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        for i in range(0, len(potentialPoints)):
            #If there are no groups, create one
            if len(self.allGroups) == 0:
                print 'First Group'
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
                            if ((self.allGroups[l].cells[m].x < potentialPoints[i].x + 2 and 
                                 self.allGroups[l].cells[m].x > potentialPoints[i].x - 2) and
                                (self.allGroups[l].cells[m].x < potentialPoints[i].y + 2 and 
                                 self.allGroups[l].cells[m].y > potentialPoints[i].y - 2)):
                                self.allGroups[l].addCelltoGroup(potentialPoints[i])
                                notCloseEnough = 0
                                break
                        else:
                            continue
                        break
                    
                    #if not, add the cell to its own group and move to the next cell
                    if notCloseEnough == 1:
                        #print 'New Group'
                        self.allGroups.append(Group(self.groupCount))
                        self.allGroups[len(self.allGroups) - 1].addCelltoGroup(potentialPoints[i])
                        self.groupCount += 1
        print 'Groups prior to out of date deletion:', len(self.allGroups)
        
        print 'Old Cells Deleted'
        self.allGroups = delOutofDateCells(self.allGroups, self.threshold)
        
        print 'Groups prior to merging:', len(self.allGroups)
        lenGroups = len(self.allGroups)
        for p in range(lenGroups - 1, -1, -1):
            #print 'Groups =', len(self.allGroups), ', P = ', p
            #print 'Cells =', len(self.allGroups[p].cells)
            cellEqual = 0
            for r in range(len(self.allGroups) - 1, -1, -1):
                #print 'Check against group', r
                if r != p:
                    for q in range(0, len(self.allGroups[p].cells)):
                        for s in range(0, len(self.allGroups[r].cells)):
                            #print 'Cell', s, 'of group', r, 'against cell', q, 'of group', p
                            if ((self.allGroups[r].cells[s].x < self.allGroups[p].cells[q].x + 3 and 
                                 self.allGroups[r].cells[s].x > self.allGroups[p].cells[q].x - 3) and 
                                (self.allGroups[r].cells[s].y < self.allGroups[p].cells[q].y + 3 and 
                                 self.allGroups[r].cells[s].y > self.allGroups[p].cells[q].y - 3)):
                                cellEqual += 1
                                if cellEqual >= 1:
                                    #print 'Appending groups together'
                                    self.allGroups[r].cells.extend(self.allGroups[p].cells)
                                    del self.allGroups[p]
                                    break
                        else:
                            continue
                        break
                    else:
                        continue
                    break
                #else:
                    #print 'Same Group. Repicking...'
        print 'Merged groups:', len(self.allGroups)
        validity = 0
        for t in range(0, len(self.allGroups)):
            if checkIfValidGroup(self.allGroups[t]) > -1:
                validity = 1
        if validity == 1:
            print 'INVALID'
        self.allGroups = delSmallGroups(self.allGroups, self.groupThreshold)
            
        print 'Groups after small deletion:', len(self.allGroups)
        publishGroups(self.allGroups)

g = Global()

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
        
def checkIfValidGroup(group):
    newGroup = group
    newGroup.cells = sorted(newGroup.cells, key = lambda cell: cell.x)
    for i in range(0, len(newGroup.cells)):
        if i == len(group.cells) - 1:
            break
        else:
            if abs(newGroup.cells[i].x - newGroup.cells[i + 1].x) > 4:
                return i
    newGroup.cells = sorted(newGroup.cells, key = lambda cell: cell.y)
    for i in range(0, len(newGroup.cells)):
        if i == len(group.cells) - 1:
            return -1
        else:
            if abs(newGroup.cells[i].y - newGroup.cells[i + 1].y) > 4:
                return i
            
            

#delSmallGroups: function used to delete old groups with no data. 
def delSmallGroups(listOfGroups, cutoff):
    newList = listOfGroups
    for i in range(len(newList) - 1, -1, -1):
        if len(newList[i].cells) < cutoff:
            del newList[i]
    return newList

def delOutofDateCells(listOfGroups, threshold):
    
    mapWidth = globalCostMapGrid.info.width
    
    mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
    mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
    
    newList = listOfGroups
    for i in range(len(newList) - 1, -1, -1):
        for j in range(len(newList[i].cells) - 1, -1, -1):
            if globalCostMapGrid.data[((newList[i].cells[j].y - mapOriginY) * mapWidth) + 
                                      newList[i].cells[j].x - mapOriginX] < threshold:
                del newList[i].cells[j]
    return newList

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

def getGlobalUpdateData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, globalCostmapUpdate)

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
    
    localMapOriginX = int(math.floor(localCostMapGrid.info.origin.position.x * 20))
    localMapOriginY = int(math.floor(localCostMapGrid.info.origin.position.y * 20))
    
    mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
    mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.05
    cells.cell_height = 0.05
    for i in range(0, len(listOfCells)):
        point = Point()
        point.x = ((listOfCells[i].x) * 0.05) + 0.025
        point.y = ((listOfCells[i].y) * 0.05) + 0.025
        point.z = 0
        cells.cells.append(point)
    pub.publish(cells)  

def publishGroups(listOfGroups):
    pub = rospy.Publisher('/team2/groups', GridCells)
    
    localMapOriginX = int(math.floor(localCostMapGrid.info.origin.position.x * 20))
    localMapOriginY = int(math.floor(localCostMapGrid.info.origin.position.y * 20))
    
    mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
    mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.05
    cells.cell_height = 0.05
    for i in range(0, len(listOfGroups)):
        for j in range(0, len(listOfGroups[i].cells)):
            point = Point()
            point.x = ((listOfGroups[i].cells[j].x) * 0.05) + 0.025
            point.y = ((listOfGroups[i].cells[j].y) * 0.05) + 0.025
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

def globalCostmapUpdate(data):
    global globalCostMapUpdate
    global globalCostMapGrid
    
    globalCostMapUpdate = data
    
    mapWidth = globalCostMapGrid.info.width
    mapHeight = globalCostMapGrid.info.height
    
    if len(globalCostMapGrid.data) > 0:
        mapData = list(globalCostMapGrid.data)
        for y in range(0, data.height):
            for x in range(0, data.width):
                mapData[((y + data.y) * mapWidth) + x + data.x] = data.data[(y * data.width) + x]
        globalCostMapGrid.data = tuple(mapData)
    
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

if __name__ == '__main__':
    rospy.init_node('PartyServer', anonymous=True)
    global g
    
    getOdomData()
    getStaticMapData()
    getLocalCostmapData()
    getGlobalCostmapData()
    getGlobalUpdateData()
    
    while(len(staticMapGrid.data) == 0 or
          len(globalCostMapGrid.data) == 0):
        rand  = 0
    while(1):
        cells = g.evaluatePoints()
        publishCells(cells)
        oldData = globalCostMapGrid.data
        g.evaluateGroups(cells)
        while oldData == globalCostMapGrid.data:
            rand = 1
        print 'Costmap Updated'