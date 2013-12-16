#!/usr/bin/env python

#Ayesha Fathima
#Gregory McCarthy
#Robert Edwards

#Lab 5: Final Project Code
#PartyServer.py

#Imports
import rospy
import roslib
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

#Global Map Variables
staticMapGrid = OccupancyGrid()
globalCostMapGrid = OccupancyGrid()

#global Class for storing group information and grid cell data
class Global:
    def __init__(self):
        self.threshold = 75
        self.dockingThreshold = 10
        self.groupCount = 0
        self.allGroups = []
        self.largeGroups = []
        self.xPos = 0
        self.yPos = 0
        self.theta = 0
        self.groupThreshold = 30
    
    #evaluatePoints: can be used to find points containing obstacles in the global costmap
    def evaluatePoints(self):
        
        #Map Constants
        mapWidth = staticMapGrid.info.width
        mapHeight = staticMapGrid.info.height
        mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
        
        potentialPoints = []
        
        for y in range(0, mapHeight):
            for x in range(0, mapWidth):
                #Check if the global costmap data is above the threshold
                if globalCostMapGrid.data[(y * mapWidth) + x] > self.threshold:
                    #Check if that grid on the static map is unknown space
                    if staticMapGrid.data[(y * mapWidth) + x] != -1:
                        staticValue = 0
                        #Check if any grids within 20cm around the point are occupied on the static map
                        for yRange in range(-4, 5):
                            for xRange in range(-4, 5):
                                if staticMapGrid.data[((y + yRange) * mapWidth) + x + xRange] != 0:
                                    staticValue = 1
                                    break
                            else:
                                continue
                            break
                        #If not occupied, add the point to the list of cells, updating it's position relative to the origin.
                        if staticValue == 0:
                            potentialPoints.append(Cell(x + mapOriginX, y + mapOriginY))
        print 'Points:', len(potentialPoints)
        return potentialPoints
    
    def evaluateDockingPoints(self):
        
        mapWidth = staticMapGrid.info.width
        mapHeight = staticMapGrid.info.height
        mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
        
        potentialPoints = []
        
        for y in range(0, mapHeight):
            for x in range(0, mapWidth):
                if (globalCostMapGrid.data[(y * mapWidth) + x] == 0):
                    yVal = -4
                    xVal = -4
                    for yRange in range(-3, 4):
                        yVal =yRange
                        for xRange in range(-3, 4):
                            xVal = xRange
                            if globalCostMapGrid.data[((y + yRange) * mapWidth) + x + xRange] > 0:
                                staticValue = 0
                                for yRange1 in range(-13, 14):
                                    for xRange1 in range(-13, 14):
                                        if staticMapGrid.data[((y + yRange1) * mapWidth) + x + xRange1] != 0:
                                            staticValue = 1
                                            break
                                    else:
                                        continue
                                    break
                        #If not occupied, add the point to the list of cells, updating it's position relative to the origin.
                                if staticValue == 0:
                                    closeEnough = 0
                                    for yValue in range(-9, 10):
                                        for xValue in range(-9, 10):
                                            if globalCostMapGrid.data[((y + yValue) * mapWidth) + x + xValue] > 0:
                                                closeEnough = 1
                                                break
                                        else:
                                            continue
                                        break
                                    if closeEnough == 1:
                                        potentialPoints.append(Cell(x + mapOriginX, y + mapOriginY))
                                        break
                        else:
                            continue
                        break
        print 'Points:', len(potentialPoints)
        return potentialPoints
    
    #evaluateGroups: used to take in points and group them together
    def evaluateGroups(self, potentialPoints, dockingPoints):
        
        #Map Constants
        mapWidth = globalCostMapGrid.info.width
        mapHeight = globalCostMapGrid.info.height
        mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        #For all points
        for i in range(0, len(potentialPoints)):
            #If there are no groups, create a new one
            if len(self.allGroups) == 0:
                print 'First Group'
                self.allGroups.append(Group(self.groupCount))
                self.allGroups[0].addCelltoGroup(potentialPoints[i])
                self.groupCount += 1
            else:
                inclusive = 0
                #If the cell is already in a group, discard the point
                for j in range(0, len(self.allGroups)):
                    for k in range(0, len(self.allGroups[j].cells)):
                        if potentialPoints[i].__eq__(self.allGroups[j].cells[k]):
                            inclusive = 1
                            break
                    else:
                        continue
                    break
                #If it is not in a group, check if it is close enough to a given group.
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
                    
                    #If the cell is not close enough, create a new group and add the cell to it.
                    if notCloseEnough == 1:
                        #print 'New Group'
                        self.allGroups.append(Group(self.groupCount))
                        self.allGroups[len(self.allGroups) - 1].addCelltoGroup(potentialPoints[i])
                        self.groupCount += 1
        
        print 'Groups prior to out of date deletion:', len(self.allGroups)
        
        #Delete any out of date cells.
        self.allGroups = delOutofDateCells(self.allGroups, self.threshold)
        
        #Merge any groups in range on one another
        print 'Groups prior to merging:', len(self.allGroups)
        lenGroups = len(self.allGroups)
        for p in range(lenGroups - 1, -1, -1):
            cellEqual = 0
            for r in range(len(self.allGroups) - 1, -1, -1):
                if r != p:
                    for q in range(0, len(self.allGroups[p].cells)):
                        for s in range(0, len(self.allGroups[r].cells)):
                            if ((self.allGroups[r].cells[s].x < self.allGroups[p].cells[q].x + 4 and 
                                 self.allGroups[r].cells[s].x > self.allGroups[p].cells[q].x - 4) and 
                                (self.allGroups[r].cells[s].y < self.allGroups[p].cells[q].y + 4 and 
                                 self.allGroups[r].cells[s].y > self.allGroups[p].cells[q].y - 4)):
                                cellEqual += 1
                                if cellEqual >= 1:
                                    self.allGroups[r].cells.extend(self.allGroups[p].cells)
                                    del self.allGroups[p]
                                    break
                        else:
                            continue
                        break
                    else:
                        continue
                    break
        print 'Merged groups:', len(self.allGroups)
        #Check the validity of the groups
        validity = 0
        for t in range(0, len(self.allGroups)):
            if checkIfValidGroup(self.allGroups[t]) > -1:
                validity = 1
        if validity == 1:
            print 'INVALID'
            
        #Delete any groups below a certain threshold. 
        self.allGroups = delSmallGroups(self.allGroups, self.groupThreshold)    
        for j in range(0, len(dockingPoints)):
            for k in range(0, len(self.allGroups)):
                for l in range(0, len(self.allGroups[k].cells)):
                    if((math.sqrt(math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2) + 
                                 math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2)) < 10) and
                       (math.sqrt(math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2) + 
                                 math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2)) > 5)):
                        self.allGroups[k].dockingPoints.append(dockingPoints[j])
                        break
                else:
                    continue
                break
        
                        
        
        #Publish the groups to RViz
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
        self.dockingPoints = []
        self.centerX = 0
        self.centerY = 0
    
    def addCelltoGroup(self, cell):
        self.cells.append(cell)
    
    def addDockingPoint(self, point):
        self.dockingPoints.append(point)
    
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
            if abs(newGroup.cells[i].x - newGroup.cells[i + 1].x) > 5:
                return i
    newGroup.cells = sorted(newGroup.cells, key = lambda cell: cell.y)
    for i in range(0, len(newGroup.cells)):
        if i == len(group.cells) - 1:
            return -1
        else:
            if abs(newGroup.cells[i].y - newGroup.cells[i + 1].y) > 5:
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

def getOdomData():
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    
def getStaticMapData():
    sub = rospy.Subscriber("/map", OccupancyGrid, staticMapCallBack)
        
def getGlobalCostmapData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapCallBack)

def getGlobalUpdateData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, globalCostmapUpdate)
    
#class Publisher:
    #Publisher Functions
    #publishGoal: publishes the x position, y position and angle for the robot to move to
def publishGoal(xPos, yPos, angle):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    goal = PoseStamped()
    goal.pose.position.x = xPos
    goal.pose.position.y = yPos
        #determine theta eventually
    pub.publish(goal);
    
def publishCells(listOfCells):
    pub = rospy.Publisher('/team2/cells_with_obstructions', GridCells)
    
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

def publishDockingPoints(listOfCells):
    pub = rospy.Publisher('/team2/docking_points', GridCells)
    
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
    

#Subscriber Callback functions
def staticMapCallBack(data):
    global staticMapGrid
    staticMapGrid = data
    
def globalCostmapCallBack(data):
    global globalCostMapGrid
    globalCostMapGrid = data

def globalCostmapUpdate(data):
    global globalCostMapGrid
    
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

#Main Code
if __name__ == '__main__':
    rospy.init_node('PartyServer', anonymous=True)
    global g
    
    getOdomData()
    getStaticMapData()
    getGlobalCostmapData()
    getGlobalUpdateData()
    
    while(len(staticMapGrid.data) == 0 or
          len(globalCostMapGrid.data) == 0):
        rand  = 0
    while(1):
        cells = g.evaluatePoints()
        dockCells = g.evaluateDockingPoints()
        publishCells(cells)
        publishDockingPoints(dockCells)
        g.evaluateGroups(cells, dockCells)
        oldData = globalCostMapGrid.data
        while oldData == globalCostMapGrid.data:
            rand = 1
        print 'Costmap Updated'