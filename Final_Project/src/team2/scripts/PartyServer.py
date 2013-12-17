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
        self.groupThreshold = 50
    
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
        print 'Docking Points:', len(potentialPoints)
        return potentialPoints
    
    #evaluateGroups: used to take in points and group them together
    def evaluateGroups(self, potentialPoints, dockingPoints):
        
        #Map Constants
        mapWidth = globalCostMapGrid.info.width
        mapHeight = globalCostMapGrid.info.height
        mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        #Delete any groups below a certain threshold. 
        self.delSmallGroups()
        
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
                        
        #Merge any groups in range on one another
        print 'Groups prior to merging:', len(self.allGroups)
        lenGroups = len(self.allGroups)
        
        for p in range(lenGroups - 1, -1, -1):
            cellEqual = 0
            for r in range(len(self.allGroups) - 1, -1, -1):
                if r != p:
                    for q in range(0, len(self.allGroups[p].cells)):
                        for s in range(0, len(self.allGroups[r].cells)):
                            if ((self.allGroups[r].cells[s].x <= self.allGroups[p].cells[q].x + 4 and 
                                 self.allGroups[r].cells[s].x >= self.allGroups[p].cells[q].x - 4) and 
                                (self.allGroups[r].cells[s].y <= self.allGroups[p].cells[q].y + 4 and 
                                 self.allGroups[r].cells[s].y >= self.allGroups[p].cells[q].y - 4)):
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
        
                
            
        #Delete any out of date cells.
        self.delOutofDateCells()
            
        #Delete any groups below a certain threshold. 
        self.delSmallGroups  
        
        #Reset a groups given docking points
        for k in range(0, len(self.allGroups)):
            self.allGroups[k].dockingPoints = []
            
        #Append the recalculated docking points to the appropriate groups in an appropriate manner
        for j in range(0, len(dockingPoints)):
            for k in range(0, len(self.allGroups)):
                for l in range(0, len(self.allGroups[k].cells)):
                    if((math.sqrt(math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2) + 
                                 math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2)) < 10) and
                       (math.sqrt(math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2) + 
                                 math.pow(self.allGroups[k].cells[l].x - dockingPoints[j].x, 2)) > 9)):
                        self.allGroups[k].dockingPoints.append(dockingPoints[j])
                        break
                else:
                    continue
                break
        
                        
        
        #Publish the groups to RViz
        publishGroups(self.allGroups)
        publishDockingPoints(self.allGroups)
    
    def delSmallGroups(self):
        for i in range(len(self.allGroups) - 1, -1, -1):
            if len(self.allGroups[i].cells) < self.groupThreshold:
                del self.allGroups[i]
    
    def delOutofDateCells(self):
    
        mapWidth = globalCostMapGrid.info.width
        mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
        
        for i in range(len(self.allGroups) - 1, -1, -1):
            for j in range(len(self.allGroups[i].cells) - 1, -1, -1):
                if globalCostMapGrid.data[((self.allGroups[i].cells[j].y - mapOriginY) * mapWidth) + 
                                          self.allGroups[i].cells[j].x - mapOriginX] < self.threshold:
                    del self.allGroups[i].cells[j]
    
    def checkIfValidGroup(self, group):
        group.cells = sorted(group.cells, key = lambda cell: cell.x)
        for i in range(0, len(group.cells)):
            if i == len(group.cells) - 1:
                break
            else:
                if abs(group.cells[i].x - group.cells[i + 1].x) > 5:
                    return i
        group.cells = sorted(group.cells, key = lambda cell: cell.y)
        for i in range(0, len(group.cells)):
            if i == len(group.cells) - 1:
                return -1
            else:
                if abs(group.cells[i].y - group.cells[i + 1].y) > 5:
                    return i

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
            
class Frontier_Exploration :
    def __init__(self):
        self.MAP_OPEN_LIST = 1
        self.MAP_CLOSE_LIST = 2
        self.FRONTIER_OPEN_LIST = 3
        self.FRONTIER_CLOSE_LIST = 4
        self.OCC_THRESHOLD = 10
        self.map = staticMapGrid
        self.map_width = self.map.info.width
        self.map_height = self.map.info.height
        self.map_size = self.map_height * self.map_width
        self.x = 0
        self.y = 0        
        self.pose = int(math.floor(self.map.info.origin.position.y * 20))
        
    def wfd(self):
        
        frontiers = []
        
        self.cell_states = {}
        self.q_m = []
        self.q_m.insert(0,self.pose)
        
        self.cell_states[self.pose] = self.MAP_OPEN_LIST
        self.adj_vector = []
        self.v_neighbors= []
        
        while len(self.q_m) > 0 :
            cur_pos = self.q_m.pop(0)
            if self.cell_states[cur_pos] == self.MAP_CLOSE_LIST :
                continue
            if self.is_frontier_point(cur_pos) :
                self.q_f = []
                self.new_frontier = []
                self.q_f.insert(0, cur_pos)
                self.cell_states[cur_pos] = self.FRONTIER_OPEN_LIST
                
                while len(self.q_f) > 0 :
                    n_cell = self.q_f.pop(0)
                    if self.cell_states[n_cell] == self.MAP_CLOSE_LIST or self.cell_states[n_cell] == self.FRONTIER_CLOSE_LIST :
                        continue
                    if self.is_frontier_point(n_cell) :
                        self.new_frontier.append(n_cell)
                        self.adj_vector = self.get_neighbors(cur_pos)
                        
                        for i in range(0,len(self.adj_vector)) :
                            if self.adj_vector[i] < self.map_size and self.adj_vector[i] >= 0 :
                                if (self.adj_vector[i] not in self.cell_states.keys() or
                                    (self.cell_states[self.adj_vector[i]] != self.FRONTIER_OPEN_LIST and
                                    self.cell_states[self.adj_vector[i]] != self.FRONTIER_CLOSE_LIST and
                                    self.cell_states[self.adj_vector[i]] != self.MAP_CLOSE_LIST)) :
                                    
                                    if self.map.data[self.adj_vector[i]] != 100 :
                                        self.q_f.insert(0, self.adj_vector[i])
                                        self.cell_states[self.adj_vector[i]] = self.FRONTIER_OPEN_LIST
                                        
                    self.cell_states[n_cell] = self.FRONTIER_CLOSE_LIST
                if len(self.new_frontier) > 2 :
                    frontiers.append(self.new_frontier)
                   
                for i in range(0, len(self.new_frontier)) :
                    self.cell_states[self.new_frontier[i]] = self.MAP_CLOSE_LIST
                    
            self.adj_vector = self.get_neighbors(cur_pos)
            
            for i in range(0, len(self.adj_vector)) :
                if self.adj_vector[i] < self.map_size and self.adj_vector[i] >= 0 :
                    if (self.adj_vector[i] not in self.cell_states.keys() or
                        (self.cell_states[self.adj_vector[i]] != self.MAP_OPEN_LIST and 
                        self.cell_states[self.adj_vector[i]] != self.MAP_CLOSE_LIST)):
                        
                        self.v_neighbors = self.get_neighbors(self.adj_vector[i])
                        map_open_neighbor = False
                    
                        for j in range(0, len(self.v_neighbors)) :
                            if self.v_neighbors[j] < self.map_size and self.v_neighbors[j] >= 0 :
                                if self.map.data[self.v_neighbors[j]] < self.OCC_THRESHOLD and self.map.data[self.v_neighbors[j]] >= 0 :
                                    map_open_neighbor = True
                                    break
                    
                        if map_open_neighbor :
                            self.q_m.insert(0, self.adj_vector[i])
                            self.cell_states[self.adj_vector[i]] = self.MAP_OPEN_LIST
                        
            self.cell_states[cur_pos] = self.MAP_CLOSE_LIST

        l = {}
        for i in range(0,len(frontiers)) :
            l[len(frontiers[i])] = frontiers[i]

        # Publish Frontiers
        cells = []
        for i in range(0, len(frontiers)) :
            for j in range(0, len(frontiers[i])) :
                x_offset = int(math.floor(self.map.info.origin.position.x * 20))
                x = self.get_column_from_offset(frontiers[i][j]) + x_offset
                y = self.get_row_from_offset(frontiers[i][j]) + int(math.floor(self.map.info.origin.position.y * 20))
                cell = Cell(x, y)
                cells.append(cell)
            publishFrontier(cells)
        
        return frontiers
    

    
    def get_neighbors(self, position):
        n_array = []
        n_array.append(position - self.map_width - 1)
        n_array.append(position - self.map_width)
        n_array.append(position - self.map_width + 1)
        n_array.append(position - 1)
        n_array.append(position + 1)
        n_array.append(position + self.map_width - 1)
        n_array.append(position + self.map_width)
        n_array.append(position + self.map_width + 1)
        return n_array
    
    def is_frontier_point(self, point) :
        if self.map.data[point] != 0 :
            return False
        locations = self.get_neighbors(point)
        found = 0
        for i in range(0, len(locations)) :
            if locations[i] < self.map_size and locations[i] >= 0 :
                if self.map.data[locations[i]] > self.OCC_THRESHOLD :
                    return False
                if self.map.data[locations[i]] == -1 :
                    found += 1
                    if found == 1 :
                        return True
        return False
    
    def makeCentroid(self, frontier):
        sumx = 0
        sumy = 0
        count = 0
        for i in range(0, len(frontier)) :
            x = frontier[i] % self.map_width + int(math.floor(self.map.info.origin.position.x*20))
            y = int(math.floor(frontier[i] / self.map_width + int(math.floor(self.map.info.origin.position.y*20))))
            sumx += x
            sumy += y
            count += 1
        centroid = Cell( sumx/count, sumy/count)
        
        if (self.map_width * centroid.y + centroid.x < 0 or
            self.map_width * centroid.y + centroid.x >= self.map_size or
            not self.map.data[self.map_width * centroid.y + centroid.x] == 0) :

            dists = {}
            for i in range(0, len(frontier)) :
                x = frontier[i] % self.map_width + int(math.floor(self.map.info.origin.position.x*20))
                y = int(math.floor(frontier[i] / self.map_width + int(math.floor(self.map.info.origin.position.y*20))))
                dists[math.sqrt(math.pow(centroid.x - x, 2)+math.pow(centroid.y - y,2))] = (x,y)
            key = min(dists.keys())
            centroid = Cell( dists[key][0], 
                             dists[key][1] )
        publishCentroid([centroid])
        print centroid.x, centroid.y
        #publishGoal((centroid.x * 0.05) + 0.025, (centroid.y * 0.05) + 0.025, 0)
            
    def get_column_from_offset(self, i):
        return int(math.floor(i % self.map_width))
    
    def get_row_from_offset(self, i):
        return int(math.floor(i / self.map_width))
        
f = Frontier_Exploration()



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
    goal.header.frame_id = 'map'
    goal.pose.position.x = xPos
    goal.pose.position.y = yPos
    goal.pose.position.z = 0
    goal.pose.orientation.w =1
    
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
    
def publishCentroid(listOfCells):
    pub = rospy.Publisher('/team2/centroid', GridCells)
    
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
    
def publishFrontier(listOfCells):
    pub = rospy.Publisher('/team2/frontier', GridCells)
    
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

def publishDockingPoints(listOfGroups):
    pub = rospy.Publisher('/team2/docking_points', GridCells)
    
    mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
    mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.05
    cells.cell_height = 0.05
    for i in range(0, len(listOfGroups)):
        for j in range(0, len(listOfGroups[i].dockingPoints)):
            point = Point()
            point.x = ((listOfGroups[i].dockingPoints[j].x) * 0.05) + 0.025
            point.y = ((listOfGroups[i].dockingPoints[j].y) * 0.05) + 0.025
            point.z = 0
            cells.cells.append(point)
    pub.publish(cells)  
    

#Subscriber Callback functions
def staticMapCallBack(data):
    global staticMapGrid
    global f
    staticMapGrid = data
    f.map = data
    f.map_width = f.map.info.width
    f.map_height = f.map.info.height
    f.map_size = f.map_height * f.map_width
    f.pose = ((f.y - int(math.floor(data.info.origin.position.y * 20))) * data.info.width) + f.x - int(math.floor(data.info.origin.position.x * 20))
    frontiers = f.wfd()
    fs = []
    for i in range(0,len(frontiers)) :
        fs.extend(frontiers[i])
    centroid = f.makeCentroid(fs)
    
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
    global f
    
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    g.xPos = px
    f.x = int(math.floor(data.pose.pose.position.x * 20))
    f.y = int(math.floor(data.pose.pose.position.y * 20))
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
#        cells = g.evaluatePoints()
#        dockCells = g.evaluateDockingPoints()
#        publishCells(cells)
#        g.evaluateGroups(cells, dockCells)
#        oldData = globalCostMapGrid.data
#        while oldData == globalCostMapGrid.data:
#            rand = 1
#        print 'Costmap Updated'
#        rand = 0
