#!/usr/bin/env python
import rospy
import roslib

#roslib.load_manifest('Final_Project')

import time
import math
import numpy

from numpy import *

#Message Type Imports
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from random import choice
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

from std_msgs.msg import Header

from kobuki_msgs.msg import BumperEvent

staticMapGrid = OccupancyGrid()
localCostMapGrid = OccupancyGrid()
globalCostMapGrid = OccupancyGrid()
globalCostMapUpdate = OccupancyGridUpdate()

updateCount = 0

class DriveToGroups:
    def __init__(self):
        self.trajectory = []
        self.way_points = [(3.5367, 0.9897, 3.067), (-0.76956, 1.70658, -1.81), 
                           (0.647343, 0.319575, -1.554), (0.84895, -2.084891, 0.843), 
                           (-1.389, -0.452, 0.825)]
    def setGoal(self, Point, Orientation, x2, y2):
        publishGoal(Point[0], Point[1], Orientation, x2, y2)
        
    #finds the distance between any two points
    def dist_between(self, point1, point2):  
        distance = sqrt( ((point2[1] - point1[1])**2) + ((point2[0] - point1[0])**2))
        #print "distance between", distance
        return round(distance,4)  
    #adds points to trajectory  
    def add_to_trajectory(self, group1, group2, group3):
        self.trajectory.append(group1)
        self.trajectory.append(group2)
        self.trajectory.append(group3)
    #def execute_
    #execute trajectory to visit the groups
    def execute_Trajectory(self):
        global g
        if len(self.trajectory) == 0:
            return
        print "trajectory", self.trajectory
        #for i in range(0, len(self.trajectory)):
            #print "i,", i
        while True:
            if len(self.trajectory) == 0:
                break 
            self.setGoal(self.trajectory[0], g.theta, ((self.trajectory[0][0] * 0.05) + 0.025), ((self.trajectory[0][1]* 0.05) + 0.025))
            #remove goal from trajectory
            self.trajectory.remove(self.trajectory[0])
            #play sound
            print "serving guests"
            #serve guest 
            rospy.sleep(2)
            print "guest served"
            #play sound
            
    def serve(self):
        global g
        #find priority of groups
        g.priority_queue()     
        mapWidth = staticMapGrid.info.width
        mapHeight = staticMapGrid.info.height
        mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
        mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))

        robot_docking_points = (int(math.floor(g.xPos*20)), int(math.floor(g.yPos*20)))
        print len(g.priority_groups)
        #determine shortest distance between current group i and the next group i+1
        docking_distance = {}
        for i in range(0, len(g.priority_groups)):
            #print g.priority_groups[i].dockingPoints
            for l in range(0, len(g.priority_groups[i].dockingPoints)):
                if l == len(g.priority_groups[i].dockingPoints) - 1:
                    break
                current_x = g.priority_groups[i].dockingPoints[l].x #Extract cells
                current_y = g.priority_groups[i].dockingPoints[l].y
                docking_point = ()
                docking_point = docking_point + ((current_x, current_y),)
                docking_point = docking_point + (robot_docking_points,)
                docking_distance[docking_point] = self.dist_between(robot_docking_points, docking_point[0])
            #print docking_distance
            min_distance_docking_points = min(docking_distance, key = docking_distance.get)
            #pick one set of points to go to between the current and the next group
            current_docking_points = (min_distance_docking_points)  #choice
            print "minimum between group and robot: ", current_docking_points
            self.trajectory.append(current_docking_points[0])
            self.execute_Trajectory()

            
        
    def serve_all(self):
        global g
        #find priority of groups
        g.priority_queue()     
        docking_distance = {}
        if len(g.priority_groups) == 0:
            return
        for i in range(0, len(g.priority_groups)):
            if i == len(g.priority_groups) - 1:
                break
            if i+1 in range(0, len(g.priority_groups)) and i+2 in range(0, len(g.priority_groups)):
                #find distance between each docking point of the next two consecutive groups
                for j in range(0, len(g.priority_groups[i+2].dockingPoints)):
                    if j == len(g.priority_groups[i+2].dockingPoints) - 1 or j == len(g.priority_groups[i+1].dockingPoints) - 1:
                        break
                    docking_point = ()
                    next_x = g.priority_groups[i+2].dockingPoints[j].x  #Extract cells 
                    next_y = g.priority_groups[i+2].dockingPoints[j].y 
                    docking_point = docking_point + ((next_x, next_y),)
                    for k in range(0, len(g.priority_groups[i+1].dockingPoints)):
                        if k == len(g.priority_groups[i+1].dockingPoints) - 1:
                            break
                        second_next_x = g.priority_groups[i+1].dockingPoints[k].x #Extract cells 
                        second_next_y = g.priority_groups[i+1].dockingPoints[k].y
                        docking_point = docking_point + ((second_next_x, second_next_y),)
                        
                        docking_distance[docking_point] = self.dist_between(docking_point[0], docking_point[1])
                        docking_point = ()
                        docking_point = docking_point + ((next_x, next_y),)
                        
                     
                #find docking points with minimum distance between the next two consecutive groups
                min_distance_docking_points = min(docking_distance, key = docking_distance.get)
                print "minimum between group 3 and 2: ", min_distance_docking_points
                #pick one set of points to go to between the next two groups 
                next_docking_points = (min_distance_docking_points) #choice
                
                #determine shortest distance between current group i and the next group i+1
                docking_distance = {}
                #print g.priority_groups[i].dockingPoints
                for l in range(0, len(g.priority_groups[i].dockingPoints)):
                    if l == len(g.priority_groups[i].dockingPoints) - 1:
                        break
                    current_x = g.priority_groups[i].dockingPoints[l].x #Extract cells
                    current_y = g.priority_groups[i].dockingPoints[l].y
                    docking_point = ()
                    docking_point = docking_point + ((current_x, current_y),)
                    docking_point = docking_point + (next_docking_points[1],)
                    docking_distance[docking_point] = self.dist_between(next_docking_points[1], docking_point[0])
                #print docking_distance
                min_distance_docking_points = min(docking_distance, key = docking_distance.get)
                #pick one set of points to go to between the current and the next group
                current_docking_points = (min_distance_docking_points)  #choice
                print "minimum between group 1 and 2: ", current_docking_points
                
                self.add_to_trajectory(current_docking_points[0], current_docking_points[1], next_docking_points[0])
                #break if end of list reached
                if i+2 == len(g.priority_groups):
                    break
        self.execute_Trajectory()
        
class Global:
    def __init__(self):
        self.threshold = 75
        self.groupCount = 0
        self.allGroups = []
        self.largeGroups = []
        self.priority_groups = []
        self.header = Header()
        self.xPos = 0
        self.yPos = 0
        self.theta = 0
        self.quat = Quaternion()
        self.groupThreshold = 15
    def priority_queue(self):
        self.priority_groups = self.allGroups
        for i in range(0, len(self.allGroups)):
            self.allGroups[i].timer()
        self.priority_groups = sorted(self.priority_groups, key = lambda group: group.time_duration)
        #print len(self.priority_groups)
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
                            if staticMapGrid.data[((y + yRange) * mapWidth) + x + xRange] != 0:
                                staticValue = 1
                                break
                        else:
                            continue
                        break
                    if staticValue == 0:
                        potentialPoints.append(Cell(x + mapOriginX, y + mapOriginY))
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
                            if ((self.allGroups[r].cells[s].x < self.allGroups[p].cells[q].x + 5 and
                                 self.allGroups[r].cells[s].x > self.allGroups[p].cells[q].x - 5) and
                                (self.allGroups[r].cells[s].y < self.allGroups[p].cells[q].y + 5 and
                                 self.allGroups[r].cells[s].y > self.allGroups[p].cells[q].y - 5)):
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
 
        self.largeGroups = self.allGroups            
        
        #Publish the groups to RViz
        publishGroups(self.allGroups)
    
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
        self.dockingPoints = []
        self.time_found = rospy.get_time()
        self.time_duration = 0
        #self.timer = rospy.Duration(300) #300 seconds = 5 minutes
        #rospy.Timer(rospy.Duration(2), self.timer_callback)
        #rospy.Timer(rospy.Duration(15), self.timer_callback)
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
    def timer(self):
        self.time_duration = rospy.get_time() - self.time_found
    #def resetTimer(self):
    #    self.timer = rospy.Duration(0)
    #def timer_callback(self, event):
        #print 'Timer called at ' + str(event.current_real)
     #   self.timer = self.timer - rospy.Duration(15)
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
   # self.getOdomData()
   # self.getStaticMapData()
   # self.getGlobalCostmapData()
    #Subscriber Functions

def getOdomData():
    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, odomCallback)
    
def getStaticMapData():
    sub = rospy.Subscriber("/map", OccupancyGrid, staticMapCallBack)
        
def getGlobalCostmapData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapCallBack)

def getGlobalUpdateData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, globalCostmapUpdate)

def getLocalCostmapData():
    sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, localCostmapCallBack)
#finds the distance between any two points
def dist_between_goals(point1, point2):  
    distance = math.sqrt( ((point2[1] - point1[1])**2) + ((point2[0] - point1[0])**2))  
    return distance
#class Publisher:
    #Publisher Functions
    #publishGoal: publishes the x position, y position and angle for the robot to move to
def publishGoal(xPos, yPos, angle, x2, y2):
    print "publishing goal"
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    #goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = (xPos * 0.05) + 0.025
    goal.pose.position.y = (yPos * 0.05) + 0.025
    goal.pose.position.z = 0
    goal.pose.orientation.w = 1
    r = rospy.Rate(.7)
    #wait for goal to be reached
    while True:
        print dist_between_goals((g.xPos, g.yPos), (x2,y2)) 
        if dist_between_goals((g.xPos, g.yPos), (x2,y2)) > 0.5:
            pub.publish(goal)
            print "goal reached"
        else:
            break
    print goal.pose.position.x, goal.pose.position.y
    
def publishWayPoints(xPos, yPos, angle, w_p):
    print "publishing waypoint"
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    goal = PoseStamped()
    
    initTime = rospy.get_time()
    
    mapWidth = globalCostMapGrid.info.width
    
    mapOriginX = int(math.floor(globalCostMapGrid.info.origin.position.x * 20))
    mapOriginY = int(math.floor(globalCostMapGrid.info.origin.position.y * 20))
    
    goal.header = g.header
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = xPos
    goal.pose.position.y = yPos
    goal.pose.position.z = 0
    quat = quaternion_from_euler(0, 0, angle)
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]
    
    valX = int(math.floor(d.way_points[w_p][0] * 20)) - mapOriginX
    valY = int(math.floor(d.way_points[w_p][1] * 20)) - mapOriginY
    
    r = rospy.Rate(.7)
    while math.sqrt(math.pow(g.xPos - d.way_points[w_p][0], 2) + math.pow(g.yPos - d.way_points[w_p][1], 2)) > 0.5:
        print "d:",math.sqrt(math.pow(g.xPos - d.way_points[w_p][0], 2) + math.pow(g.yPos - d.way_points[w_p][1], 2))
        finalTime = rospy.get_time()
        changeTime = finalTime - initTime
        if(globalCostMapGrid.data[(valX * mapWidth) + valY] > g.threshold) or (changeTime > 120):
            break
        else:
            pub.publish(goal)


def publishInitialPose(xPos, yPos, angle):
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)
    start = PoseWithCovarianceStamped()
    start.header = g.header
    start.pose.pose.position.x = xPos
    start.pose.pose.position.y = yPos
    start.pose.pose.orientation = g.quat
    pub.publish(start)

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
    
   # cells = g.evaluatePoints()
    #dockCells = g.evaluateDockingPoints()
    #publishCells(cells)
    #publishDockingPoints(dockCells)
    #g.evaluateGroups(cells, [Cell(0, 0)])
    
def odomCallback(data):
    global g
    
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    g.header = data.header
    g.quat = quat
    g.xPos = px
    g.yPos = py
    g.theta = yaw * 180.0 / math.pi

if __name__ == '__main__':
    rospy.init_node('PartyServer', anonymous=True)
    global g
    d = DriveToGroups()
    getOdomData()
    getStaticMapData()
    getLocalCostmapData()
    getGlobalCostmapData()
    getGlobalUpdateData()
    
    while(len(staticMapGrid.data) == 0 or
          len(globalCostMapGrid.data) == 0):
        rand = 0
    
    mapWidth = staticMapGrid.info.width
    
    mapOriginX = int(math.floor(staticMapGrid.info.origin.position.x * 20))
    mapOriginY = int(math.floor(staticMapGrid.info.origin.position.y * 20))
    w_p = 0
    while(1):
        cells = g.evaluatePoints()
        dockCells = g.evaluateDockingPoints()
        publishCells(cells)
        publishDockingPoints(dockCells)
        g.evaluateGroups(cells, dockCells)
        d.serve_all()
        #publishWayPoints(d.way_points[w_p][0], d.way_points[w_p][1], d.way_points[w_p][1], w_p)
        w_p += 1
        