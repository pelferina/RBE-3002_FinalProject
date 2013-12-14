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
from collections import deque

class Global:
    def __init__(self):
        self.StaticMapGrid = OccupancyGrid()
        self.MapGrid = OccupancyGrid()
        self.costMapGrid = OccupancyGrid()
        self.mapWidth = 0
        self.mapHeight = 0
        self.mapOriginX = 0
        self.mapOriginY = 0
        self.threshold = 50
        self.groupCount = 0
        self.allGroups = []
        self.largeGroups = []
        self.xPos = 0
        self.yPos = 0
        self.theta = 0
    #evaluatePoints: can be used to find points containing obstacles in the global costmap
    def evaluatePoints(self):        
        potentialPoints = []
        for i in range(0, mapWidth):
            for j in range(0, mapHeight):
                index = (i * mapWidth) + j
                index_exist = False
                for ind,val in enumerate(self.costMapGrid.data):
                    if ind == index:
                        index_exist = True
                if index_exist and self.costMapGrid.data[index] > self.threshold:
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

class Frontier_Exploration:
        def __init__(self):
            frontiers = [] 
            #Cell state list for map/frontier open/closed
            self.map_size = map_height * map_width 
            self.cell_states = {}
            self.q_m = dequeue()
            self.q_f = dequeue()  
            self.pose = 0      
            self.adj_vector = [None]*g.N_S
            self.v_neighbours = [None]*g.N_S
            self.MIN_FOUND = 1 
            self.MAP_OPEN_LIST = 1
            self.MAP_CLOSE_LIST = 2
            self.FRONTIER_OPEN_LIST = 3
            self.FRONTIER_CLOSE_LIST = 4
            self.OCC_THRESHOLD = 10
            self.N_S = 8
        def wfd(self):    
            self.q_m.append(pose)
            self.cell_states[pose] = g.MAP_OPEN_LIST
            self.adj_vector[g.N_S] = 0
            self.v_neighbours[g.N_S] = 0
            #ROS_INFO("wfd 1") 
            while not len(self.q_m) == 0:
                #ROS_INFO("wfd 2") 
                cur_pos = self.q_m[0] 
                self.q_m.pop() 
                #ROS_INFO("cur_pos: %d, cell_state: %d",cur_pos, cell_states[cur_pos]) 
                #Skip if map_close_list
                if self.cell_states[cur_pos] == MAP_CLOSE_LIST:
                        continue 
                if self.is_frontier_point(map, cur_pos, map_size, map_width):
                        q_f = dequeue() 
                        new_frontier = dequeue()  
                        q_f.append(cur_pos) 
                        cell_states[cur_pos] = g.FRONTIER_OPEN_LIST 
                        #Second BFS
                        while not len(q_f == 0):
                                #ROS_INFO("wfd 3") 
                                #ROS_INFO("Size: %d", q_f.size()) 
                                n_cell = self.q_f[0] 
                                self.q_f.pop() 
                                if cell_states[n_cell] == MAP_CLOSE_LIST or cell_states[n_cell] == FRONTIER_CLOSE_LIST:
                                        continue         
                                if is_frontier_point(map, n_cell, map_size, map_width):
                                        #ROS_INFO("adding %d to frontiers", n_cell) 
                                        new_frontier.append(n_cell) 
                                        self.get_neighbours(adj_vector, cur_pos, map_width)                      
                                        for i, elem in enumerate(self.adj_vector):
                                            if (self.adj_vector[i] < map_size and adj_vector[i] >= 0):
                                                    if (not self.cell_states[adj_vector[i]] == FRONTIER_OPEN_LIST) and (not self.cell_states[adj_vector[i]] == FRONTIER_CLOSE_LIST) and (not self.cell_states[adj_vector[i]] == MAP_CLOSE_LIST):
                                                        #ROS_INFO("wfd 4") 
                                                        if (map.data[adj_vector[i]] != 100):
                                                            self.q_f.append(adj_vector[i]) 
                                                            self.cell_states[adj_vector[i]] = FRONTIER_OPEN_LIST
                                cell_states[n_cell] = FRONTIER_CLOSE_LIST            
                        if len(new_frontier) > 2:
                                frontiers.append(new_frontier)                         
                        #ROS_INFO("WFD 4.5") 
                        for i, elem in enumerate(new_frontier):
                            self.cell_states[new_frontier[i]] = MAP_CLOSE_LIST 
                            #ROS_INFO("WFD 5") 
                self.get_neighbours(self.adj_vector, cur_pos, map_width) 

                for i, elem in enumerate(self.adj_vector):
                    #ROS_INFO("wfd 6") 
                    if self.adj_vector[i] < self.map_size and  self.adj_vector[i] >= 0:
                        if (self.cell_states[adj_vector[i]] != MAP_OPEN_LIST and self.cell_states[adj_vector[i]] != MAP_CLOSE_LIST):
                            get_neighbours(v_neighbours, adj_vector[i], map_width) 
                            map_open_neighbor = False 
                            for j,elem in enumerate(v_neighbours):
                                if(v_neighbours[j] < map_size and v_neighbours[j] >= 0):
                                    if (map.data[v_neighbours[j]] < OCC_THRESHOLD and map.data[v_neighbours[j]] >= 0):
                                        map_open_neighbor = true 
                                        break 
                            if(map_open_neighbor):
                                    q_m.push(adj_vector[i]) 
                                    cell_states[adj_vector[i]] = MAP_OPEN_LIST 
                #ROS_INFO("wfd 7") 
                self.cell_states[cur_pos] = g.MAP_CLOSE_LIST 
                #ROS_INFO("wfd 7.1") 
        #ROS_INFO("wfd 8") 
        return frontiers 
    get_neighbours(n_array, position, map_width) {
            n_array[0] = position - map_width - 1 
            n_array[1] = position - map_width 
            n_array[2] = position - map_width + 1 
            n_array[3] = position - 1 
            n_array[4] = position + 1 
            n_array[5] = position + map_width - 1 
            n_array[6] = position + map_width 
            n_array[7] = position + map_width + 1 
    }
    
    get_big_neighbours(n_array[], position, map_width) {
            n_array[0] = position - map_width - 1 
            n_array[1] = position - map_width 
            n_array[2] = position - map_width + 1 
            n_array[3] = position - 1 
            n_array[4] = position + 1 
            n_array[5] = position + map_width - 1 
            n_array[6] = position + map_width 
            n_array[7] = position + map_width + 1 
    
            n_array[8] = position - (map_width * 2) - 2 
            n_array[9] = position - (map_width * 2) - 1
            n_array[10] = position - (map_width * 2)
            n_array[11] = position - (map_width * 2) + 1
            n_array[12] = position - (map_width * 2) + 2
            n_array[13] = position - 2
            n_array[14] = position + 2
            n_array[15] = position + (map_width * 2) - 2
            n_array[16] = position + (map_width * 2) - 1
            n_array[17] = position + (map_width * 2)
            n_array[18] = position + (map_width * 2) + 1
            n_array[19] = position + (map_width * 2) + 2
            n_array[20] = position + (map_width) + 2
            n_array[21] = position + (map_width) - 2
            n_array[22] = position - (map_width) + 2
            n_array[23] = position - (map_width) - 2
    
    is_frontier_point(map, point, map_size, map_width) 
        #The point under consideration must be known
        if(map.data[point] != -1) 
                return False
        int locations[N_S] 
        get_neighbours(locations, point, map_width)
        found = 0
        for i = 0  i < N_S  i++:
            if(locations[i] < map_size and locations[i] >= 0) 
                # None of the neighbours should be occupied space.                
                if map.data[locations[i]] > OCC_THRESHOLD:
                        return False 
                
                #At least one of the neighbours is open and known space, hence frontier point
                if(map.data[locations[i]] == 0) 
                        found++ 
                        if(found == MIN_FOUND)
                                return True
            return False
    
    int get_row_from_offset(int offset, int width):
            return offset / width 
    
    
    int get_column_from_offset(int offset, int width):
            return offset % width         
#Group class: for storing cells in a given group
class Group:
    def __init__(self, num):
        self.index = num
        self.cells = []
        self.centerX = 0
        self.centerY = 0
        self.timer = rospy.Duration(0) #300 seconds = 5 minutes
        rospy.Timer(rospy.Duration(2), self.timer_callback)
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
    def resetTimer(self):
        self.timer = rospy.Duration(0)
    def timer_callback(self, event):
        print 'Timer called at ' + str(event.current_real)
        self.timer = rospy.Duration(self.timer + 2)

class Subscriber:
    
    def __init__(self):
        self.getOdomData()
        self.getMapData()
        self.getStaticMapData()
        self.getGlobalCostmapData()
    #Subscriber Functions
    def getOdomData(self):
        sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    
    def getMapData(self):
        sub = rospy.Subscriber("/map", OccupancyGrid, MapCallBack)
    
    def getGlobalCostmapData(self):
        sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapCallBack)
    
    def getStaticMapData(self):
        sub = rospy.Subscriber("/static/map", OccupancyGrid, StaticMapCallBack)
        

class Publisher:
    #Publisher Functions
    #publishGoal: publishes the x position, y position and angle for the robot to move to
    def publishGoal(self, xPos, yPos, angle):
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
    #publishStart: publishes the current x position, y position and angle for the robot to navigate using amcl
    def publishStart(self, xPos, yPos, angle):
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)
        start = PoseWithCovarianceStamped
        start.pose.pose.position.x = xPos
        start.pose.pose.position.y= yPos
        #determine theta eventually
        pub.publish(start)
#Subscriber Callback functions
def MapCallBack(data):
    global g
    g.MapGrid = data
    g.mapWidth = data.info.width
    g.mapHeight = data.info.height
    g.mapOriginX = int(math.floor(g.MapGrid.info.origin.position.x * 20))
    g.mapOriginY = int(math.floor(g.MapGrid.info.origin.position.y * 20))
def StaticMapCallBack(data):
    global g
    g.StaticMapGrid = data
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
    #print len(g.costMapGrid.data)
    while(1):
        cells = g.evaluatePoints()
        p.publishCells(cells)
        for i in range(0, len(cells)):
            print '[', cells[i].x, ' ', cells[i].y, ']'
