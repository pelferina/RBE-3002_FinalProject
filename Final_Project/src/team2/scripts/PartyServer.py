#!/usr/bin/env python
import rospy
import roslib

roslib.load_manifest('redwards_lab3')

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

#Global Declarations

mapGrid = OccupancyGrid()

costMapGrid = OccupancyGrid()

threshold = 75

allGroups = []
largeGroups = []


#Subscriber Functions
def getOdomData():
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)

def getMapData():
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    
def getGlobalCostmapData():
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapCallBack)

#Subscriber Callback functions
def mapCallBack(data):
    global mapGrid
    mapGrid = data
    
def globalCostmapCallBack(data):
    global costMapGrid
    costMapGrid = data
    
    
def odomCallback(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xPos
    global yPos
    global theta
    xPos = px
    yPos = py
    theta = yaw * 180.0 / math.pi 
    
#Publisher functions
def publishTwist(u,w):
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    twist = Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    pub.publish(twist)

def publishGoal(xPos, yPos, angle):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    goal = PoseStamped()
    goal.pose.position.x = xPos
    goal.pose.position.y = yPos
    #determine theta eventually
    pub.publish(goal)


#Cell class for storing position data
class Cell:
    def __init__(self, xPos, yPos):
        self.x = xPos
        self.y = yPos
    
    def __eq__(self, cell):
        if self.x == cell.x and self.y == cell.y:
            return True
        else:
            return False

#Group class for storing cells in a given group
class Group:
    def __init__(self, num):
        self.numGroup = num
        self.cells = []
        
    def addCelltoGroup(self, cell):
        self.cells.append(cell)
        

#evaluatePoints: can be used to find points containing obstacles in the global Costmap
def evaluatePoints(staticMap, costMap):
    mapWidth = staticMap.info.width
    mapHeight = staticMap.info.height
    mapOriginX = int(math.floor(costMap.info.origin.position.x * 20))
    mapOriginY = int(math.floor(costMap.info.origin.position.y * 20))
    potentialPoints = []
    global largeGroups
    global allGroups
    for i in range(0, mapWidth):
        for j in range(0, mapHeight):
            if costMap.data[(i * mapWidth) + j] > threshold:
                staticValue = 0
                for k in range(-3, 4):
                    for l in range(-3, 4):
                        staticValue += staticMap.data[((i + k) * mapWidth) + j + k]
                if staticValue <= 0:
                    points.append([j - mapOriginX, i - mapOriginY])

def evaluateGroups(potentialPoints):
    for i in range(0, len(potentialPoints)):
        point = 
        



if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
