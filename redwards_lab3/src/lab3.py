#!/usr/bin/env python

#Author: Robert Edwards
#Lab 3 Code

#Imports

import rospy
import roslib

roslib.load_manifest('redwards_lab2')

import time
import math

from tf.transformations import euler_from_quaternion



#Message Types
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

mapData = 0
width = 0
height = 0

xInit = 0
yInit = 0
thetaInit = 0

xEnd = 0
yEnd = 0
thetaEnd = 0

noParent = 0

def mapCallBack(data):
    global mapData
    global width
    global height
    mapData = data.data
    width = data.info.width
    height = data.info.height

def startCallBack(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi

def endCallBack(data):
    px = data.pose.position.x
    py = data.pose.position.y
    quat = data.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi

def publishTwist(position):
    pub = rospy.Publisher('~gridCellData', GridCells)
    cells = GridCells()
    cells.cell_width = 1
    cells.cell_height = 1
    for i in range(0, len(position) - 1):
	cells.cells.append([position[i][0], position[i][1], 0])
    pub.publish(cells)

def getMapData():
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)

def getStartData():
    sub = rospy.Subscriber("/initialPose", PoseWithCovarianceStamped, startCallBack)

def getEndData():
    sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, endCallBack)

#node class for storing data for a given cell
class node:
    position = [0,0]
    parent = 0
    g = 0 #distance traveled so far
    h = 0 #estimated distance from node to end
    f = 0 #total estimated cost, or sum of g and h
    def __init__(self, xPos, yPos, parent, g, h, f):
	self.position = [xPos, yPos]
	self.parent = parent
	self.g = g
	self.h = h
	self.f = f
    #redefine the equality condition for a node
    def __eq__(self, node):
	#if the positions are the same, it's the same node
	if (self.position[0] == node.position[0]) and (self.position[0] == node.position[0]):
	    return true
	else:
	    return false
    #obtains the frontiers for a given node at a resolution of 0.05 cm/cell, with diagonals
    def getFrontiers(self): 
	frontiers = []
	frontiers[0] = node(self.position[0] - 0.05, self.position[1], self.g + 0.05, self, 0, 0)
	frontiers[1] = node(self.position[0] + 0.05, self.position[1], self.g + 0.05, self, 0, 0)
	frontiers[2] = node(self.position[0] - 0.05, self.position[1] + 0.05, \
			    self.g + math.sqrt(2 * math.pow(0.05, 2)), self, 0, 0)
	frontiers[3] = node(self.position[0] + 0.05, self.position[1] + 0.05, \
			    self.g + math.sqrt(2 * math.pow(0.05, 2)), self, 0, 0)
	frontiers[4] = node(self.position[0] - 0.05, self.position[1] - 0.05, \
			    self.g + math.sqrt(2 * math.pow(0.05, 2)), self, 0, 0)
	frontiers[5] = node(self.position[0] + 0.05, self.position[1] - 0.05, \
			    self.g + math.sqrt(2 * math.pow(0.05, 2)), self, 0, 0)
	frontiers[6] = node(self.position[0], self.position[1] + 0.05, self.g + 0.05, self, 0, 0)
	frontiers[7] = node(self.position[0], self.position[1] - 0.05, self.g + 0.05, self, 0, 0)
    
def aStarMapping(startx, starty, endx, endy)
    closedNodes = []
    openNodes = []
    
    path = []
    
    initNode = node(startx, starty, noParent, 0, 0, 0)
    initNode.h = math.sqrt(math.pow(endx - initNode.position[0], 2) + math.pow(endy - initNode.position[1], 2))
    initNode.f = initNode.g + initNode.h
    openNodes.insert(initNode, 0)
    while(len(openNodes) != 0):
	frontiers = openNodes[0].getFrontiers()
	closedNodes.append(openNodes[0])	
	del openNodes[0]
	#If the frontier value is unobstructed and unexplored, find the h, f and add it to openNodes
        for i in range(0, len(frontiers) - 1):
	    if !(mapData[20 * (frontiers[i].position[1] * width + frontiers[i].position[x])] == -1) or \
	       !(mapData[20 * (frontiers[i].position[1] * width + frontiers[i].position[x])] == 100) or \
	       !(frontiers[i] in closedNodes):
		frontiers[i].h = math.sqrt(math.pow(endx - frontiers[i].position[0], 2) + \
				 math.pow(endy - frontiers[i].position[1], 2))
		frontiers[i].f = frontiers[i].g + frontiers[i].h
		openNodes.append(frontiers[i])
	#sort by f value to bring lowest to index 0
	sorted(openNodes, key = lambda node: node.f) 
	if the most recent node is the target, 
	if(openNodes[0].position[0] == endx) and (openNodes[0].positon[1] == endy):
	    node = openNodes[0]
	    while(node.parent != noParent):
		path.insert(node.position, 0)
		node = node.parent
	    path.insert(initNode.position)
	    break

if __name__ == '__main__' :
    rospy.init_node('redwards_lab3', anonymous=True)
    while(1):
	publishCells([[1, 1], [2, 2]])
