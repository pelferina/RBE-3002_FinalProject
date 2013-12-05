#!/usr/bin/env python

#Author: Robert Edwards
#Lab 3 Code

#TODO: Reformat, wait for non-buggy turtlebot to do testing.

#Library Imports

import rospy
import roslib

roslib.load_manifest('redwards_lab3')

import time
import math

from numpy import *

#Message Type Imports
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent

grid = OccupancyGrid()

xPos = 0
yPos = 0
theta = 0

mapData = []
width = 0
height = 0

xInit = 0
yInit = 0
thetaInit = 0

xEnd = 0
yEnd = 0
thetaEnd = 0

noParent = 0

pubPath = rospy.Publisher("/gridCellPath", GridCells)
pubFrontier = rospy.Publisher("/gridCellFrontier", GridCells)
pubFinal = rospy.Publisher("/gridCellFinal", GridCells)

#Call Back functions

def mapCallBack(data):
    global mapData, grid
    global width
    global height
    global originX
    global originY
    originX = int(math.floor(data.info.origin.position.x * 20))
    originY = int(math.floor(data.info.origin.position.y * 20))
    grid = data
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
    xInit = math.ceiling(xInit * 20) / 20
    yInit = py
    yInit = math.cieling(yInit * 20) / 20
    thetaInit = yaw * 180.0 / math.pi

def endCallBack(data):
    px = data.point.x
    py = data.point.y
    global xEnd
    global yEnd
    xEnd = px
    #xEnd = math.ceiling(xEnd * 20) / 20
    yEnd = py
    #yEnd = math.ceiling(yEnd * 20) / 20

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

#Grid Cell Publishers
def publishPathCells(pathnodes, pub):
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.05
    cells.cell_height = 0.05
    for i in range(0, len(pathnodes)):
	point = Point()
	point.x = pathnodes[i].position[0] * 0.05 + 0.025
	point.y = pathnodes[i].position[1] * 0.05 + 0.025
	point.z = 0
	cells.cells.append(point)
    pub.publish(cells)
    #rospy.sleep(1)

def publishFrontierCells(frontier, pub):
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.05
    cells.cell_height = 0.05
    for i in range(0, len(frontier)):
	point = frontier[i]
	cells.cells.append(point)
    pub.publish(cells)

def publishFinalCells(path, pub):
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.05
    cells.cell_height = 0.05
    for i in range(0, len(path)):
	point = Point()
	point.x = path[i][0] * 0.05 + 0.025 
	point.y = path[i][1] * 0.05 + 0.025
	point.z = 0
	cells.cells.append(point)
    pub.publish(cells)

#Twist message publisher
def publishTwist(u,w):
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    twist = Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    pub.publish(twist)

#Subscriber Functions
def getOdomData():
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)

def getMapData():
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)

def getStartData():
    sub = rospy.Subscriber("/initialPose", PoseWithCovarianceStamped, startCallBack)

def getEndData():
    sub = rospy.Subscriber("/clicked_point", PointStamped, endCallBack)

#Driving Functions:

#driveStraight: drive forward by a certain distance at a given speed
def driveStraight(speed, distance):
    print "straight"
    u = speed
    w = 0
    init_distX = xPos
    print "init x: ", init_distX
    init_distY = yPos
    init_angle = theta
    print "init y: ", init_distY
    print distance * math.cos(math.radians(init_angle))
    print xPos - init_distX
    print distance * math.sin(math.radians(init_angle))
    print yPos - init_distY
    totalDist = 0
    prevDist = [xPos, yPos]
    while totalDist < distance - 0.005:
        #print "trying"
        publishTwist(u, w)
        rospy.sleep(0.05)
        totalDist = math.sqrt(math.pow(xPos - prevDist[0], 2) + math.pow(yPos - prevDist[1], 2))
        print totalDist
    publishTwist(0,0)
    print "done"

#rotate: spins the robot on its axis by a desired angle. 
#Positive denotes right turns and negative denotes left turns
def rotate(angle):
    u = 0
    w = 0.24
    initAngle = theta
    print "Rotating", angle, "degrees"
    print "initial:", initAngle
    desiredAngle = angle + initAngle
    while desiredAngle < -180 or desiredAngle > 180:
        if angle > 0:
            desiredAngle -= 360
        else:
            desiredAngle += 360
    print "Desired:", desiredAngle
    if angle > 0:
        while theta > desiredAngle + 1 or theta < desiredAngle - 1:
            #print theta
            publishTwist(u, w)
            rospy.sleep(0.05)
            #print desiredAngle
    else:
        while theta > desiredAngle + 1 or theta < desiredAngle - 1:
            publishTwist(u, -w)
            #print theta
            rospy.sleep(0.05)
            #print desiredAngle
    publishTwist(0,0)

#Node Class: used for storing node information
class Node:
   
    def __init__(self, xPos, yPos, parent, g, h, f):
	self.position = [xPos, yPos]
	self.parent = parent
	self.g = g
	self.h = h
	self.f = f
    #redefine the equality condition for a node
    def __eq__(self, node):
	#if the positions are the same, it's the same node
	if (self.position[0] == node.position[0]) and (self.position[1] == node.position[1]):
	    return True
	else:
	    return False
    #Obtains the frontiers for a given node at a resolution of 0.05 cm/cell, with diagonals
    def getFrontiers(self): 
	frontiers = []
	frontiers.append(Node(self.position[0] - 1, self.position[1], self, self.g + 1, 0, 0))
	frontiers.append(Node(self.position[0] + 1, self.position[1], self, self.g + 1, 0, 0))
	frontiers.append(Node(self.position[0] - 1, self.position[1] + 1, \
			    self, self.g + math.sqrt(2), 0, 0))
	frontiers.append(Node(self.position[0] + 1, self.position[1] + 1, \
			    self, self.g + math.sqrt(2), 0, 0))
	frontiers.append(Node(self.position[0] - 1, self.position[1] - 1, \
			    self, self.g + math.sqrt(2), 0, 0))
	frontiers.append(Node(self.position[0] + 1, self.position[1] - 1, \
			    self, self.g + math.sqrt(2), 0, 0))
	frontiers.append(Node(self.position[0], self.position[1] + 1, self, self.g + 1, 0, 0))
	frontiers.append(Node(self.position[0], self.position[1] - 1, self, self.g + 1, 0, 0))
	return frontiers

#reconstructPath: recreates the path between two nodes and formats them into a list of 
#two int lists, for x and y positions
def reconstructPath(node):
    path = []
    while(node != None):
        path.append([node.position[0], node.position[1]])
        node = node.parent
    publishFinalCells(path, pubFinal)
    return path

#aStarMapping: develops a path between the given start and end points, which should give 
#the shortest available path.
def aStarMapping(startx, starty, endx, endy):
    
    #Lists for storing nodes
    closedNodes = []
    openNodes = [] 
    frontier = []  
    
    #create an initial and end node for comparisons
    initNode = Node(startx, starty, None, 0, 0, 0)
    endNode = Node(endx, endy, None, 0, 0, 0)
    
    #compute the heuristic and f value of the initial node
    initNode.h = math.sqrt(math.pow(endx - initNode.position[0], 2) + \
		 math.pow(endy - initNode.position[1], 2))
    initNode.f = initNode.g + initNode.h
    openNodes.append(initNode)
    while(len(openNodes) != 0):
        frontiers = openNodes[0].getFrontiers()
        publishFrontierCells(frontier, pubFrontier)	
        currentNode = openNodes[0]
        print openNodes[0].position[0], " ",  openNodes[0].position[1]
        closedNodes.append(openNodes[0])	
        publishPathCells(closedNodes, pubPath)	
        if currentNode.__eq__(endNode):
            print "finish"
            path = reconstructPath(currentNode)
            return path
        del openNodes[0]
	#If the frontier value is unobstructed and unexplored, find the h, f and add it to openNodes
        for i in range(0, len(frontiers)):
            occupiedCount = 0
            for j in range(-4, 5):
                for k in range(-4, 5):
                    occupiedCount += mapData[((frontiers[i].position[1] + j - originY) * width) + (frontiers[i].position[0] + k - originX)]
            print occupiedCount
            if (occupiedCount <= 0):
                if (mapData[(frontiers[i].position[1] - originY) * width + (frontiers[i].position[0] - originX)] != 100 and 
                    frontiers[i] not in closedNodes):
                    frontiers[i].h = math.sqrt(math.pow(endx - frontiers[i].position[0], 2) + 
                                               math.pow(endy - frontiers[i].position[1], 2))
                    frontiers[i].f = frontiers[i].g + frontiers[i].h
                    #print "%f" % frontiers[i].f
                    if(frontiers[i] not in openNodes):
                        openNodes.append(frontiers[i])
                        frontier.append(Point(frontiers[i].position[0] * 0.05 + 0.025, frontiers[i].position[1] * 0.05 + 0.025, 0))
		
	#sort by f value to bring lowest to index 0
        openNodes.sort(key = lambda node: node.f)
        rospy.sleep(0.01)

#computeCentroid: computes the centroid of a series of points by 
#finding the median of the set of frontier nodes (Not 100% functional)
def computeCentroid(frontierPos):
    xAvg = 0
    yAvg = 0
    medianPos = 0
    frontiersPos.sort(key = lambda pos: pos[0])
    if(len(frontierPos) % 2 == 0):
        medianPos = ((len(frontierPos) + 1) / 2) - 1
    else:
        medianPos = (len(frontierPos) / 2) + 1
    xAvg = frontierPos[medianPos][0]
    yAvg = frontierPos[medianPos][1]
    return [xAvg, yAvg]

#driveBot: drives the robot by rotating and driving straight, given a 
def driveBot(nextPos):
    #derive the distance to the desired point, for x and y positional change
    distX = (nextPos[0] * 0.05) - (int(math.floor(xPos * 20)) * 0.05)
    print distX
    distY = (nextPos[1] * 0.05) - (int(math.floor(yPos * 20)) * 0.05)
    print distY
    #derive the angle of the right triangle made by the points
    thetaTriangle = math.atan2(distY, distX)
    #derive the desired angle of the robot
        
    print "desired:", thetaTriangle
    #find the change in angle between the current and desired rotation
    changeAngle = thetaTriangle - math.radians(theta)
    #put the change in angle between -180 and 180 degrees
    print changeAngle * 180 / math.pi
    #rotate the robot by the desired angle, converted to degrees
    rotate(changeAngle * 180 / math.pi)
    
    #drive the robot straight by the desired distance, linear from point to point
    driveStraight(0.07, abs(math.sqrt(math.pow(distX, 2) + math.pow(distY, 2))))
    rospy.sleep(0.1)
    

if __name__ == '__main__' :
    rospy.init_node('redwards_lab3', anonymous=True)
    #Subscriptions
    getOdomData()
    getMapData()
    getEndData()
    #Wait for published end point
    while xEnd == 0 and yEnd == 0:
        rand = 1
    #Wait for map data to be stored
    while width == 0:
        rand = 0
        #print width
    rospy.sleep(2)
    #print width
    #print mapData
    #find a path between the current and end position
    fullPath = aStarMapping(int(math.floor(xPos * 20)), int(math.floor(yPos * 20)), int(math.floor(xEnd * 20)), int(math.floor(yEnd * 20)))
    #print the path
    for i in range(0, len(fullPath)):
        print fullPath[len(fullPath) - 1 - i]
    #drive the robot along the path.
    for i in range(0, len(fullPath) - 1):
        driveBot(fullPath[len(fullPath) - 2 - i])
        print fullPath[len(fullPath) - 1 - i]
    
    print "At goal!"
    rospy.sleep(1)
