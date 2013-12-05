#!/usr/bin/env python

import rospy	#import rospy for ROS Node

from geometry_msgs.msg import Twist	
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from std_msgs.msg import Empty
from std_msgs.msg import Float64 

from tf.transformations import euler_from_quaternion
from math import degrees
from math import floor
from math import cos
from math import sin
from math import sqrt
from math import atan
from math import atan2
from math import tan
import math

from random import choice
from itertools import groupby
from operator import itemgetter

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
#from map_msgs.msg import OccpancyGridUpdate

from sensor_msgs.msg import LaserScan
#from sensor_msgs import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from kobuki_msgs.msg import BumperEvent

import sys	#import sys for arguments

global r, b

global pi

global current_yaw, px, py, robot_orientation

global callback_flag

global MAP_DATA, HEADER

global WIDTH, HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION

global start, goal, path

global PtArr_Empty, PtArr_Unknown, PtArr_Occupied

global COST_MAP

import numpy

def round_to_nearest_cell(num, divisor):    #finds the corner of the given cell
    #return num - (num%divisor)
    #print "fmod: ", numpy.fmod(num,divisor)
    return float(float(num) - numpy.fmod(num,divisor))

def A_star(start, goal, debug): #function generating path using A_Star algorithm 
    print "A_star"
    closedset= []   #Set of cells already evaluated
    openset = [start.position]   #Set of tentative cells to be evaluated, initially containing the starting node
    parents = {}    #empty map of navigated nodes
    g_score = {start.position:0} #cost from start along best known path
    #estimated total cost from start to goal
    f_score = {start.position: (g_score[start.position] + heuristic_cost_estimate(start.position,goal.position))}
    while openset:
        #create a f_score dictionary of openset values
        f_score_openset = {}
        for elem in openset:
            f_score_openset[elem] = f_score[elem]
        min_value = min(f_score_openset.itervalues())
        min_keys = [k for k in f_score_openset if f_score_openset[k] == min_value] #cells with lowest f_score[] value in openset
        #print "min_value", min_value
       # print "min_keys", min_keys
        #print "f_score", f_score
        current = choice(min_keys)  #randomly choose cell if two cells have same min value
        #delete all other possible same cost choices from openset
        for elem in min_keys:
            if not (numpy.allclose(elem.x, current.x) and numpy.allclose(elem.y, current.y)):
                openset.remove(elem) 
        print "expanding"
        if(debug):
            #Expand Cell Visulaization
            Expanded_GridCell(current)
        #if goal is reached, construct path
        if current == goal.position:
            return reconstruct_path(parents, goal.position)
        #else, remove current cell from openset
        openset.remove(current)
        #print "current.x", current.x
       # print "current.y", current.y
        #print "openset", len(openset)
        #add current cell to closed set
        closedset.append(current)
        #for each neighbor of current cell
        neighboring_nodes = neighbor_nodes(current, debug)
       # print neighboring_nodes
        for neighbor in neighboring_nodes:
            tentative_g_score = 0
            tentative_f_score = 0
            #print "neighbor.x: ",neighbor.x
            #print "neighbor.y: ", neighbor.y
            #Calculate tentative g_score
            tentative_g_score = g_score[current] + dist_between(current, neighbor)
            #Calculate tentative f_score
            tentative_f_score = tentative_g_score + heuristic_cost_estimate(neighbor, goal.position)
            #Calculate tentative_f_score with new_heuristic_cost_estimate
            #tentative_f_score = tentative_g_score + new_heuristic_cost_estimate(neighbor, goal.position)

            #Find f_score of neighbor
            for key in f_score:
                if numpy.allclose(key.x,neighbor.x) and numpy.allclose(key.y, neighbor.y):
                    current_neighbor_key = key
            #if neighbor in closed set, i.e., already evaluated and tentative_f_score >= f_score of neighbor       
            if neighbor in closedset and tentative_f_score >= f_score[current_neighbor_key]:
                continue
            #if neighbor not in openset or tentative_f_score <f_score of neighbor
            elif not neighbor in openset or tentative_f_score < f_score[current_neighbor_key]:
                parents_neighbor = {neighbor: current}
                parents.update(parents_neighbor)
                
                g_score_neighbor = {neighbor: tentative_g_score}
                g_score.update(g_score_neighbor)
                
                f_score_neighbor = {neighbor: tentative_f_score}
                f_score.update(f_score_neighbor)
                #print "tentative_f_score", tentative_f_score
                
                if not neighbor in openset:
                    #print "adding neighbor"
                    openset.append(neighbor)
        if debug:
            rospy.sleep(3)

def dist_between(point1, point2):   #finds the distance between any two points
    distance = sqrt( ((point2.y - point1.y)**2) + ((point2.x - point1.x)**2) )
    #print "distance between", distance
    return round(distance,4)

def reconstruct_path(parents, current):  #reconstructs the path the robot must traverse
    global path
    for key in parents:
        if numpy.allclose(key.x,current.x) and numpy.allclose(key.y,current.y): 
            path = reconstruct_path(parents, parents[key])
            return path + [current]
    return [current] 

def heuristic_cost_estimate(point, goal): #Straight line distance between the two points
    return round(sqrt( ((goal.y - point.y)**2) + ((goal.x - point.x)**2) ),4)

def new_heuristic_cost_estimate(point, goal): #Straight line distance between the two points accounting for local costmap 
    #linearizing original cost value to get new cost value
    Cost_Value = COST_MAP[(point.y-ORIGIN_Y-(RESOLUTION/2))/RESOLUTION*WIDTH]
    actual_distance = round(sqrt( ((goal.y - point.y)**2) + ((goal.x - point.x)**2) ),4)
    return actual_distance*(Cost_Value/100)

def neighbor_nodes(current, debug): #finds empty neighboring cells
    return check_occupancy(current, debug)

def find_all_neighbors(current):    #finds ALL neighbors of the given cell
    #print "finding neighbors"
    Neighbor_6 = Pose()
    Neighbor_6.position.x = round(current.x-RESOLUTION, 4)  
    Neighbor_6.position.y = round(current.y-RESOLUTION, 4) 
    Neighbor_6.orientation.w = pi + (pi/4)
    
    Neighbor_2 = Pose()
    Neighbor_2.position.x = round (current.x+RESOLUTION, 4)  
    Neighbor_2.position.y = round(current.y+RESOLUTION,4)  
    Neighbor_2.orientation.w = pi/4
    
    Neighbor_5 = Pose()
    Neighbor_5.position.x = round(current.x-RESOLUTION,4)
    Neighbor_5.position.y = round(current.y,4)
    Neighbor_5.orientation.w = pi
    
    Neighbor_1 = Pose()
    Neighbor_1.position.x = round(current.x+RESOLUTION,4) 
    Neighbor_1.position.y = round(current.y,4) 
    Neighbor_1.orientation.w = 0
    
    Neighbor_7 = Pose()
    Neighbor_7.position.x = round(current.x,4)
    Neighbor_7.position.y = round(current.y-RESOLUTION,4)
    Neighbor_7.orientation.w = 3*pi/2
    
    Neighbor_3 = Pose()
    Neighbor_3.position.x = round(current.x,4) 
    Neighbor_3.position.y = round(current.y +RESOLUTION,4)
    Neighbor_3.orientation.w = pi/2
    
    Neighbor_8 = Pose()
    Neighbor_8.position.x = round(current.x + RESOLUTION,4)
    Neighbor_8.position.y = round(current.y - RESOLUTION,4)
    Neighbor_8.orientation.w = (3*pi/2)+(pi/4)
    
    Neighbor_4 = Pose()
    Neighbor_4.position.x = round(current.x - RESOLUTION,4) 
    Neighbor_4.position.y = round(current.y + RESOLUTION,4)
    Neighbor_4.orientation.w = (pi/2) + (pi/4)
    
    neighbors_all = [Neighbor_1, Neighbor_2, Neighbor_3, Neighbor_4, Neighbor_5, Neighbor_6, Neighbor_7, Neighbor_8]
    return neighbors_all

def check_occupancy(current, debug):    #check whether the cell is occupied
    FRONTIER_PUBLISH = rospy.Publisher('/Frontier', GridCells)   #Frontier Grid Cells Publisher   
    
    neighbors_all = find_all_neighbors(current)
    #print neighbors_all
    neighbors_empty = []
    Frontier = []
    
    for neighbor_index, neighbor in enumerate(neighbors_all):
        #cell_index = ( neighbor.y - (RESOLUTION/2) ) * WIDTH
        #if MAP_DATA[int(cell_index)] < 50:   #EMPTY CELL
        for elem in PtArr_Empty:
            if numpy.allclose([elem.x], [neighbor.position.x]) and numpy.allclose([elem.y], [neighbor.position.y]):
            #if elem.x == neighbor.position.x and elem.y == neighbor.position.y:
            #print "index:", cell_index
            #print int(cell_index)
                Frontier.append(Point(neighbor.position.x, neighbor.position.y, 0))
                neighbors_empty.append(neighbor.position)
               # print "adding neighbor: ", neighbor_index
            #if elem.x == neighbor.position.x:
               # print "element_x: ", elem.x
                #print "element_y: ", elem.y
    #print PtArr_Empty
    Frontier_Empty  = GridCells()
    Frontier_Empty.header = HEADER
    Frontier_Empty.cell_width = RESOLUTION
    Frontier_Empty.cell_height = RESOLUTION
    Frontier_Empty.cells = Frontier
    if debug:
        count = 0
        while count < 1500:
            FRONTIER_PUBLISH.publish(Frontier_Empty)
            count = count+1
    return neighbors_empty

def Expanded_GridCell(cell):    #Expand Cell visually on rviz when debug is activated
    EXPANDED_PUBLISHER = rospy.Publisher('/Expanded', GridCells) #Expanded Grid Cells Publisher   
    cell_row = cell.x 
    cell_column = cell.y
    Expanded = []
    Expanded.append(Point(cell_row, cell_column, 0))
    Expanded_GridCell  = GridCells()
    Expanded_GridCell.header = HEADER
    Expanded_GridCell.cell_width = RESOLUTION
    Expanded_GridCell.cell_height = RESOLUTION
    Expanded_GridCell.cells = Expanded
    count = 0
    while count < 1500:
        EXPANDED_PUBLISHER.publish(Expanded_GridCell)
        count = count+1
    #Occupancy_Grid_init()
def Frontier_init():
    Frontier = rospy.Publisher('/Frontier', GridCells)
    #find maximum point in empty space in the x and y drection
    Array_EmptyCells = PtArr_Empty
    OCCUPIED = False
    MAX_BOOL = False
    MIN_BOOL = False
    while Array_EmptyCells:
        MAX_Point = max(Array_EmptyCells)
        MIN_Point = min(Array_EmptyCells)
        Frontier_Array = []
        while True:
            #check If the point after the max_point.x is occupied
            for elem in PtArr_Occupied:
                if (numpy.close(MAX_Point.x + RESOLUTION, elem.x) and numpy.close(MAX_Point.y, elem.y)):
                    OCCUPIED = True
                    MAX_BOOL = True
                    break
            for elem in PtArr_Occupied:
                if (numpy.close(MIN_Point.x - RESOLUTION, elem.x) and numpy.close(MIN_Point.y, elem.y)):
                    MIN_BOOL = True
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                if MAX_BOOL:
                    Frontier_Array.append(MAX_Point)    #add it to frontier array
                if MIN_BOOL:
                    Frontier_Array.append(MIN_Point)    #add it to frontier array
        Frontier_MaxPoint = Max_Point
        Frontier_MinPoint = Min_Point
        while True:
            Frontier_MaxPoint.y = Frontier_MaxPoint.y + RESOLUTION    #Next frontier point
            for elem in PtArr_Occupied:
                if numpy.close(Frontier_MaxPoint.y, elem.y ) and numpy.close(MAX_Point.x, elem.x) :
                    Frontier_Array.append(Frontier_MaxPoint)
                    break
            for elem in PtArr_Occupied:
                if numpy.close(Frontier_MinPoint.y, elem.y ) and numpy.close(Frontier_MinPoint.x, elem.x) :
                    Frontier_Array.append(Frontier_MinPoint)
                    break
            break
        #Remove row from frontier array
        for elem in Array_EmptyCells:
            if numpy.close(MAX_Point.y, elem.y):
                Array_EmptyCells.remove(elem)
        
    gridcells_frontier = GridCells()
    gridcells_frontier.header = HEADER
    gridcells_frontier.cell_width = RESOLUTION
    gridcells_frontier.cell_height = RESOLUTION
    gridcells_frontier.cells = Frontier_Array
    Frontier.publish(gridcells_frontier)
    return

def Occupancy_Grid_init():  #initialize visual aid for Occupancy Grid
    global PtArr_Empty, PtArr_Unknown, PtArr_Occupied
    GridCells_Occupied = rospy.Publisher('/GridCellsOccupied', GridCells)
    GridCells_Unknown =  rospy.Publisher('/GridCellsUnknown', GridCells)
    GridCells_Empty = rospy.Publisher('/GridCellsEmpty', GridCells)
    PtArr_Occupied = []
    PtArr_Unknown = []
    PtArr_Empty = []
    for i in range(len(MAP_DATA)):
        row = round(((i / WIDTH)*RESOLUTION) + (RESOLUTION/2), 4) + ORIGIN_Y
        column = round(((i % WIDTH)*RESOLUTION) + (RESOLUTION/2), 4) + ORIGIN_X
        if MAP_DATA[i] > 50: #Occupied
           PtArr_Occupied.append(Point(column, row, 0))
        elif MAP_DATA[i] == -1: #Unknown
           PtArr_Unknown.append(Point(column, row, 0))
        else:  #Empty
           PtArr_Empty.append(Point(column, row, 0))      
    gridcells_Occupied = GridCells()
    gridcells_Occupied.header = HEADER
    gridcells_Occupied.cell_width = RESOLUTION
    gridcells_Occupied.cell_height = RESOLUTION
    gridcells_Occupied.cells = PtArr_Occupied
    
    gridcells_Unknown = GridCells()
    gridcells_Unknown.header = HEADER
    gridcells_Unknown.cell_width = RESOLUTION
    gridcells_Unknown.cell_height = RESOLUTION
    gridcells_Unknown.cells = PtArr_Unknown
    
    gridcells_Empty  = GridCells()
    gridcells_Empty.header = HEADER
    gridcells_Empty.cell_width = RESOLUTION
    gridcells_Empty.cell_height = RESOLUTION
    gridcells_Empty.cells = PtArr_Empty
    
    GridCells_Occupied.publish(gridcells_Occupied)
    GridCells_Unknown.publish(gridcells_Unknown)
    GridCells_Empty.publish(gridcells_Empty)

def find_centroid():    #returns centroid of frontier
    Frontier_Centroid = rospy.Publisher('/FrontierCentroid', GridCells)
    x_c = y_c = count = 0
    for elem in PtArr_Empty:
        x_c = elem.x + x_c
        y_c = elem.y + y_c
        count = count + 1
    x_c = round_to_nearest_cell(x_c/count, RESOLUTION) + (RESOLUTION/2)
    y_c = round_to_nearest_cell(y_c/count, RESOLUTION) + (RESOLUTION/2)
    Centroid = Point(x_c, y_c, 0)
    gridcells_centroid  = GridCells()
    gridcells_centroid.header = HEADER
    gridcells_centroid.cell_width = RESOLUTION
    gridcells_centroid.cell_height = RESOLUTION
    gridcells_centroid.cells = [Centroid]
    Frontier_Centroid.publish(gridcells_centroid)
    Centroid_Pose = Pose()
    Centroid_Pose.position = Centroid
    Centroid_Pose.orientation.w = 0
    return Centroid_Pose

def Map_CallBack(data):#Callback for Occupancy Grid published on topic: /map
    global MAP_DATA, HEADER, path
    global WIDTH, HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION
    MetaMap = data.info
    WIDTH = MetaMap.width
    HEIGHT = MetaMap.height
    RESOLUTION = round(MetaMap.resolution, 4)
    ORIGIN_X = MetaMap.origin.position.x
    ORIGIN_Y = MetaMap.origin.position.y
    pz = MetaMap.origin.position.z
    quat = MetaMap.origin.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    #print "resolution: ", RESOLUTION
    #print "\n width: {0:+2.5f}".format(WIDTH) + "height: {0:+2.5f}".format(HEIGHT)
    #print "\n x: {0:+2.5f}".format(ORIGIN_X) + ", y: {0:+2.5f}".format(ORIGIN_Y)    + ", z: {0:+2.5f}".format(pz)  + ", w: {0:+.4f}".format(yaw)
    MAP_DATA = data.data
    HEADER = data.header
    if not start == None and not goal == None: 
        Occupancy_Grid_init()
        path = A_star(start, goal, 0)
        print "path: ", len(path)
        Visualize_Path(path)
        executeTrajectory(path)
    #print "data: ", len(data.data)
    #print data.data
   

def CostMap_Callback(data): #Callback for Costmap
    global COST_MAP
    COST_MAP = data.data

def MapMetaData_Callback(data): #Callback for MetaData of map
    global RESOLUTION, WIDTH, HEIGHT
    WIDTH = data.width
    HEIGHT = data.height
    px = data.origin.position.x
    py = data.origin.position.y
    pz = data.origin.position.z
    quat = data.origin.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    
    #print "\n width: {0:+2.5f}".format(WIDTH) + "height: {0:+2.5f}".format(HEIGHT)
    #print "\n x: {0:+2.5f}".format(px) + ", y: {0:+2.5f}".format(py)    + ", z: {0:+2.5f}".format(pz)  + ", w: {0:+.4f}".format(yaw)

def Visualize_Path(path):   #Function to publish path
    print "visualizing path"
    Path = rospy.Publisher('/Path', GridCells)
    Path_GridCells = GridCells()
    Path_GridCells.header = HEADER
    Path_GridCells.cell_width = RESOLUTION
    Path_GridCells.cell_height = RESOLUTION
    Path_GridCells.cells = path
    Path.publish(Path_GridCells)

def Start_Callback(data): #Callback for start position
    global start, goal
    
    start = Pose()
    start_x = data.pose.pose.position.x
    start.position.x = round_to_nearest_cell(start_x, RESOLUTION)
     #find center of start cell
    if  start.position.x > 0:
        start.position.x = start.position.x + (RESOLUTION/2) 
    else:
        start.position.x = start.position.x - (RESOLUTION/2)
    start.position.x = round(start.position.x, 4)
    
    start_y = data.pose.pose.position.y
    start.position.y = round_to_nearest_cell(start_y, RESOLUTION) 
     #find center of start cell
    if start.position.y > 0:
         start.position.y  =  start.position.y  + (RESOLUTION/2) 
    else:
         start.position.y =  start.position.y - (RESOLUTION/2) 
    start.position.y = round(start.position.y, 4)
     #find orientation of start cell
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, start_yaw = euler_from_quaternion(q)
    start.orientation.w = start_yaw
    
    print "\n start_x: ",start.position.x
    print "start_y: ",start.position.y
    print "\n start_yaw: {0:+.4f}".format(start_yaw)
    
    rotation_angle = angle_of_rotation(current_yaw, wrap_to_2pi(start.orientation.w))
    rotate(rotation_angle)

def Goal_Callback(data):    #Callback for goal position
    global start, goal, path
    goal = Pose()
    goal_x = data.pose.position.x
    goal.position.x = round_to_nearest_cell(goal_x, RESOLUTION)
    #find center of goal cell
    if goal.position.x > 0:
        goal.position.x = goal.position.x  + (RESOLUTION/2) 
    else:
        goal.position.x = goal.position.x - (RESOLUTION/2)
    goal.position.x = round(goal.position.x, 4)
    
    goal_y = data.pose.position.y
    goal.position.y = round_to_nearest_cell(goal_y,RESOLUTION) 
    #find center of goal cell
    if goal.position.y > 0:
        goal.position.y = goal.position.y + (RESOLUTION/2)
    else:
        goal.position.y = goal.position.y - (RESOLUTION/2)
    goal.position.y = round(goal.position.y, 4) 
    #find orientation of goal cell
    quat = data.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, goal_yaw = euler_from_quaternion(q)
    goal.orientation.w = goal_yaw
    
    print "\n goal_x: {0:+2.5f}".format(goal.position.x) + "goal_y: {0:+2.5f}".format(goal.position.y)
    print "\n goal_yaw: {0:+.4f}".format(goal_yaw) 
    

def wrap_to_pi(x):
    return numpy.mod(x+numpy.pi,2*numpy.pi)-numpy.pi

def wrap_to_2pi(x):
    twoPi = 2*pi
    return x - twoPi * floor( x / twoPi)

def sign(x):
    if x > 0: return +1
    if x < 0: return -1
    return 0


def publishTwist(u,w):	#takes the linear and angular velocity as inputs and publishes them to the robot.
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)	#publishing to cmd_vel_mux/input/teleop topic
									#using the message type Twist from geometry.msgs.msg.Twist
	twist = Twist()	#define own twist
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0;	#set the linear velocity
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w;	#set the angular velocity
	#rospy.loginfo(twist)
	pub.publish(twist)		#publish the twist messages

def spinWheels(phi1, phi2, time):	#takes as input the velocities of the left and right wheels, 
				#computes the translational and rotational robot velocities and 
				#drives the robot for the specified period of time.
	while True:	
		init_time = rospy.get_time()	#determine current time in seconds 
		if not init_time == 0:
			break;
	while True:
		#map wheel speeds to linear and angular velocities	
		u = r/2*(phi1+phi2)	#calculated from forward kinematics of the robot in prelab
		w = r/b*(phi1-phi2)	
		publishTwist(u,w)	#call publishTwist with the calculated arguments

		while True:
			final_time = rospy.get_time()	#determine current time in seconds 
			if not final_time == 0:
				break;

		time_elapsed = final_time - init_time
		if time_elapsed >= time:	#if time reached
			#rospy.loginfo("time_elapsed: %f", time_elapsed)
			break;	#break out of the loop

def Odom_Callback(data):
	
	global px, current_yaw, py, robot_orientation
	global callback_flag
	robot_orientation = Pose()
	px = robot_orientation.position.x = data.pose.pose.position.x
	py = robot_orientation.position.y = data.pose.pose.position.y 
	quat = robot_orientation.orientation = data.pose.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, current_yaw = euler_from_quaternion(q)
	callback_flag = True
	
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	yaw_rate = data.twist.twist.angular.z
	
	#print "{0:+2.5f}".format(px) + ", {0:+2.5f}".format(py)	+ ",{0:+.4f}".format(current_yaw) 
	#print "{0:+2.5f}".format(vx) + ", {0:+2.5f}".format(vy)	+ ",{0:+.2f}".format(yaw_rate) 

def driveStraight(speed, distance):
    global callback_flag
    rospy.Subscriber("odom", Odometry, Odom_Callback)
    initial_distance_x=px
    initial_distance_y=py
    initial_angle=current_yaw
    final_distance_x = initial_distance_x + (distance*cos(current_yaw))
    final_distance_y = initial_distance_y + (distance*sin(current_yaw))
    #print "init_x: ", px
    #print "init_y: ", py
    #print "final_x: ", final_distance_x
    #print "final_y: ", final_distance_y
    while True:
        publishTwist(speed, 0)    #call publishTwist with the calculated arguments
        #print "px - final_distance_x: ", px - final_distance_x
        #print "py - final_distance_y: ", py - final_distance_y
        if abs(px - final_distance_x) < 0.02 and abs(py - final_distance_y) < 0.02:
            publishTwist(0, 0)
            break

def rotate(angle):
    global callback_flag
    reset_odom()
    while True:
        if callback_flag == True:
            initial_angle = current_yaw
            callback_flag = False
            break
    if angle < 0:
        direction = 'right'
    elif angle > 0:
        direction = 'left'
    print "turning angle without wrapping", angle
    angle = wrap_to_pi(initial_angle + angle)#initial_angle
    print "turning angle: ", angle
    if direction == 'right':	#right
		while True:
			publishTwist(0.02, -0.25)	#call publishTwist with the calculated arguments
			if callback_flag == True: 
				callback_flag = False
				if abs(current_yaw-angle) <= 0.04:
					publishTwist(0, 0)
					break
    else:	#left
		while True:
			publishTwist(0.02, 0.25)	#call publishTwist with the calculated arguments
			if callback_flag == True: 
				callback_flag = False
				if abs(current_yaw-angle) <= 0.04:
					publishTwist(0, 0)
					break

def driveArc(radius, speed, angle):
	global callback_flag
	reset_odom()
	while True:
		if callback_flag == True:
			initial_angle = current_yaw
			callback_flag = False
			break
		
	if(angle < 0):
		direction = 'right'
	if angle > 0:
		direction = 'left'
		
	angle = wrap_to_pi(initial_angle + angle)
	
	if direction == 'right': #right
		while True:
			u = speed
			w = u/radius	
			publishTwist(u, -w)	#call publishTwist with the calculated arguments
			if abs(current_yaw-angle) <= 0.04:
				publishTwist(0, 0)
				break
	else: #left
		while True:
			u = speed
			w = u/radius
			publishTwist(u, w)	#call publishTwist with the calculated arguments
			if abs(current_yaw-angle) <= 0.04:
				publishTwist(0, 0)
				break 

'''def BumperCallback(data):
	if (data.state == BumperEvent.RELEASED):
		state = "released"
	else: 
		state = "pressed"
		#executeTrajectory()
	#rospy.loginfo("bumper is %s."%(state))'''

def executeTrajectory(path):
    for index, elem in enumerate(path):
        if index+1 < len(path):
            all_neighbors = find_all_neighbors(elem)
            immediate_start = Pose()
            immediate_start.position.x = elem.x
            immediate_start.position.y = elem.y
            immediate_start.orientation.w = current_yaw
                            
            immediate_goal = Pose()
            immediate_goal.position.x = path[index+1].x
            immediate_goal.position.y = path[index+1].y
            for neighbor_index, neighbor in enumerate(all_neighbors):
                if numpy.allclose(neighbor.position.x,immediate_goal.position.x) and numpy.allclose(neighbor.position.y,immediate_goal.position.y):
                    immediate_goal.orientation.w = neighbor.orientation.w
                    #print "goal_orientation_set: ", immediate_goal.orientation.w
                   # print "neighbor #: ", neighbor_index
                    break
               # else:
                 #   print "couldn't find neighbor"
         #  print "TRAJECTORY immediate_start: ", immediate_start.position.x, immediate_start.position.y
            #print "TRAJECTORY immediate_goal: ", immediate_goal.position.x, immediate_goal.position.y
            rotation_angle = angle_of_rotation(immediate_start.orientation.w, immediate_goal.orientation.w)
            if abs(rotation_angle)>0.1:
                print "rotating: ", rotation_angle
                rotate(rotation_angle)
            distance = dist_between(immediate_start.position, immediate_goal.position)
            print "driving straight: ", distance
            driveStraight(0.1, distance)
    print "stopped"
    #rotation_angle = angle_of_rotation(current_yaw, goal.orientation.w)
    #rotate(rotation_angle)

def angle_of_rotation(immediate_start, immediate_goal):
    if immediate_start >= -0.2 and immediate_start  <=3.14:
        print "positive", immediate_start
        initial_angle = abs(immediate_start )
        print "init_angle: ", initial_angle
    else: 
        print "negative", immediate_start
        initial_angle = wrap_to_2pi(immediate_start)
        print "init_angle: ", initial_angle
    final_angle = immediate_goal
    print "final_angle: ", final_angle
    angle_of_rotation = final_angle - initial_angle
    if abs(angle_of_rotation) < pi:
        print "less than 2pi"
        if (angle_of_rotation > -0.1):  #turn left
            print "turnng left",angle_of_rotation
            return angle_of_rotation
        else:   #turn left
            print "turning right", (angle_of_rotation)
            return angle_of_rotation
    elif final_angle > initial_angle:
        print "final>initial"
        if (angle_of_rotation > -0.1):  #turn left
           print "turnng left",angle_of_rotation
           return angle_of_rotation - (2*pi)
        else:   #turn left
           print "turning right", (angle_of_rotation)
           return angle_of_rotation - (2*pi)
    else:
        print "final<initial"
        if (angle_of_rotation > -0.1):  #turn left
           print "turnng left",angle_of_rotation
           return angle_of_rotation + (2*pi)
        else:   #turn left
           print "turning right", (angle_of_rotation)
           return angle_of_rotation + (2*pi)

def reset_odom():
    global current_yaw, px, py
    global callback_flag
    callback_flag = False
    current_yaw = 0
    px = 0
    py = 0

def init_globals():
    global start, goal, MAP_DATA
    start = goal = None
    MAP_DATA = None
def init_consts():
	global r, b
	global pi
	pi = 3.14
	r = 0.038
	b = 0.228

if __name__ == '__main__':
    try:
        global path
        rospy.init_node('lab3')    #initialize the node
        init_globals()
        init_consts()
        rospy.Subscriber("map", OccupancyGrid, Map_CallBack)
        rospy.Subscriber("map_metadata", MapMetaData, MapMetaData_Callback)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, CostMap_Callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, Start_Callback)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, Goal_Callback)
        rospy.Subscriber("odom", Odometry, Odom_Callback)
        
        
        rospy.spin()
	#init_consts()
	#reset_odom()
	#rospy.init_node('lab2')	#initialize the node
	#rospy.myargv(argv=sys.argv)	#take arguments
	#rospy.loginfo("spinning wheels")
	#executeTrajectory()
	#rospy.Subscriber("mobile_base/events/bumper", BumperEvent, BumperCallback)
	#rospy.spin()
	#if len(sys.argv) < 4:	#print error for invalid number of arguments
	#	rospy.loginfo("Invalid number of arguments")
	#else:
	
	#spinWheels(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
	#driveStraight(float(sys.argv[1]), float(sys.argv[2]))
	#rotate(-1.57)
	#rotate (-3.14)
	#rotate(float(sys.argv[1]))
	#driveArc(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    except rospy.ROSInterruptException:
        pass

