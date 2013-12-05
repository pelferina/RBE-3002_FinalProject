#!/usr/bin/env python

import rospy	#import rospy for ROS Node

from geometry_msgs.msg import Twist	#import Twist command from geometry_msgs
from std_msgs.msg import Empty 

from tf.transformations import euler_from_quaternion
from math import degrees
from math import cos
from math import sin

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import CliffEvent

import sys	#import sys for arguments

global r, b

global pi

global px, yaw, py

global callback_flag

import numpy

def wrap_to_pi(x):
    return numpy.mod(x+numpy.pi,2*numpy.pi)-numpy.pi

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
	
	global px, yaw, py
	global callback_flag
	
	px = data.pose.pose.position.x
	py = data.pose.pose.position.y
	quat = data.pose.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)
	
	callback_flag = True
	
	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	yaw_rate = data.twist.twist.angular.z
	
	print "{0:+2.5f}".format(px) + ", {0:+2.5f}".format(py)	+ ",{0:+.4f}".format(yaw) 
	#print "{0:+2.5f}".format(vx) + ", {0:+2.5f}".format(vy)	+ ",{0:+.2f}".format(yaw_rate) 

'''def driveStraight(speed, distance):
	global callback_flag
	rospy.Subscriber("odom", Odometry, Odom_Callback)
	while True:
		if callback_flag == True:
			initial_distance = px
			callback_flag = False
			break
	while True:
		publishTwist(speed, 0)	#call publishTwist with the calculated arguments
		if callback_flag == True: 
			callback_flag = False
			if yaw > -0.1:
				if px >= distance + initial_distance:
					publishTwist(0, 0)
					break
			else:
				if px <= initial_distance - distance:
					publishTwist(0, 0)
					break'''

def driveStraight(speed, distance):
    global callback_flag
    rospy.Subscriber("odom", Odometry, Odom_Callback)
    initial_distance_x=px
    initial_distance_y=py
    initial_angle=yaw
    final_distance_x = initial_distance_x + (distance*cos(yaw))
    final_distance_y = initial_distance_y + (distance*sin(yaw))
    print "init_x: ", px
    print "init_y: ", py
    print "final_x: ", final_distance_x
    print "final_y: ", final_distance_y
    while True:
        publishTwist(speed, 0)    #call publishTwist with the calculated arguments
        print "px - final_distance_x: ", px - final_distance_x
        print "py - final_distance_y: ", py - final_distance_y
        if abs(px - final_distance_x) < 0.12 and abs(py - final_distance_y) < 0.12:
            publishTwist(0, 0)
            break

def rotate(angle):
	global callback_flag
	reset_odom()
	rospy.Subscriber("odom", Odometry, Odom_Callback)
	while True:
		if callback_flag == True:
			initial_angle = yaw
			callback_flag = False
			break
		
	if(angle < 0):
		direction = 'right'
	if angle > 0:
		direction = 'left'
	
	angle = wrap_to_pi(initial_angle + angle)
		
	if direction == 'right':	#right
		while True:
			publishTwist(0.02, -0.25)	#call publishTwist with the calculated arguments
			if callback_flag == True: 
				callback_flag = False
				if abs(yaw-angle) <= 0.04:
					publishTwist(0, 0)
					break
	
	else:	#left
		while True:
			publishTwist(0.02, 0.25)	#call publishTwist with the calculated arguments
			if callback_flag == True: 
				callback_flag = False
				if abs(yaw-angle) <= 0.04:
					publishTwist(0, 0)
					break
			

def driveArc(radius, speed, angle):
	global callback_flag
	reset_odom()
	rospy.Subscriber("odom", Odometry, Odom_Callback)
	while True:
		if callback_flag == True:
			initial_angle = yaw
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
			if abs(yaw-angle) <= 0.04:
				publishTwist(0, 0)
				break
	else: #left
		while True:
			u = speed
			w = u/radius
			publishTwist(u, w)	#call publishTwist with the calculated arguments
			if abs(yaw-angle) <= 0.04:
				publishTwist(0, 0)
				break 
		
def BumperCallback(data):
	if (data.state == BumperEvent.RELEASED):
		state = "released"
	else: 
		state = "pressed"
		executeTrajectory()
	#rospy.loginfo("bumper is %s."%(state))
		
def executeTrajectory():
	driveStraight(0.2, 0.60)	
	rotate(-1.57)	#rotate right 90 
	driveArc(0.15, 0.08, 3.14)
	rotate(2.356)	#rotate left 135
	driveStraight(0.2, 0.42)	
	
def reset_odom():
    global yaw, px, py
    global callback_flag
    callback_flag = False
    yaw = 0
    px = 0
    py = 0

def init_consts():
	global r, b
	global pi
	pi = 3.14
	r = 0.038
	b = 0.228
	
if __name__ == '__main__':
    try:
    	init_consts()
    	reset_odom()
    	rospy.init_node('lab2')	#initialize the node
        #driveStraight(0.2, 0.60)    
        rotate(0.78)    #rotate right 90 
        #driveArc(0.15, 0.08, 3.14)
        #rotate(2.356)    #rotate left 135
        driveStraight(0.2, 1.4)
        #rotate(-3.14)
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

