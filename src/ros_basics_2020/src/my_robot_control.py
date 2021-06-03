#!/usr/bin/env python

# ------------- Imports ---------------------

import rospy
import numpy as np
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

# ------------- Callback Functions ----------

# Callback function to get pose
def callback_odom(data):
    # Global Pose Variables
    global POS_X
    global POS_Y
    global YAW 

    # Position
    POS_X = data.pose.pose.position.x
    POS_Y = data.pose.pose.position.y

    # Orientation
    q = (data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    e = tf.transformations.euler_from_quaternion(q)
    YAW = e[2]

# Callback function to get goal position
def callback_goal(data):
    # Global Waypoint list
    global WAYPOINTS
    WAYPOINTS.append(data)

# Callback functions to get sensor data
def callback_front_sensor(data):
    # Global Front Sensor array
    global FRONT
    FRONT = data.ranges

def callback_left_sensor(data):
    # Global Left Sensor array
    global LEFT
    LEFT = data.ranges

def callback_right_sensor(data):
    # Global Right Sensor array
    global RIGHT
    RIGHT = data.ranges

# ------------- Controller Functions ----------

def compute_errors():
    # Error computation for PD controller
    error_x = WAYPOINTS[0].x-POS_X
    error_y = WAYPOINTS[0].y-POS_Y
    yawl = YAW
    if (abs(yawl)) > 3:
        yawl = abs(yawl)
    error_angle = np.arctan2(error_y, error_x)-yawl
    if (abs(error_angle)) > 3:
        error_angle = 3.14 - yawl + error_angle%3.14
    return error_x, error_y, error_angle

def angle_controller(err_angle, old_err_angle, vel, pub):
    # Angle PD controller to orient robot correctly
    Kp = 1.2
    Kd = 0.2
    d_err_angle = err_angle - old_err_angle
    vel.angular.z = err_angle*Kp + d_err_angle*Kd
    pub.publish(vel)

def pos_controller(err_x, err_y, old_err_x, old_err_y, vel, pub):
    # Position PD controller to move robot
    Kp = 0.3
    Kd = 0.08
    d_err_x = err_x - old_err_x
    d_err_y = err_y - old_err_y
    speed = Kp*np.linalg.norm([err_x, err_y]) + Kd*np.linalg.norm([d_err_x, d_err_y])
    if (speed > 0.5):
        speed = 0.5
    vel.linear.x = speed
    pub.publish(vel)

def stop(vel, pub):
    # Stops robot
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)

# ------------- Subscribe and Publish Functions ---------

# Subscribe to Topics (odom, goal_pos, sensor data)
def subscribers():
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback_odom)
    rospy.Subscriber("goal_pos", Point, callback_goal)
    rospy.Subscriber("laser_front/scan", LaserScan, callback_front_sensor)
    rospy.Subscriber("laser_left/scan", LaserScan, callback_left_sensor)
    rospy.Subscriber("laser_right/scan", LaserScan, callback_right_sensor)

# Publish to Odometry Topic
def publishers():
    return rospy.Publisher("cmd_vel", Twist, queue_size=10)

# ------------- Waypoint Navigation Functions ----------

def navigation(err_x, err_y, err_angle, switch, vel, pub):
    global STATE
	
	# navigation state with waypoints
    if (len(WAYPOINTS) > 0) & (STATE == 0):  
		# Prints  
        if (switch == 1):
            rospy.loginfo("Waypoint X: %f", WAYPOINTS[0].x)
            rospy.loginfo("Waypoint Y: %f\n", WAYPOINTS[0].y)
            switch = 0

		# Errors for controllers and conditions
        old_err_x = err_x
        old_err_y = err_y
        old_err_angle = err_angle
        err_x, err_y, err_angle = compute_errors()

        if (abs(np.linalg.norm([err_x,err_y])) > abs(np.linalg.norm([old_err_x,old_err_y]))):
            stop(vel, pub)

        if (abs(err_angle)>0.05) & (abs(np.linalg.norm([err_x,err_y])) > 0.05):
			# Angle Controller
            angle_controller(err_angle, old_err_angle, vel, pub)
        elif (abs(np.linalg.norm([err_x,err_y])) > 0.05):
			# Position Controller
            pos_controller(err_x, err_y, old_err_x, old_err_y, vel, pub)
        else:
            stop(vel, pub)
            rospy.loginfo("Waypoint Reached\n")
            WAYPOINTS.pop(0)
            switch = 1

	# Check for obstacles
        obstacle_check(vel, pub, switch)
    else:
        if (switch == 0):
            rospy.loginfo("Waiting for Waypoint\n")
            switch = 1
    
    return err_x, err_y, err_angle, switch

# ------------- Obstacle Avoidance Functions -----------

def obstacle_check(vel, pub, switch):
    global STATE
    
    # Check front sensors and choose contour
    if (FRONT[7] < 0.2) | (FRONT[6] < 0.2):
        stop(vel, pub)
        switch = 0
        rospy.loginfo("Obstacle Detected\n")
        if(FRONT[6] > FRONT[7]):
            STATE=2
            rospy.loginfo("Right Contour Initiated\n")
        else:
            STATE=1
            rospy.loginfo("Left Contour Initiated\n")

def left_contour(vel, pub, switch):
    global STATE
    
    # Ideal wall distance and error
    dist = 0.13
    error = (RIGHT[7]+RIGHT[6])/2 - dist
    
    # Approach wall and turn right
    if (switch == 0):
        if (FRONT[7] > 0.13) | (FRONT[6] > 0.13):
            vel.linear.x = 0.1
            vel.angular.z = 0
            pub.publish(vel) 
        else:
            vel.linear.x = 0
            vel.angular.z = 1
            pub.publish(vel)
            time.sleep(1.5)
            switch = 1
    else: # Wall following
        if (((error < -0.05) & (error > -0.08)) |
            (FRONT[7] < 0.12) | (FRONT[6] < 0.12) |
            ((FRONT[6] < 0.12) & (RIGHT[7] < 0.12))):
            vel.linear.x = 0
            vel.angular.z = 1
            pub.publish(vel)
        elif (((error > 0.05) & (error < 0.08)) | 
            ((RIGHT[3] > 0.2) & (RIGHT[7] > 0.2))):
            vel.linear.x = 0
            vel.angular.z = -1
            pub.publish(vel)
        else:
            vel.linear.x = 0.1
            vel.angular.z = 0
            pub.publish(vel)
	
		# Check if obstacle is avoided
        _, _, err_angle = compute_errors()
        if (err_angle > 0):
            STATE = 0
            rospy.loginfo("Obstacle Avoided\n")

    return switch

def right_contour(vel, pub, switch):
    global STATE
    
    # Ideal wall distance and error
    dist = 0.13
    error = (LEFT[7]+LEFT[6])/2 - dist
    
    # Approach wall and turn left
    if (switch == 0):
        if (FRONT[7] > 0.13) | (FRONT[6] > 0.13):
            vel.linear.x = 0.1
            vel.angular.z = 0
            pub.publish(vel) 
        else:
            vel.linear.x = 0
            vel.angular.z = -1
            pub.publish(vel)
            time.sleep(1.5)
            switch = 1
    else: # Wall following
        if (((error < -0.05) & (error > -0.08)) |
            (FRONT[7] < 0.12) | (FRONT[6] < 0.12) |
            ((FRONT[6] < 0.12) & (LEFT[6] < 0.12))):
            vel.linear.x = 0
            vel.angular.z = -1
            pub.publish(vel)
        elif (((error > 0.05) & (error < 0.08)) | 
            ((LEFT[10] > 0.2) & (LEFT[6] > 0.2))):
            vel.linear.x = 0
            vel.angular.z = 1
            pub.publish(vel)
        else:
            vel.linear.x = 0.1
            vel.angular.z = 0
            pub.publish(vel)

		# Check if obstacle is avoided
        _, _, err_angle = compute_errors()
        if (err_angle < 0):
            STATE = 0
            rospy.loginfo("Obstacle Avoided\n")
            
    return switch

# ------------- Algorithm -------------------

# Initializations
global STATE
WAYPOINTS = []
FRONT     = [0]*14
RIGHT     = []
LEFT      = []
POS_X     = 0
POS_Y     = 0
YAW       = 0
STATE     = 0
err_x     = 10
err_y     = 10
err_angle = 10
switch    = 0

# ROS functions
subscribers()
pub = publishers()
vel = Twist()
rate = rospy.Rate(10)

# Infinite loop
while not rospy.is_shutdown():
    if (STATE == 0):
        err_x, err_y, err_angle, switch = navigation(err_x, err_y, err_angle, switch, vel, pub)
    elif (STATE == 1):
        switch = left_contour(vel, pub, switch)
    elif (STATE == 2):
        switch = right_contour(vel, pub, switch)
    else:
        STATE = 0
    rate.sleep()
