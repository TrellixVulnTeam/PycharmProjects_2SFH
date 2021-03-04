#!/usr/bin/env python

# This is a PID controller to determine twist values

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from math import pi
from jacqueline_pa4.msg import Filtered_Laserscan

# scan_values_handler subscriber callback
def sub_svh_cb(msg):
    global filtered_laserscan
    filtered_laserscan = msg
    calc_pid()

# calculate errors and pid components
def calc_pid():
    global err_prev_L
    global err_prev_R
    global err_curr_L
    global err_curr_R
    global err_sum_L
    global err_sum_R
    global pid_sum_L
    global pid_sum_R
    if filtered_laserscan.LEFT:
        err_L = EXP_DIST - filtered_laserscan.dist_from_wall_L
        err_prev_L = err_curr_L
        err_curr_L = err_L
        err_sum_L += err_curr_L * dT
        pid_sum_L = get_pid(err_curr_L, err_prev_L, err_sum_L)
    if filtered_laserscan.RIGHT:
        err_R = EXP_DIST - filtered_laserscan.dist_from_wall_R
        err_prev_R = err_curr_R
        err_curr_R = err_R
        err_sum_L += err_curr_L * dT
        pid_sum_R = get_pid(err_curr_R, err_prev_R, err_sum_R)

def get_pid(curr, prev, sum):
    p_component = curr
    d_component = (curr - prev)/dT
    i_component = sum 
    return P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component

#CALLBACKS FOR ANYTHING YOUR PID NEEDS TO SUBSCRIBE TO FROM scan_values_handler

# Init node
rospy.init_node('pid')

# Create publisher for suggested twist objects
pub = rospy.Publisher('pid_twist', Twist, queue_size = 1)

#SUBSCRIBERS FOR THINGS FROM scan_values_handler YOU MIGHT WANT
sub_svh = rospy.Subscriber('cross_err', Filtered_Laserscan, sub_svh_cb)

# initialize Filtered_Laserscan object
filtered_laserscan = Filtered_Laserscan()

# Twist and rate object
t = Twist()
r = 10
rate = rospy.Rate(r)

# allowed error
err = 0.5

# initialize variables for pid calculation
EXP_DIST = 1.0
err_sum_L = 0.0
err_sum_R = 0.0
err_prev_L = 0.0
err_prev_R = 0.0
err_curr_L = 0.0
err_curr_R = 0.0
dT = 1 / r

# Linear speed of the robot
LINEAR_SPEED = 0.3
# Angular speed of the robot
ANGULAR_SPEED = pi/6

# Multipliers used to tune the PID controller
# Proportional constant
P_CONSTANT = 0.5
# Integral constant
I_CONSTANT = 0.3
# Derivative constant
D_CONSTANT = 0.1
# pid
pid_sum_L = 0.0
pid_sum_R = 0.0

# buffer loop
while filtered_laserscan == Filtered_Laserscan(): continue

while not rospy.is_shutdown():
    #angular velocity
    if filtered_laserscan.LEFT and not filtered_laserscan.RIGHT:
        # found/following wall on the right
        t.angular.z = ANGULAR_SPEED * pid_sum_L
        t.linear.x = LINEAR_SPEED
    elif filtered_laserscan.RIGHT and not filtered_laserscan.LEFT:
        # found/following wall on the left
        t.angular.z = ANGULAR_SPEED * pid_sum_R
        t.linear.x = LINEAR_SPEED
    elif filtered_laserscan.LEFT and filtered_laserscan.RIGHT:
        #fonud two walls
        if filtered_laserscan.dist_from_wall_L > 1 + err and filtered_laserscan.dist_from_wall_R >= filtered_laserscan.dist_from_wall_L:
            # there is more space on the right, follow the right wall
            t.angular.z = ANGULAR_SPEED * pid_sum_R
            t.linear.x = LINEAR_SPEED
        elif filtered_laserscan.dist_from_wall_R > 1 + err and filtered_laserscan.dist_from_wall_L >= filtered_laserscan.dist_from_wall_R:
            # there is more space on the left, follow the left wall
            t.angular.z = ANGULAR_SPEED * pid_sum_L
            t.linear.x = LINEAR_SPEED
        else:
            # no space for two walls, stop the robot
            t.angular.z = 0
            t.linear.x = 0
    else:
        # no wall found (driver handles this situation)
            t.angular.z = ANGULAR_SPEED
            t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 