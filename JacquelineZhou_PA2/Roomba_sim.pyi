#!/usr/bin/env python

# Based on Starter Code for New PA2
# Starter Code based on code from August Soderberg
# I also looked at examples in textbook
import rospy
import numpy as np
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi, sqrt, atan2

def scan_cb(msg):
    global ranges
    ranges = np.array(msg.ranges)

# function to wander around avoiding obstacles
def avoid_obs_move(ranges):
    if min(ranges[0:30]) >= threshold_1 and min(ranges[330:360]) >= threshold_1:
        twist.linear.x = forward_speed
        twist.angular.z = 0
    else:
        twist.linear.x = 0
        if min(ranges[0:50]) < min(ranges[310:360]):
            # if further obstacle on the right
            twist.angular.z = -abs_rotate
        else:
            # if further obstacle on the left
            twist.angular.z = abs_rotate
    pub.publish(twist)

def odom_cb(msg):
    global pose
    pose = msg.pose
    global x
    x = pose.pose.position.x
    global y
    y = pose.pose.position.y
    global yaw
    orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

# function to calculate which angle the robot should be at
def calc_turn_to():
    if x0 > x and y0 < y:
        turn_to = 3*pi/2 + atan2(abs(x0 - x), abs(y0 - y))
    elif x0 > x and y0 > y:
        turn_to = pi/2 - atan2(abs(x0 - x), abs(y0 - y))
    elif x0 < x and y0 > y:
        turn_to = pi/2 + atan2(abs(x0 - x), abs(y0 - y))
    elif x0 < x and y0 < y:
        turn_to = 3*pi/2 - atan2(abs(x0 - x), abs(y0 - y))
    elif x0 == x and y0 > y:
        turn_to = pi/2
    elif x0 == x and y0 < y:
        turn_to = 3*pi/2
    elif y0 == y and x0 < x:
        turn_to = pi
    elif y0 == y and x0 > x:
        turn_to = 0
    return turn_to

# function for the robot to go back to original point
def going_back(yaw):
    # calculate how far it is from the original point
    distance = sqrt(abs(x-x0)*abs(x-x0) + abs(y-y0)*abs(y-y0))
    turn_to = calc_turn_to()
    # making turn
    if yaw < 0 :
        yaw = yaw + 2*pi
    print(yaw)
    target = abs(yaw - turn_to)
    if turn_to > yaw:
        rotate_speed = abs_rotate
    else:
        rotate_speed = -abs_rotate
    print(target)
    # going straight forward
    twist.angular.z = rotate_speed
    twist.linear.x = 0
    tics = int(target / abs_rotate * r)
    for i in range(tics):
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())
    tics = int(distance / forward_speed * r)
    for i in range(tics):
        avoid_obs_move(ranges)
        rate.sleep()
    pub.publish(Twist())
    if distance <= 0.25:
        global back
        back = True

# initialized node
rospy.init_node('roomba_sim')
# set Hz and rate
r = 5
rate = rospy.Rate(r)
# set absolute speed for robot
forward_speed = 0.2
abs_rotate = 0.5
act_speed = 0.5
# set obstacle threshold distance
threshold_1 = 0.25

ranges = []
pose = None

pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
twist = Twist()

start_time = rospy.Time.now()

back = False

# initialize pose
while pose == None: continue
start_pose = pose
yaw = 0
x = pose.pose.position.x
y = pose.pose.position.y
x0 = start_pose.pose.position.x
y0 = start_pose.pose.position.y

# wait for ranges initialization
while ranges == []: continue

# action spin
while not rospy.is_shutdown():
    if rospy.Time.now() - start_time < rospy.Duration(secs = 30):
        avoid_obs_move(ranges)
    else:
        break
    rate.sleep()

pub.publish(Twist())

while not back:
    going_back(yaw)
