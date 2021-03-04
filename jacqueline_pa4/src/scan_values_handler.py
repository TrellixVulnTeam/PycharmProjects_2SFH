#!/usr/bin/env python

# This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
# from constants import *
from state_definitions import *
from jacqueline_pa4.msg import Filtered_Laserscan

# Process all the data from the LIDAR
def cb(msg):
    # define the state by processed ranges info
    global ranges
    ranges = np.array(msg.ranges)
    for i in range(len(ranges)):
        if ranges[i] <= 0 or ranges[i] > 4:
            ranges[i] = smooth_value(ranges, i)
    s = state_define(ranges)
    # publish state
    pub_state.publish(s)
    # calculate msg to publish to pid
    msg = calc_cross_err(ranges)
    # publish msg
    pub_svh.publish(msg)


# smooth unusable laserscan data by average of 10 data around it
def smooth_value(ranges, i):
    sum = 0
    count = 0
    for j in range(i-5, i+5):
        if ranges[j] < 0.1 or ranges[j] > 4:
            sum += ranges[j]
            count += 1
    return sum / count

# calculate dist from wall and combine info into a message
def calc_cross_err(ranges):
    msg = Filtered_Laserscan()
    msg.filtered_ranges = ranges
    msg.LEFT = LEFT
    msg.RIGHT = RIGHT
    if msg.LEFT:
        msg.dist_from_wall_L = min(ranges[0:180])
    else:
        msg.dist_from_wall_L = 4
    if msg.RIGHT:
        msg.dist_from_wall_R = min(ranges[180:360])
    else:
        msg.dist_from_wall_R = 4
    return msg

# define the state by laserscan data
def state_define(ranges):
    global LEFT
    global RIGHT
    # following state
    if mean(ranges[80:100]) <= 1 + err or mean(ranges[260:280]) <= 1 + err:
        LEFT = True
        RIGHT = True
        return FOLLOWING
    elif mean(ranges[80:100]) <= 1 + err:
        LEFT = True
        RIGHT = False
        return FOLLOWING
    elif mean(ranges[260:280]) <= 1 + err:
        LEFT = False
        RIGHT = True
        return FOLLOWING
    # found a wall state
    elif min(ranges[0:180]) < 4 and min(ranges[0:180]) > 1 + err and min(ranges[180:360]) >= min(ranges[0:180]):
        LEFT = True
        RIGHT = False
        return FOUND_WALL
    elif min(ranges[180:360]) < 4 and min(ranges[180:360]) > 1 + err and min(ranges[0:180]) >= min(ranges[180:360]):
        LEFT = False
        RIGHT = True
        return FOUND_WALL
    # wandering state
    elif min(ranges) > 4:
        LEFT = False
        RIGHT = False
        return WANDERING
    else:
        return ""


# Init node
rospy.init_node('scan_values_handler')

# Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

# Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
#THINK OF WHAT INFO TO PUBLISH TO THE PID
pub_svh = rospy.Publisher('cross_err', Filtered_Laserscan, queue_size = 1)

# Rate object
rate = rospy.Rate(10)

ranges = []

# initialize error objects
err = 0.5
cross_error = 0
# if the robot is going to follow the wall on left side
LEFT = False
RIGHT = False

# buffer loop
while ranges == []: continue

# Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 