#!/usr/bin/env python

# Based on Starter Code for New PA2
# Starter Code based on code from August Soderberg
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def scan_cb(msg):
    global ranges
    ranges = np.array(msg.ranges)

def odom_cb(msg):
    global pose
    pose = msg.pose

def initial_roomba():
	rospy.init_node('pilot', anonymous = False)
	scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
	odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

	pose = None
	rate = rospy.Rate(5)

def roomba_start():
	initial_roomba()

	start_time = rospy.Time.now()
	while pose == None:
	    print("Waiting for simulated robot")

	start_pose = pose

	while not rospy.is_shutdown():
	    if rospy.Time.now() - start_time < rospy.Duration(secs=30):
	        print(pose.pose.position.x)
	        print(">>>>>>>>>>> roomba is vaccuming")
	    else:
	        print("<<<<<<<<<< roomba returns to start")
	        break
	    rate.sleep()

#main
if __name__ == '__main__':
    try:
        roomba_start()
    except rospy.ROSInterruptException:
        pass