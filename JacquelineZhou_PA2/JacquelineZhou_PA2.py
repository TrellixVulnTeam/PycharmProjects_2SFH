#!/usr/bin/env python

# Based on Starter Code for New PA2
# Starter Code based on code from August Soderberg
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pi

ranges = None
pose = None


def scan_cb(msg):
    global ranges
    ranges = np.array(msg.ranges)


def odom_cb(msg):
    global pose
    pose = msg.pose


# stage 1: the robot goes straight until near a wall
# and then make a "random" turn, start again
def random_vaccum(pub, rate):
    global ranges
    while ranges[360] >= 0.2:
        twist = set_forward(0.3)
        print(">>>>>>>>>>> roomba vaccuming <<<<<<<<<<<")
        pub.publish(twist)
        rate.sleep()
    pub.publish(Twist())
    target_angle = (90 + 45 * np.random.normal(0, 1, 1)) / 180 * pi
    if (ranges[340] <= ranges[380]):
        angular_speed = 0.5
    else:
        angular_speed = -0.5
    ticks = int(target_angle / 0.5 * 50)
    twist = set_rotate()
    print(">>>>>>>>>>> roomba rotating <<<<<<<<<<<")
    for i in range(ticks):
        pub.publish(twist)
        rate.sleep()

    pub.publish(Twist())
    rate.sleep()


# def back_origin():


def roomba_start():
    rospy.init_node('pilot', anonymous=False)
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    r = rospy.Rate(50)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        if (rospy.Time.now() - start_time) < rospy.Duration(secs=60):
            # print(pose.pose.position.x)
            random_vaccum(pub, r)
        else:
            print(">>>>>>>>>>> roomba returning <<<<<<<<<<<")
            break
    r.sleep()


# return a message with a forward speed
def set_forward(speed):
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    return twist


# return a message with an angular speed
def set_rotate(degree):
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = degree
    return twist


# main
if __name__ == '__main__':
    try:
        roomba_start()
    except rospy.ROSInterruptException:
        pass