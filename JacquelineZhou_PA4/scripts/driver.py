#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.

import rospy
import random
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

# wandering cmd_vel generator
def wander():
    twist = Twist()
    twist.linear.x = LINEAR_SPEED
    twist.angular.z = random.randrange(4) * rand_one() * ANGULAR_SPEED
    return twist

# randomly generate 1 or -1
def rand_one():
    return 1 if random.random() < 0.5 else -1

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
r = 10
rate = rospy.Rate(r)

# Linear speed of the robot
LINEAR_SPEED = 0.3
# Angular speed of the robot
ANGULAR_SPEED = pi/6

# starting state
state = 0

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

# buffer loop
while state == 0 or t_pid == Twist(): continue

print("STARTING")

while not rospy.is_shutdown():
    print("STATE: ", state)
    if (state == WANDERING):
        # have the robot wander around
        t_pub = wander()
    elif (state == FOUND_WALL):
        t_pub = t_pid
    elif (state == FOLLOWING):
        t_pub = t_pid
    else:
        print("STATE NOT FOUND")
    pub_vel.publish(t_pub)
    rate.sleep()
