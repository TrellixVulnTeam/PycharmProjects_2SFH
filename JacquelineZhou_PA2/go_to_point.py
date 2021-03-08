# function for the robot to go back to original point
def going_back(seeker_yaw):
    # calculate how far it is from the original point
    distance = sqrt(abs(seeker_x-hider_x)*abs(seeker_x-hider_x) + abs(seeker_y-hider_y)*abs(seeker_y-hider_y))
    turn_to = calc_turn_to()
    # making turn
    if seeker_yaw < 0 :
        seeker_yaw = seeker_yaw + 2*pi
    print(seeker_yaw)
    target = abs(seeker_yaw - turn_to)
    if turn_to > seeker_yaw:
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

# function to calculate which angle the robot should be at
def calc_turn_to():
    if hider_x > seeker_x and hider_y < seeker_y:
        turn_to = 3*pi/2 + atan2(abs(hider_x - seeker_x), abs(hider_y - seeker_y))
    elif hider_x > seeker_x and hider_y > seeker_y:
        turn_to = pi/2 - atan2(abs(hider_x - seeker_x), abs(hider_y - seeker_y))
    elif hider_x < seeker_x and hider_y > seeker_y:
        turn_to = pi/2 + atan2(abs(hider_x - seeker_x), abs(hider_y - seeker_y))
    elif hider_x < seeker_x and hider_y < seeker_y:
        turn_to = 3*pi/2 - atan2(abs(hider_x - seeker_x), abs(hider_y - seeker_y))
    elif hider_x == seeker_x and hider_y > seeker_y:
        turn_to = pi/2
    elif hider_x == seeker_x and hider_y < seeker_y:
        turn_to = 3*pi/2
    elif hider_y == seeker_y and hider_x < seeker_x:
        turn_to = pi
    elif hider_y == seeker_y and hider_x > seeker_x:
        turn_to = 0
    return turn_to

"""
# initialize position
# change these in odom callbacks
seeker_yaw = 0
seeker_x = 0
seeker_y = 0
hider_x = 0
hider_x = 0
"""