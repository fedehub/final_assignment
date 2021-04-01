#! /usr/bin/env python

## @package wall_follower_bug
#  Documentation for the wall_follower_bug.py module.
#
#  More details.
#
# It provides a service for avoiding the collision between our robot and the neighboring walls while performing the target achievement. It is exploited by the bug algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
# publisher
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
# initialising the state 
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

## Documentation for the wall_follower_switch function.
#
#  More details.
#
# @param req the object pointer
# @var active_ activate/deactivate_the service
# @var res returns a boolean as the response of the service

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

## Documentation for the clbk_laser function.
#
#  More details.
#
# @param msg the object pointer of type LaserScan
# @var regions_ specifies how far the walls are from the laser sources

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()

## Documentation for the change_state function.
#
#  More details.
#
# @param state the object pointer
# @var state_ 

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

## Documentation for the take_action function.
#
#  More details.
#
# This function check the laser output. By doing so, it is possible to know where
# the surrounding walls are located with respect to the specific orientation settled
# by the robot. Moreover, the robot will change its state depending on where the walls
# are detected 
#
# @var regions 
# @var msg The stop message of Twist type

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(1)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

## Documentation for the find_wall function.
#
#  More details.
#
# This function publishes both the angular and linear robot velocity
# @var msg The message published for detecting the surrounding walls 
def find_wall():
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.6
    return msg

## Documentation for the turn_left function.
#
#  More details.
#
# This function publishes the angular robot velocity
# @var msg The message published for turning the robot left
def turn_left():
    msg = Twist()
    msg.angular.z = 0.8
    return msg

## Documentation for the turn_left function.
#
#  More details.
#
# This function publishes the angular robot velocity
# @var msg The message published for following the wall, by setting the linear velocity along x direction
# of 0.5

def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main():
    global pub_, active_
    # initialising the node 
    rospy.init_node('reading_laser')
    # initialising the publisher for setting the velocity through /cmd_vel topic
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # initialising the publisher for obtaining the Laser scan through the /scan topic
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    # initialising the service /wall_follower_bug for following the walls 
    srv = rospy.Service('wall_follower_bug', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
	    # cheking the status 
	    # choose one of the available states to simulate a specific behavior
	    # the function retrn the message that has to be published through the /cmd_vel topic
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
