#! /usr/bin/env python

## @package bug_m
#  Documentation for the bug_m.py module.
#
#  More details.
#
# It provides a service for carrying out the **bug algorithm** behaviour.
# This module represents the master of the bug algorithm structure. Indeed it
# communicates with the otehr nodes.

import rospy
import time # for using sleep() function
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math

# initialisation of global variables 
pub = None
srv_client_ui_=None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_client_user_interface_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
# available states
state_desc_ = ['Go to point', 'wall following', 'target reached']
state_ = 0
first=True
clock_start=0
# 0 - go to point
# 1 - wall following

# callbacks
active_=False

## Documentation for the clbk_odom function.
#
#  More details.
#
# @param msg the object pointer of type Odometry
# @var position_ memorise the actual robot position
# @var yaw_ memorise the robot orientation

def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

## Documentation for the clbk_laser function.
#
#  More details.
#
# @param msg the object pointer of type LaserScan
# @var regions_ specifies how far the walls are from the laser sources
#
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

## Documentation for the active_bug function.
#
#  More details.
#
# @param req the object pointer
# @var active_ activate/deactivate_the service
# @var res returns a boolean as the response of the service
#
def active_bug(req):
    global active_
    active_ = req.data
    if active_==False:
        first = True
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

## Documentation for the change_state function.
#
#  More details.
#
# @param state the object pointer
# @var state_ the actual robot state
#

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global clock_start
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        # publish message to stop the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        clock_start=time.clock()



def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    time.sleep(2)
    # initiialisation of global variables
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub,srv_client_ui_
    global active_,first,clock_start
    # initialisation of the bug0 node
    rospy.init_node('bug0')

    # initialising Subscriber for /scan topic. As argument it takes clbk_laser Callback
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    # initialising Subscriber for /scan topic . As argument it takes clbk_odom Callback
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    # initialising Pubscriber for /cmd_vel topic .
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # initialising services as service client
    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_bug', SetBool)
    srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)
    # initialisng the bug algorithm service as a service server
    srv = rospy.Service('bug_alg', SetBool, active_bug)
    srv_client_ui_ = rospy.ServiceProxy('/ui', Empty)

    # initialised to "stop" state
    change_state(2)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      if active_== False:
            continue
      else:
        # getting the actual clock time
        clock_time = time.clock()
        # initializing expired_time
        expired_time = clock_time - clock_start
        # if expired_time is over 20 seconds,
        if expired_time > 20:
            first = True
            active_ = False
            # blocking the robot
            change_state(2)
            print ('Unfortunatelly the target is not reachable! If you want to exploit the bug algorithm, please insert: 5')
            # calling our user interface
            resp=srv_client_ui_()

        # going on if time is not expired
        else:
            # if there are not walls in the neighborhood
            if regions_ == None:
                continue
            # if the go to point behavior is set to True
            if state_ == 0:
                err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                        2) + pow(desired_position_.x - position_.x, 2))
                if(err_pos < 0.3):
                    change_state(2)

                elif regions_['front'] < 0.5:
                    change_state(1)

            # if the wall follower behavior is set to True
            elif state_ == 1:
                desired_yaw = math.atan2(
                    desired_position_.y - position_.y, desired_position_.x - position_.x)
                err_yaw = normalize_angle(desired_yaw - yaw_)
                err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                        2) + pow(desired_position_.x - position_.x, 2))

                if(err_pos < 0.3):
                    change_state(2)
                if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                    change_state(0)

            # if the robot is stopped
            elif state_ == 2:
                if first==False:
		           print('target has been reached! If you want to exploit the bug algorithm, please insert: 5')
		           resp=srv_client_ui_()

                # if first = true (if it is the first time, i call the user_interface)
                resp = srv_client_user_interface_()
                # starting the clock
                clock_start = time.clock()
                time.sleep(2)

                # the second time i got in the loop, first will be initialize as false
                # and i will go back to the if loop above
                first=False

                # getting the goal position of x and y coordinates
                desired_position_.x = rospy.get_param('des_pos_x')
                desired_position_.y = rospy.get_param('des_pos_y')
                err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                        2) + pow(desired_position_.x - position_.x, 2))
                if(err_pos > 0.35):
                    change_state(0)

        rate.sleep()


if __name__ == "__main__":
    main()
