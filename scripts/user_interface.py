#! /usr/bin/env python


## @package user_interface
#  Documentation for the user_interface module.
#
#  More details.
#
#  Within user interface.py we find the `/user_interface service`.
#  Its purpose basically consists in
#  providing a tool for selecting the x,y coordinates of the next desired
#  robot target position, once the previous target has been reached. Moreover,
#  this is the user-interface exploited by the bug algorithm

# import ros stuff
import rospy
from std_srvs.srv import *

## Documentation for the set_new_pos function.
#
#  More details.
#
# @param req the object pointer, returning the new position.
# @var x the x coordinate entered by the user
# @var y the y coordinate entered bu the user
def set_new_pos(req):
    print("Target reached! Please insert a new position")
    x = float(raw_input('x :'))
    y = float(raw_input('y :'))
    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    print("Thanks! Let's reach the next position")
    return []

## Documentation for the main function.
#
#  More details.
#
#
# @var x the x coordinate taken by means of get_param method
# @var y the y coordinate taken by means of get_param method
# @var srv the /user_interface service. It has set_new_pos as Callback argument
# @var rate useful to fix the frequency of the loop

def main():

    ## method for initialisng the node user_interface
    rospy.init_node('user_interface')

    x = rospy.get_param("des_pos_x")
    y = rospy.get_param("des_pos_y")

    srv = rospy.Service('user_interface', Empty, set_new_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
