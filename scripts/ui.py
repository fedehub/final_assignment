#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *

# service callback


def set_new_pos(req):
    print("Please insert the desired robot state: ")
    print("Available states: ")
    print("First state: (state 1)- move randomly")
    print("Second state: (state 2) - target position")
    print("Third state: (state 3) -  walls following")
    print("Fourth state: (state 4) - stop")
    print("Fifth state: (state 5) - bug algorithm")
			  
    # check between 1 and 4 after 20 line 
    state = int(raw_input('chosen state: '))	
    
    rospy.set_param("state", state)
    print("State has been succesfully chosen")
    return []


def main():
    rospy.init_node('ui')

    srv = rospy.Service('ui', Empty, set_new_pos)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
