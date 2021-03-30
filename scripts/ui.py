#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *

# Cheking the user input 
def check_user_input(input):
    try:
        # Convert it into integer
        temp_val = int(input)
        if temp_val in range(1,5):
            print("Input is a valid integer number.\n Saving state:", temp_val)
            state = temp_val
        else :
            print("Input is not belonging to the range 1, 5")
    except ValueError:
        try:
            # Convert it into float
            temp_val = float(input)
            print("Input is a float  number. Please, enter an integer one")
        except ValueError:
            print("Input is a string instead of a number. \n I am exiting the program!")


# service callback
def set_new_pos(req):
    print("Please insert the desired robot state: \n")
    print("Available states: \n")
    print("First state: (state 1)- move randomly \n")
    print("Second state: (state 2) - target position \n")
    print("Third state: (state 3) -  walls following \n")
    print("Fourth state: (state 4) - stop \n")
    print("Fifth state: (state 5) - bug algorithm \n")
			  
    # check between 1 and 4 after 20 line 
    state = input("Enter a state by specifying an integer between 1 and 5:\n")
    # ensuring right entrance by the user 
    check_user_input(state)
     
    rospy.set_param("state", state)
    #the state has been changed
    rospy.set_param('change_state',1)
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
