#! /usr/bin/env python

## @package ui
#  Documentation for the ui.py module.
#
#  More details.
#
# It provides a service for letting the user communicate by means of
# an ad-hoc interface

# import ros stuff
import rospy
from std_srvs.srv import *


## Documentation for the check_for_input function.
#
#  More details.
#
# @param input1 the object pointer
# @var exit boolean variable for entering again the loop
# @var states_available available states
# @var value input variable to check

def check_for_input(input1):

    exit = True
    try:
        state=0
        states_available = [1,2,3,4,5]
        value = float(input1)
        # Only allows input floats
        if value in states_available:
            return value
        else:
            print("Please, enter a state between those available")
            return False
    except (ValueError, TypeError):
        print('Error, please enter a numeric input between 1 and 5')
        if exit:
            quit()
        return False

## Documentation for the set_new_pos function.
#
#  More details.
#
# @param req the object pointer
# @var count variable for debugging-issues
# @var input_number variable which memorize the user input
# @var number variable where I save the checked user's input
def set_new_pos(req):
    print("Please insert the desired robot state: \n")
    print("Available states: \n")
    print("First state: (state 1)- move randomly \n")
    print("Second state: (state 2) - target position \n")
    print("Third state: (state 3) -  walls following \n")
    print("Fourth state: (state 4) - stop \n")
    print("Fifth state: (state 5) - bug algorithm \n")

    # Initializing variable
    count = 0
    # Stays in loop until break
    while True:
        input_number = input('Please, follow these instructions.\n 1. Enter a valid state.\n 2. Press enter. \n Once "valid state!" message appears, digit "done"\n')
        if input_number == 'done':
            # exit the while loop
            break

        number = check_for_input(input_number)
        if not number:
            continue

        # counter for debugging puroses
        count += 1
        print("valid state:", number)
		# initializing state
        state = number

    # setting the parameter
    rospy.set_param("state", state)
    #the state has been changed
    rospy.set_param('change_state',1)
    print("State has been succesfully chosen")
    return []


def main():
    # initialisng the node
    rospy.init_node('ui')
    # initialing the service server. As argument it takes the set_new_pos Callback
    srv = rospy.Service('ui', Empty, set_new_pos)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
