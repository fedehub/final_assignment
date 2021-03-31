#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *
import random

# service callback
def choose_rand(req):

	number=random.randint(1,6)
	if number == 1:
		rospy.set_param("des_pos_x", -4)
		rospy.set_param("des_pos_y", -3)

	elif number == 2:
		rospy.set_param("des_pos_x", -4)
		rospy.set_param("des_pos_y", 2)

	elif number == 3:
		rospy.set_param("des_pos_x", -4)
		rospy.set_param("des_pos_y", 7)

	elif number == 4:
		rospy.set_param("des_pos_x", 5)
		rospy.set_param("des_pos_y", -7)

	elif number == 5:
		rospy.set_param("des_pos_x", 5)
		rospy.set_param("des_pos_y", -3)

	elif number == 6:
		rospy.set_param("des_pos_x", 5)
		rospy.set_param("des_pos_y", 1)

        return []


def main():
    rospy.init_node('random_srv')

    srv = rospy.Service('random_srv', Empty, choose_rand)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
