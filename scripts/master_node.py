#! /usr/bin/env python

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

import math

# commonn publisher to publish stop!
pub_cmdvel = None
pub_mvbase = None
pub_goalid = None
# services nodes

srv_client_wall_follower_ = None # boolean node
srv_client_ui_ = None
srv_client_random_ = None
srv_client_bug_ = None

# actual robot position (global)
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
#regions_ = None

# state descriptions 

state_desc_ = ['move randomly', 'target postion', 'walls following', 'stop','bug']

# getting the state
state_= rospy.get_param('state')

# 1 - move randomly
# 2 - target position
# 3 - walls following
# 4 - stop
# 5 - bug

# callbacks
def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position

def target_distance():
    global position_
    p1 = rospy.get_param('des_pos_x')                
    p2 = rospy.get_param('des_pos_y')
    p3 = position_.x
    p4 = position_.y
    distance = math.sqrt( ((p4-p2)**2)+((p3-p1)**2) )

    return distance
      
   
# decleare and publish a message of type move_based 

def publish_ag():
        
	global pub_mvbase 
	msg_Action_Goal = MoveBaseActionGoal() # declearing msg of type MoveBaseActionGOal
	msg_Action_Goal.goal.target_pose.header.frame_id="map";
	msg_Action_Goal.goal.target_pose.pose.orientation.w=1;

	#set in the position the desired ones

	msg_Action_Goal.goal.target_pose.pose.position.x=rospy.get_param('des_pos_x');
	msg_Action_Goal.goal.target_pose.pose.position.y=rospy.get_param('des_pos_y');

	# publishing message goal
	pub_mvbase.publish(msg_Action_Goal)
	
	return[]

def check_for_input_x(input_number_x, exit = True):
    """
    Checks if the type of "input1" is a float and returns the value if so.
    Input:    input1 -- variable to check
    Output: val -- value of float
    """
    try:
        x_available = [-4,5]
        value = float(input_number_x)
        # Only allows input floats   
        if value in x_available:
            return value 
        else:
            print("Please, enter a state between those available")
            return False
    except (ValueError, TypeError):
        print('Error, please enter a numeric input within those available!')
        if exit:
            quit()
        return False

def check_for_input_y(input_number_y, exit = True):
    """
    Checks if the type of "input1" is a float and returns the value if so.
    Input:    input1 -- variable to check
    Output: val -- value of float
    """
    try:
        x_available = [-4,5]
        value = float(input_number_y)
        # Only allows input floats   
        if value in x_available:
            return value 
        else:
            print("Please, enter a state between those available")
            return False
    except (ValueError, TypeError):
        print('Error, please enter a numeric input within those available!')
        if exit:
            quit()
        return False

def change_state():
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_random_,  srv_client_bug_,srv_client_ui_, pub_cmdvel, pub_mvbase
    state_ = rospy.get_param('state')
    log = "state changed: %s" % state_desc_[state_-1]
    rospy.loginfo(log)
    change_state=rospy.set_param('change_state',0)

    # 1 - move randomly
    if state_ == 1:
        resp = srv_client_wall_follower_(False)
        resp = srv_client_bug_(False)
    resp = srv_client_random_()
	publish_ag() 
    	
    # 2 - target position
    if state_ == 2:
	    resp = srv_client_wall_follower_(False)
        resp = srv_client_bug_(False)
	    print('Please, insert the desired target position between: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)] ')
       
        # For x coordinates 
        print(' Available x coordinates: [-4,5]')                       
        while True:                               
            input_number_x = input('Please, follow these instructions.\n 1. Enter a valid x coordinate.\n 2. Press enter. \n Once "valid state!" message appears, digit "done"\n')
            if input_number_x == 'done':
                # exit the while loop 
                break                             

            x_number = check_for_input_x(input_number_x, False)
            if not x_number:
                continue
        
                                    
            print("valid state:", x_number)
            # initializing state
            x = x_number
            # checking
            print("x value is:", x)

        # For y coordinates
        print(' Available y coordinates: [-3,3,7,-7,-3-1]')
                # Stays in loop until break
        while True:                               
            input_number_y = input('Please, follow these instructions.\n 1. Enter a valid x coordinate.\n 2. Press enter. \n Once "valid state!" message appears, digit "done"\n')
            if input_number_y == 'done':
                # exit the while loop 
                break                             

            y_number = check_for_input_y(input_number_y, False)
            if not y_number:
                continue
        
                                    
            print("valid state:", y_number)
            # initializing state
            y = y_number
            # checking
            print("y value is:", y)


        # further checking 
	    print('You have chosen: x=' + str(x) +" y=" + str(y))
	    rospy.set_param("des_pos_x", x)
	    rospy.set_param("des_pos_y", y)

	    publish_ag()
	
        
        

    # walls following 
    if state_ == 3:
	    resp = srv_client_wall_follower_(True)
        resp = srv_client_bug_(False)
	    resp = srv_client_ui_()
	

    # 4 - stop
    if state_ == 4: 
	    resp = srv_client_wall_follower_(False)
        resp = srv_client_bug_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
	    twist_msg.linear.y = 0 
        twist_msg.angular.z = 0
        pub_cmdvel.publish(twist_msg)
	    print("Robot position (stopped at): x= "+ str(position_.x) +" y=" + str(position_.y))
        resp = srv_client_ui_()

    # 5 - bug algorithm 
    if state_==5:
         resp = srv_client_wall_follower_(False)
         resp = srv_client_bug_(True)
         #resp = srv_client_ui_()

def main():
    time.sleep(2)
    global position_, desired_position_, state_
    global srv_client_ui_, srv_client_wall_follower_, srv_client_bug_,srv_client_random_, pub_cmdvel, pub_mvbase, pub_goalid
    #bool_check=True
    new_state=0
    rospy.init_node('master_node')

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub_cmdvel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_mvbase = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1 )
    pub_goalid = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)


    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    srv_client_random_ = rospy.ServiceProxy('/random_srv',Empty)
    srv_client_ui_ = rospy.ServiceProxy('/ui', Empty)
    srv_client_bug_ = rospy.ServiceProxy('/bug_alg', SetBool)
    # initialize going to the point
    change_state() # to check the numb. state 

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #se lo stato viene cambiato (puo essere cambiato solo da dentro ui dopo aver inserito un input) il parametro e =1 
        #quindi  vado su in change_state() a settare il nuovo stato
        #NOTA: dentro change_state() risetto il parametro change_state=0
        #se lo stato non e stato aggiornato  quindi change_state=0 controllo se il mio stato e 0 o 1 aspetto prima di pubblicare lo ui (come prima)
        
        
        new_state=rospy.get_param('change_state')
        if new_state == 1:
            change_state()
                
        else:
            state_=rospy.get_param('state')
            
            if state_==1 or state_==2:
                a=target_distance()
                if a<0.5 :
                    msg_goalid=GoalID()
                    pub_goalid.publish(msg_goalid)
                    resp = srv_client_ui_()

    rate.sleep()


if __name__ == "__main__":
    main()
