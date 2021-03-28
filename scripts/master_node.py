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

# actual robot position (global)
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
#regions_ = None

# state descriptions 

state_desc_ = ['move randomly', 'target postion', 'walls following', 'stop']

# getting the state
state_= rospy.get_param('state')

# 1 - move randomly

# 2 - target position

# 3 - walls following

# 4 - stop

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


def change_state():
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_random_, srv_client_ui_, pub_cmdvel, pub_mvbase
    state_ = rospy.get_param('state')
    log = "state changed: %s" % state_desc_[state_-1]
    rospy.loginfo(log)


    # move randomly
    if state_ == 1:
        resp = srv_client_wall_follower_(False)
	resp = srv_client_random_()
	publish_ag() 
	
	
    	
    # target position
    if state_ == 2:
	resp = srv_client_wall_follower_(False)
	print('Please, insert the desired target position between: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)] ')
	
	x = float(raw_input('x:'))
	y= float(raw_input('y: '))	
	print('You have chosen: x=' + str(x) +" y=" + str(y))
	
	# check if value inserted is a possible one of the available positions!!
	#
	#
	rospy.set_param("des_pos_x", x)
	rospy.set_param("des_pos_y", y)

	publish_ag()
	
        
        

    # walls following 
    if state_ == 3:
	resp = srv_client_wall_follower_(True)
	resp = srv_client_ui_()
	

    # stop
    if state_ == 4: 
	resp = srv_client_wall_follower_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
	twist_msg.linear.y = 0 
        twist_msg.angular.z = 0
        pub_cmdvel.publish(twist_msg)
	print("Robot position (stopped at): x= "+ str(position_.x) +" y=" + str(position_.y))
        resp = srv_client_ui_()



def main():
    time.sleep(2)
    global position_, desired_position_, state_
    global srv_client_ui_, srv_client_wall_follower_, srv_client_random_, pub_cmdvel, pub_mvbase, pub_goalid
    bool_check=True

    rospy.init_node('master_node')

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub_cmdvel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_mvbase = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1 )
    pub_goalid = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)


    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    srv_client_random_ = rospy.ServiceProxy('/random_srv',Empty)
    srv_client_ui_ = rospy.ServiceProxy('/ui', Empty)

    # initialize going to the point
    change_state() # to check the numb. state 

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if bool_check == True:
      	    change_state()
            
        state_=rospy.get_param('state')
        #print(state_)
        if state_==1 or state_==2:
               a=target_distance()
               if a>0.5 :
                    bool_check = False
                    #print(bool_check)
               else:
                      
                    bool_check = True
                    msg_goalid=GoalID()
                    pub_goalid.publish(msg_goalid)
                    resp = srv_client_ui_()
        rate.sleep()


if __name__ == "__main__":
    main()
