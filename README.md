# Research Track I - second assignment

In this second assignment, we were asked to develop a software architecture for controlling a robot within a specific environment.
More in details, our software will rely on two different packages:

1. the move_base pakage
2. the gmapping package

The first one, will take care of the localization of our robot within the enviroment, whereas the second one will plan the motion.

## User request ##

The architecture will provide a way to get the **user request**, and will make the robot capable of executing one of the following behaviors (depending on the user’s input):

1. Move randomly in the environment, by choosing 1 out of 6 possible target position
    Position | Coordinates
    ------------ | -------------
    Position 1 | (-4,-3)
    Position 2 | (-4,2)
    Position 3 | (-4,7)
    Position 4 | (5,-7)
    Position 5 | (5,-3)
    Position 6 | (5,1)

2. Directly ask the user for the **next** target position (checking that the position is one of the possible six) and reach it)
3. Start following the external walls
4. Stop in the last position
5. (optional) change the planning algorithm to _dijkstra_ (move_base) to the `bug0`

# How the architecture is structured

Please, find in the following sections a brief summary of the various ROS messages, services, parameters, and other sub-structures used to realise this project

## Messages

### Published messages

In the list below, the **published messages** of the whole architecture are reported:

* **actionlib_msgs/GoalID**: It publishes the messages to the **/move_base/cancel** topic in order to remove a target as it is considered reached. Indeed, it allows to easily avoid  the inadvertent overlying of robot _distinct_ behaviours

* **geometry_msgs/Twist**: It publishes the messages to the **/cmd_vel** topic. Moreover, it is used:
  1. for setting both the robot linear and and angular velocity
  2. for halting the robot.

* **move_base_msgs/MoveBaseActionGoal**: It publishes the messages to the **/move_base/goal** topic. Moreover, it is used for setting the move_base goal that the robot has to achieve

### Subscribed messages

In the list below, the **subscribed messages** of the whole architecture are reported:

* **geometry_msgs/Point**: It defines the robot position, expressed as a **Point**

* **nav_msgs/Odometry**: It provides the current robot position by means of the **/Odom** topic

* **sensor_msgs/LaserScanl** It provides the real-time laser output through the **/scan** topic

## Services

As far as services are concerned, I have here-reported a quick explanation of their purposes:

1. Within the [random_srv.py](https://github.com/fedehub/final_assignment/tree/main/scripts/random_srv.py) script, there is the **/random_srv** service. Its purpose is to choose between six different coordinate positions

2. Within  the [go_to_point_service_m.py](https://github.com/fedehub/final_assignment/tree/main/scripts/go_to_point_service_m.py) script, there is  the **/go_to_point_switch** service. Its purpose is to _activate/deactivate_ the `go-to-point` behaviour

3. Within the [ui.py](https://github.com/fedehub/final_assignment/tree/main/scripts/ui.py) script, we find the **/ui** service. its purpose consists in providing a tool for selecting one out of the `five` possible robot states.

4. Within the [user_interface.py](https://github.com/fedehub/final_assignment/tree/main/scripts/user_interface.py) script, we find the **/user_interface service**. Its purpose basically consists in providing a tool for selecting the _x,y coordinates_ of the next desired robot target position, once the previous target has been reached. Moreover, this is the _user-interface_ exploited by the `bug algorithm`

5. Within the [wall_follower_service_m.py](https://github.com/fedehub/final_assignment/tree/main/scripts/wall_follower_service_m.py) script, we find the **/wall_follower_switch** service. Its purpose is to  _activate/deactivate_  the `wall-follower`behaviour

6. Within the [wall_follower_bug.py](https://github.com/fedehub/final_assignment/tree/main/scripts/wall_follower_bug.py) script, we find the **/wall_follower_bug** service.  
Its purpose is to  _activate/deactivate_  the `wall-follower` behaviour.
> _REMARK:_ This is the service exploited by the `bug algorithm`!

7. Within the [bug_m.py](https://github.com/fedehub/final_assignment/tree/main/scripts/bug_m.py) script, we find the **/bug_alg** service. Its purpose is to  _activate/deactivate_ the `bug algorithm` behaviour (through a boolean value)

## Nodes

Within the [scripts](https://github.com/fedehub/final_assignment/tree/main/scripts) folder, the following nodes are available:

1. [master_node.py](https://github.com/fedehub/final_assignment/tree/main/scripts/master_node.py)
It implements the structure of the entire architecture. It handles the simulation structure and it provides a way for checking the robot state. Then, it triggers the required service.

2. [user_interface.py](https://github.com/fedehub/final_assignment/tree/main/scripts/user_interface.py)
As its name suggests, It provides a service, exploited by the **bug algorithm**,for letting the user choose which **target position** the robot should achieve.

3. [ui.py](https://github.com/fedehub/final_assignment/tree/main/scripts/ui.py)
It provides a user-interface in which the user has to select 1 out of 5 available states. Then, once inserted the chosen state, a "valid state" **message** appears, so that the user can save his/her choice by typing the string "done"
> _REMARK:_ When entering the "done" string or other strings, remember to add the quote marks ""

4. [wall_follower_bug.py](https://github.com/fedehub/final_assignment/tree/main/scripts/wall_follower_bug.py)
It provides a service for avoiding the collision between our robot and the neighboring walls while performing the target achievement
> _REMARK:_ We need to distinguish between _wall_follower_bug.py_ and _wall_follower_service_m.py_ because even if they share the same code structure, when calling the latter service, both 'bug algorithm' and its _wall_follower_service_m_ are disabled!

5. [wall_follower_service_m.py](https://github.com/fedehub/final_assignment/tree/main/scripts/wall_follower_service_m.py)
As its name suggests, it provides a service for simulating the wall following behaviour

6. [go_to_point_service_m.py](https://github.com/fedehub/final_assignment/tree/main/scripts/go_to_point_service_m.py)
It provides a service for making the **bug algorithm*** capable of implementing the `go-to-point` behaviour  

7. [random_srv.py](https://github.com/fedehub/final_assignment/tree/main/scripts/random_srv.py)
It provides a straightforward service, for setting a **random position** which the robot will achieve

8. [bug_m.py](https://github.com/fedehub/final_assignment/tree/main/scripts/bug_m.py)
It provides a service for carrying out the `bug algorithm` behaviour

## Parameters

They are contained in the [launch](https://github.com/fedehub/final_assignment/tree/main/launch) folder. More specifically:

* Within the [node_master.launch](https://github.com/fedehub/final_assignment/blob/main/launch/node_master.launch) file, I have initialised:
  *  `des_pos_x` and `des_pos_y` for allocating the _x,y coordinates_ of the target the user has chosen (so that the robot can reach them)
  * `state` for individuating the current state (between _1 and 5_) of the robot
  * `change_state` for initialising at _zero_ the current state of our robot. It has been proven very useful during the implementation of the **bug algorithm**

## Launch files


## Documentation

The documentation of this project, obtained by means of **DoxyGen** is visible, within the [docs](https://github.com/fedehub/final_assignment/tree/main/docs) folder

## Release History

* `0.1.0`
 * The first proper release

* `0.0.1`
 * Work in progress

## Meta

Federico Civetta– s4194543 – fedeunivers@gmail.com

Distributed under none licence

https://github.com/fedehub

## Contributing

1. Fork it (https://github.com/fedehub/final_assignment/fork)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request
