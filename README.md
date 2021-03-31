# Assignment 3 Experimental Robotics 

## Introduction
This architecture is intended to spawn a wheeled robot in a simulated environment composed by six rooms, divided by walls. In each room there is a coloured ball, used by the robot to identify its position in the house. The robot behaviour is controlled using a finite state machine, that has four states (Normal, Sleep, Play, Find) and two substates (Track Normal and Track Find). The objective of the robot is to map the entire environment, identify all the six rooms and store their position. The sensors equipped by the robot are a Camera and a Laser Scan.

## Software architecture and state diagram
### Architecture

<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/exp_ass3_diagram.png?raw=true">
</p>

#### Components
* Human Interaction Generator
* Behaviour Controller
* Motion Controller
* Ball Tracking 

#### Packages
* move_base
* gmapping
* explore_lite

#### Description
The main architecture is composed by four components and three external packages to plan and execute the movement of the robot in the environment,to create a map and to explore the unknown parts of the map.

The **Human Interaction Generator** component simulates a human who can execute two actions: send *Play* commands to the robot at random (the frequency can be changed through a ROS parameter set in the *scripts* launch file), and send *go_to* commands with the name of the room where the robot should go, when it is in the Play behaviour and in front of the human waiting for the command.

The **Behaviour Controller** component contains the finite state machine and is responsible of changing the behaviour of the robot publishing the state on a topic every time it changes, so that the other components can change their behaviour accordingly. The four behaviours are: Normal (which is the initial one), Sleep, Play and Find. Normal and Find also have a substate, respectively Track Normal and Track Find. The details will be covered in the State Machine section.

The **Motion Controller** component handles the robot motion when the behaviour is set to Normal, Sleep or Play. It subscribes to the */behaviour* topic in order to get the current state of the State Machine. 
In the Normal state, it chooses a random position in the environment, getting a random number for the x and y position between -8 and 8 for *y* and -6 and 6 for *x*, which are the maximum dimensions of the simulated map. Then it sends the goal to the *move_base* action server and waits for it to be achieved. If the state changes when a goal is yet to be reached, it cancels the current goal using the callback of the *send_goal()* function so that the robot can change its behaviour accordingly and immediately.\
In the Sleep state the motion controller sends the home position (retrieved from the Ros parameters) as the goal to the  move_base Action Server and waits. When the goal is reached it publishes on the */home_reached* topic to alert the Behaviour controller that the robot is at home.\
In the Play state there are two possibilities: if the robot is not in front of the human, it reaches the human position (always using the *move_base* action server), notifies to the *human interaction generator* node on the */human_reached* topic that it is in front of him, and waits for a *go_to* command; when the robot receives the go_to command, if the location is already known (stored in the ROS parameters server) it moves to that location, waits a random time and then returns to the human position waiting for another *go_to* command. If the location is unknown, it publishes on the */no_room* topic and the behaviour changes to *Find*.    
 
The **Ball tracking** component implements the openCv algorithm to detect the ball (more precisely the color of the ball) and controls the robot movements in the *Track Normal* and *Track Find* behaviours. It subscribes to the robot camera topic (*/robot/camera1/image_raw/compressed*) and, inside the subscriber callback, it uses the OpenCv libraries to detect the balls in the environment. When a ball is detected and the robot is in the *Normal* or *Find* behaviour, it immediately sends a message on the */ball_detected* topic for the Behaviour controller. Since there are six balls of different colours and more than one could be detected at the same time, there is a function called *get_mask_colour()* that selects the bigger (and so closer) ball detected. Then the corresponding mask is created and when the state has transitioned from Find to Track Find or from Normal to Track Normal, the node publishes velocities to the */cmd_vel* topic in order to make the robot reach the closer ball position. At this point, the position of the ball is stored on the parameter server with the name of the ball colour and a msg is published on the */ball_reached* topic to alert the behaviour controller.
If the robot is in the Track Normal state, it returns to the Normal one.  
When the robot is in the Track Find substate, if the room corresponding to the detected colour was the one specified by the human in the last *go_to* command, the one that made the robot transition from Play to Find, the node publishes *True* on the */room_found* topic. If that is the case, it means that the searched room is finally found and the robot will return to the Play behaviour; if on the contrary the room is not the right one, it publishes *False* on the */room_found* topic and the robot returns to the Find state.
This node also implements obstacle avoidance using the LaserScan sensor, to allow the robot to avoid the walls while reaching the balls.

The **move_base** package provides an implementation of an action that, given a goal in the world, will attempt to reach it with the mobile base of the robot.

The **gmapping** package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called *slam_gmapping*. Using *slam_gmapping*, you can create a 2-D occupancy grid map  from laser and pose data collected by the mobile robot.

The **explore_lite** package provides greedy frontier-based exploration. When the node is running, robot will greedily explore its environment until no frontiers could be found. Movement commands will be send to *move_base*.

#### Ros Parameters
* home_x &rarr; home x position on the map
* home_y &rarr; home y position on the map

* sleep_freq &rarr; frequency of the Sleep behaviour
* play_freq &rarr; frequency of Play command sent by the user

Parameters for rooms-colors correspondence
*  Entrance : Blue
*  Closet : Red
*  Livingroom : Green
*  Kitchen : Yellow
*  Bathroom : Magenta
*  Bedroom : Black

* explore_lite parameters
* gmapping parameters
  
#### Ros Topics 
* /play_command &rarr; topic on which the *human_interaction_gen* node sends the command to make the robot go in the Play behaviour
* /behaviour &rarr; topic on which the current state is published by the behaviour controller
* /ball_detected &rarr; topic on which it is published when the ball is detected or not using a Bool
* /ball_reached  &rarr; topic on which *ball_tracking* publishes when the robot is in front of the ball
* /room_found  &rarr; topic on which *ball_tracking* publishes if the searched room has been found
* /no_room  &rarr;  topic on which *motion_controller* publishes if the room requested by the GoTo command has been already explored 
* /home_reached &rarr; topic on which it is published when the home is reached or not during the Sleep behaviour using a Bool
* /human_reached &rarr; topic on which it is published when the human is reached or not during the Play behaviour using a Bool
* /camera1/image_raw/compressed &rarr; topic used to retrieve the images from the robot camera 
* /cmd_vel &rarr; topic used to publish velocities to the robot by the *ball_tracking* component and the *move_base* package
* /odom &rarr; topic used to get the robot odometry
* /scan &rarr; topic for the LaserScan output, used by ball_tracking and move_base to avoid obstacles
* /map &rarr; topic on which the map (OccupancyGrid) created by gmapping is published
* /goal &rarr; goal for the *move_base* action server
* /result &rarr; result of the *move_base* action server


### State Machine
This is the state machine inside the Behaviour Controller component
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/StateMachine.png?raw=true">
</p>

The **Normal** behaviour consists in moving randomly around the map. Whenever the ball is detected by the *ball_tracking* node, it goes to the Track Normal substate, otherwise it can randomly go to the *Sleep* state after at least 10 seconds have passed in the *Normal* behaviour or in the *Play* state if a *play_command* is received from the *human_interaction_gen* node.\
In the **Track Normal** behaviour the robot is controlled by the *ball_tracking* node,  that makes it reach the detected ball and store its position. After that, it returns to the *Normal* behaviour  

The **Sleep** behaviour consists in going to the home position and staying there for some time. The transition to the *Normal* state happens after a random time period (20-40 seconds), that starts after the robot has reached the home position. 

In the **Play** behaviour the robot goes in front of the human and waits for a *'go_to'* command, which is a string representing one of the rooms in the house. If the room position was already stored in the ROS parameters, the robot will remain in the *Play* state, it will reach the desired room, stay there for some time and then return to the human position. If the room position is unknown, the robot will switch to the *Find* state. After some time (random between 2 and 6 minutes) in the Play state, the robot will return to the Normal state.

In the **Find** behaviour the robot will explore the environment using the explore_lite package. When a ball is detected, it will switch to the *Track Find* substate. After some time (random between 4 and 7 minutes), it returns to the *Play* behaviour.
In the **Track Find** behaviour the robot is controlled by the *ball_tracking* node, that makes it reach the detected ball and store its position. After that, it checks if the room corresponding to the detected ball is the one requested by the user in the last *go_to* command: if yes, the robot returns to the Play state; if not, the robot returns to the Find Behaviour and continues to search for the room.

## Contents of the repository
Here the content of the folders contained in this repository is explained

### Config
Contains the configuration file for Rviz
### Documentation
Contains the html documentation of the project (in order to see it, open the *index.html* file in a web browser like Chrome or Firefox)
### Launch
Contains the necessary launch files. gmapping.launch and move_base.launch are used to launch the two packages from the other launch files.
*simulation.launch* opens the gazebo simulation (spawning the robot, the human, the balls and the world) and Rviz and launches the gmapping package. 
*scripts.launch* runs the nodes in the *src* folder and *move_base* and loads the parameters in the server
### Param
Contains the yaml files that define the parameters needed to run the *move_base* package
### Src
Contains the four python files (the components) of the main architecture: *human_interaction_gen.py*, *behaviour_controller.py*, *motion_controller.py* and *ball_tracking.py*
### Urdf
Contains the descriptions of the robot model and the gazebo file, and the description of the human. 
### Worlds
in the worlds folder there is the description of the house that will be loaded on gazebo.


## Installation and running procedure
First install this three packages, if you don't already have them on your machine: 
```console
sudo apt-get install ros-<ros_distro>-openslam-gmapping
sudo apt-get install ros-<ros_distro>-navigation
sudo apt-get install ros-<ros_distro>-explore-lite
```

The first thing to do, after cloning the repository in your Ros workspace, is to build the package, using the following command in the workspace:

```console
catkin_make
```
In order to run the system, you have to launch the two following launch files in this order. The first one loads the gazebo world and runs the gmapping package, the second one runs the move_base package, loads the necessary ros parameters and launches the rest of the nodes. You can modify the frequency of the Sleep and Play behaviours from the `scripts.launch` launchfile.

```console
roslaunch exp_assignment3 simulation.launch
roslaunch exp_assignment3 scripts.launch 
```

## System’s features
The system manages to achieve the expected behaviours in all the tested situations: in two long sessions of continuous run of the architecture the robot did not show strange behaviours and managed to reach and store all the locations in the house, as you can see in the picture at the end of this paragraph (all the colours are present in the ROS parameter server, which means that all the rooms positions were explored and stored).
Moreover, whatever state the robot is in, it always runs an obstacle avoidance algorithm, either if it is using the move_base package, either if it is reaching the balls in the Track substates. In fact, I implemented a simple wall avoidance algorithm inside the node that makes the robot track and reach the balls in the environment, so that it does not get stuck in the walls while reaching them.
Another system feature is the fact that when in the Normal and Find state, the robot will always be ready to cancel the current goal and transition to the required state or substate, thanks to the capabilities of the Action Server-Client system of *move_base* and the feedback messages.
Finally, I used randomness to stress the system: the *play* commands sent by the *human_interaction_gen* node are sent at completely random times (you can change the "frequency" in the launch file) and the locations contained in the *go_to* commands are selected randomly between the six rooms each time. Moreover, also the waiting times of the robot when sleeping or when it arrives to a location and the time passed before changing behaviours (from Play to Normal, from Normal to Sleep or from Find to Play) are chosen randomly. The system in my tests still managed to mantain the expected behaviour during the whole simulation, also with all these random factors.

<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/AllLocationsReached_2.png?raw=true">
</p>

## System’s limitations
In general, one of the main limitations of the system is the fact that it may need a long time to detect and store all the ball positions in the house, since the user requests and world exploration in the Normal state are completely random.
It may be possible that, using the explore_lite package in the Find behaviour, the robot could not find the user-requested location in time. This may happen because the exploration is only based on the knowledge of the map and not on the already stored room positions. 



## Possible technical improvements
Exploring the environment using a knowledge-based approach instead of explore_lite, using the already known locations to improve the time the robot needs to find the requested loaction and avoid returning to the Play behaviour without having found the correct room.



## Rqt_graph
### Main Architecture and Gazebo Simulation
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/rosgraph_assignment3_explore.png?raw=true">
</p>

## Author
Francesco Porta\
E-mail: francy857@gmail.com\
ID: 4376330

