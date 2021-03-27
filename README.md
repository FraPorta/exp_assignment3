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

The **Human Interaction Generator** component 

The **Behaviour Controller** component contains the finite state machine and is responsible of changing the behaviour of the robot publishing the state on a topic every time it changes, so that the other components can change their behaviour accordingly. The four behaviours are: Normal (which is the initial one), Sleep, Play and Find. Normal and Find also have a substate, respectively Track Normal and Track Find. The details will be covered in the State Machine section.

The **Motion Controller** component handles the robot motion when the behaviour is set to Normal, Sleep or Play. It subscribes to the */behaviour* topic in order to get the current state of the State Machine. 
In the Normal state, it chooses a random position in the environment, getting a random number for the x and y position between -8 and 8 for *y* and -6 and 6 for *x*, which are the maximum dimensions of the simulated map. Then it sends the goal to the *move_base* action server and waits for it to be achieved. If the state changes when a goal has yet to be reached, using the callback of the *send_goal* function it cancels the current goal so that the robot can change its behaviour accordingly and immediately.\
In the Sleep state the motion controller sends the home position (retrieved from the Ros parameters) as the goal to the  move_base Action Server and waits. When the goal is reached it publishes on the */home_reached* topic to alert the Behaviour controller that the robot is at home.\
In the Play state 

The **Ball tracking** component implements the openCv algorithm to detect the ball (more precisely the color of the ball) and makes the robot follow the ball when the actual behaviour is Play. It subscribes to the robot camera topic (*/robot/camera1/image_raw/compressed*) and, inside the subscriber callback, it uses the OpenCv libraries to detect the ball in the environment. When the ball is detected it immediately sends a message on the */ball_detected* topic for the Behaviour controller. Then when the state is transitioned to the Play one, it publishes velocities to the */robot/cmd* topic in order to make the robot follow the ball. When the ball stops, and so also the robot stops, it stops tracking the ball and publish commands to the */robot/joint_position_controller/command* topic to make the head revolute joint of the robot move to the right, then to the left and then back to the default position. After having finished it returns to track the ball and follow it until the state changes or the ball stops again.

#### Planning and Obstacle avoidance
* move_base
* g_mapping
* explore_lite

#### Ros Parameters
* home_x &rarr; home x position on the map
* home_y &rarr; home y position on the map

#### Ros Topics 
I will list only the ros topics directly related to the code that I developed (not the ones only related to gazebo, ros_control, or the ones created by the Action Servers)
* /behaviour &rarr; topic on which the current state is published by the behaviour controller
* /ball_detected &rarr; topic on which it is published when the ball is detected or not using a Bool
* /home_reached &rarr; topic on which it is published when the home is reached or not during the Sleep behaviour using a Bool
* /robot/joint_position_controller/command &rarr; topic used to move the robot head joint
* /robot/camera1/image_raw/compressed &rarr; (/robot/camera in the diagram) topic used to retrieve the images from the robot camera 
* /robot/cmd_vel &rarr; topic used to publish velocities to the robot by the ball tracking component and the Go to point robot Action Server
* /robot/odom &rarr; topic used to get the robot odometry in the Go to point robot Action Server
* /robot/reaching_goal &rarr; family of topics of the action server for the robot
* /ball/cmd_vel &rarr; topic used to publish velocities to the ball by the Go to point ball Action Server
* /ball/reaching_goal &rarr; family of topics of the action server for the ball


### State Machine
This is the state machine inside the Behaviour Controller component
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/StateMachine.png?raw=true">
</p>

The **Normal** behaviour consists in moving randomly around the map. Whenever the ball is detected by the *ball_tracking* node, it goes to the Track Normal substate, otherwise it can randomly go to the *Sleep* state after at least 10 seconds have passed in the *Normal* behaviour or in the *Play* state if a *play_command* is received from the *human_interaction_gen* node.\
In the **Track Normal** behviour the robot is controlled by the *ball_tracking* node,  that makes it reach the detected ball and store its position. After that, it returns to the *Normal* behaviour  

The **Sleep** behaviour consists in going to the home position and staying there for some time. The transition to the *Normal* state happens after a random time period (20-40 seconds), that starts after the robot has reached the home position. 

In the **Play** behaviour the robot goes in front of the human and waits for a *'go_to'* command, which is a string representing one of the rooms in the house. If the room position has already been stored in the ros parameters, the robot will remain in the *Play* state, it will reach the desired room, stay there for some time and then return to the human position. If the room position is unknown, the robot will switch to the *Find* state. After some time (random between 2 and 6 minutes) in the Play state, the robot will return to the Normal state.

In the **Find** behaviour the robot will explore the environment using the explore_lite package. When a ball is detected, it will switch to the *Track Find* substate. After some time (random between 4 and 7 minutes), it returns to the *Play* behaviour.
The **Track Find**

## Contents of the repository
Here the content of the folders contained in this repository is explained

### Config
Contains the yaml configuration file for the joint_position_controller and joint_state_controller, managed by the ros_control plugin
### Documentation
Contains the html documentation of the project (in order to see it, open the *index.html* file in a web browser like Chrome or Firefox)
### Launch
Contains two launch files. One (*gazebo_world.launch*) is for showing on gazebo the simulated world and spawning  the human, the robot, the ball and their relative action servers and joint controller.
The other one is for the behaviour architecture that manages the robot and ball movements.
### Scripts
Contains the two action servers python files: *go_to_point_ball.py* and *go_to_point_robot.py*
### Src
Contains the four python files (the components) of the main architecture: *human_interaction_gen.py*, *behaviour_controller.py*, *motion_controller.py* and *ball_tracking.py*
### Urdf
Contains the descriptions of the robot model, the ball and their relative gazebo files, and the description of the human. The description of the robot has been modified to include two new links and corresponding joints, a fixed one for the neck and a revolute for the head. Moreover a transmission motor has been included to make the robot head position controllable using the ros_control plugin.
### Worlds
in the worlds folder there is the description of the simulation world that will be loaded on gazebo.


## Installation and running procedure
First install this three packages, if you don't already have them on your machine: 
```console
sudo apt-get install ros-<ros_distro>-openslam-gmapping
sudo apt-get install ros-<ros_distro>-navigation
sudo apt-get install ros-<ros_distro>-explore-lite
```

The first thing to do, after having cloned the repository in the Ros workspace, is to build the package, using the following command in the workspace:

```console
catkin_make
```
In order to run the system, you have to launch the two following launch files in this order, the first one loads the gazebo world and runs the gmapping package, the second one runs the move_base package, loads the necessary ros parameters and launches the rest of the nodes. You can modify the frequency of the Sleep and Play behaviours from the `scripts.launch` launchfile.

```console
roslaunch exp_assignment3 simulation.launch
roslaunch exp_assignment3 scripts.launch 
```

## System’s features


## System’s limitations


## Possible technical improvements




## Rqt_graph
### Main Architecture and Gazebo Simulation
<p align="center"> 
<img src="https://github.com/FraPorta/Itslit/blob/master/rosgraph_assignment3_explore.png?raw=true">
</p>

## Author
Francesco Porta\
E-mail: francy857@gmail.com\
ID: 4376330

