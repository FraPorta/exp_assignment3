#!/usr/bin/env python3

## @package motion_controller
#
# makes the robot move in the map respecting the normal and sleep behaviour

import rospy
import random
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import CompressedImage
import math
import actionlib
import actionlib.msg
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


VERBOSE = False

# default behaviour
behaviour = None
# home position
home = [rospy.get_param('home_x'), rospy.get_param('home_y')]
# Action client goal init
goal_pos = MoveBaseGoal()

# action client init
act_c = None
# publishers for home poisition reaching
pubHome = rospy.Publisher("/home_reached", Bool, queue_size=1)

# home_reached publisher init
home_reached = False

## function get_random_position
#
# get a random position on the map
def get_random_position():
    randX = random.randint(-10, 10)
    randY = random.randint(-10, 10)
    randPos = [randX, randY]
    return randPos

## function get_behaviour
#
# subscriber callback to the behaviour topic
def get_behaviour(state):
    global behaviour
    behaviour = state.data

## function feedback_cb
#
# callback to send_goal function
def feedback_cb(feedback):
    # while the goal is being reached, check if the behaviour changes
    if behaviour == 'play' or behaviour == 'sleep':
        rospy.loginfo("The behaviour has changed! Canceling goal...")
        act_c.cancel_all_goals()

## function move_normal
#
# movement in the NORMAL state
def move_normal():
    
    # get a random position
    pos = get_random_position()

    goal_pos.target_pose.header.frame_id = "map"
    goal_pos.target_pose.header.stamp = rospy.Time.now()
    # set robot goal position
    goal_pos.target_pose.pose.position.x = pos[0]
    goal_pos.target_pose.pose.position.y = pos[1]
    goal_pos.target_pose.pose.position.z = 0
    goal_pos.target_pose.pose.orientation.w = 2
    # send robot position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos, feedback_cb=feedback_cb)
    rospy.loginfo("Robot goal position sent:")
    rospy.loginfo(goal_pos.target_pose.pose.position)
    act_c.wait_for_result(rospy.Duration.from_sec(240.0))
    result = act_c.get_result()
    if result:
        rospy.loginfo("Robot has reached the goal!")


## function move_sleep
#
# movement in the SLEEP state
def move_sleep():
    global home_reached

    goal_pos.target_pose.header.frame_id = "map"
    goal_pos.target_pose.header.stamp = rospy.Time.now()
    # set robot goal position
    goal_pos.target_pose.pose.position.x = home[0]
    goal_pos.target_pose.pose.position.y = home[1]
    goal_pos.target_pose.pose.position.z = 0
    goal_pos.target_pose.pose.orientation.w = 2
    # send robot position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos)
    rospy.loginfo("Robot goal position sent!")
    rospy.loginfo(goal_pos.target_pose.pose.position)
    act_c.wait_for_result(rospy.Duration.from_sec(240.0))
    result = act_c.get_result()
    if result:
        rospy.loginfo("Robot has reached the home position in time, now sleeps")
        home_reached = True

## function main
#
# init action client and choose how the robot moves checking its state (behaviour)
def main():
    # init node
    rospy.init_node("motion_controller")
    rate = rospy.Rate(20)
    global act_c, home_reached

    # subscriber to current behaviour
    rospy.Subscriber("/behaviour", String, get_behaviour)
    rospy.loginfo("Subscribed to the behaviour")

    # initialize action client
    act_c = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    # wait for the initialization of the server for 10 seconds
    act_c.wait_for_server(rospy.Duration(10))

    while not rospy.is_shutdown():
        # move the robot according to behaviour
        if behaviour == "sleep":
            if not home_reached:
                move_sleep()
                if home_reached:
                    pubHome.publish(home_reached)
        else:
            # reinitialize home_reached
            home_reached = False

            if behaviour == "normal":
                # wait random time
                move_normal()
                rospy.sleep(1)


if __name__ == "__main__":
    main()
