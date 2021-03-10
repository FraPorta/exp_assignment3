#!/usr/bin/env python

## @package human_interaction_gen
#
# Human interactions with the ball: makes the ball move or go underground

import rospy
import random
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
from exp_assignment2.msg import PlanningAction, PlanningActionGoal

act_c = None
goal_pos = PlanningActionGoal()

## function get_random_position
#
# get a random position on the map for the ball
def get_random_position():
    randX = random.randint(-8, 8)
    randY = random.randint(-8, 8)
    randZ = 0.5
    randPos = [randX, randY, randZ]
    return randPos

    
## function main
#
# initialize node, action client and makes the ball move on the map or underground
def main():
    # init node
    rospy.init_node("human_interaction_generator")
    rate = rospy.Rate(20)

   

    while not rospy.is_shutdown():
        
        rospy.sleep(random.randint(7,10))

        rate.sleep()


if __name__ == "__main__":
    main()
