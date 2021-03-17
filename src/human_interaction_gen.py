#!/usr/bin/env python

## @package human_interaction_gen
#
# Human interactions with the ball: makes the ball move or go underground

import rospy
import random
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
from std_msgs.msg import String, Bool

pub_play = None
pub_go_to = None

## function get_random_position
#
# get a random position on the map for the go to command
def get_random_position():
    pos_array = ["Entrance", "Closet", "Livingroom", "Kitchen", "Bathroom", "Bedroom"]
    randPos = pos_array[random.randint(0, 5)]
    return randPos

    
## function main
#
# initialize node, action client and makes the ball move on the map or underground
def main():
    # init node
    rospy.init_node("human_interaction_generator")
    rate = rospy.Rate(20)

    pub_play = rospy.Publisher("/play_command", Bool, queue_size=1)
    pub_go_to = rospy.Publisher("/go_to_command", String, queue_size=1)
   

    while not rospy.is_shutdown():
        
        rospy.sleep(random.randint(7,10))

        rate.sleep()


if __name__ == "__main__":
    main()
