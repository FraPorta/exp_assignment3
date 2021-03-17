#!/usr/bin/env python

## @package behaviour_controller
#
# state machine to control the behaviour of the pet
# States: NORMAL, SLEEP, PLAY

import rospy
import smach
import smach_ros
import random
from std_msgs.msg import String
from std_msgs.msg import Bool


## current behaviour publisher
pub_state = rospy.Publisher("/behaviour",String,queue_size=1)

## class state Normal
#
# normal behaviour of the pet
class Normal(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_to_sleep','go_play','go_track']
                            )

        self.rate = rospy.Rate(20)  # Loop at 20Hz

    ## method execute
    #
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state NORMAL')
        pub_state.publish("normal")

        self.play_command = False
        self.ball_detected = False

        ## check if a voice command is received
        rospy.Subscriber("/ball_detected", Bool, self.get_ball_detection)
        rospy.Subscriber("/play_command", Bool, self.get_play_command)
        count = 0

        init_time = rospy.Time.now()

        while not rospy.is_shutdown():  
            # count time passed from the start of Normal state
            if count == 1:
                init_time = rospy.Time.now()
            count = count + 1
            current_time = rospy.Time.now()
            time_passed = current_time.secs - init_time.secs

            #if (self.ball_detected and time_passed > 5):
            if (self.ball_detected):
                ## If the robot sees the ball goes to the Track substate
                return 'go_track'
            
            elif (self.play_command):
                ## If a play command is received, go to the Play state
                return 'go_play'    

            elif (random.randint(1,1000000) == 1 and time_passed > 30): # RICORDATI DI CAMBIARE RATE
                ## go to sleep at random 
                return 'go_to_sleep'
            
        
                    
            self.rate.sleep()
    
    ## method get_ball_detection
    #
    # subscriber callback for ball detection
    def get_ball_detection(self, ball):
        self.ball_detected = ball.data

    ## method get_play_command
    #
    # get command from human interaction generator
    def get_play_command(self, command):
        self.play_command = command.data

            

## class state Track
#
# track behaviour of the pet
class Track(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['return_normal']
                            )
        
        
        self.rate = rospy.Rate(20)  # Loop at 20Hz

    ## method execute
    #
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK')
        pub_state.publish("track")
        self.ball_reached = False

        ## check if the ball is detected
        rospy.Subscriber("/ball_reached", Bool, self.get_ball_reached)

        while not rospy.is_shutdown():  

            if (self.ball_reached):
                ## If the robot does not detect the ball anymore return to the Normal state
                return 'return_normal'

            self.rate.sleep()
    
    ## method get_ball_detection
    #
    # subscriber callback for ball detection
    def get_ball_reached(self, ball):
        self.ball_reached = ball.data
            


## class state Sleep
#
# Sleep behaviour of the pet
class Sleep(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up']
                            )
        
        self.rate = rospy.Rate(20)
        
    ## method execute
    #
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state SLEEP')
        pub_state.publish("sleep")

        self.home_reached = False
        # home position reached subscriber
        sub_home = rospy.Subscriber("/home_reached", Bool, self.get_home_reached)
        
        while not rospy.is_shutdown():  
            # check if the robot is in home position
            if(self.home_reached):
                ## wait some time to wake up
                rospy.sleep(random.randint(20,40))
                self.home_reached = False
                return 'wake_up'
            self.rate.sleep()
        
    ## method get_home_reached
    #
    # subscriber callback, gets if the robot is in the home position
    def get_home_reached(self,home_reached):    
        self.home_reached = home_reached.data
    

## class state Play
#
# Play behaviour of the pet
class Play(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stop_play'],
                            )
        self.rate = rospy.Rate(20)

    ## method execute
    #
    # state execution
    def execute(self,userdata):
        rospy.loginfo('Executing state PLAY')
        pub_state.publish("play")
        
        self.location_unknown = False
        #sub_home = rospy.Subscriber("/home_reached", Bool, self.get_home_reached)
        count = None

        while not rospy.is_shutdown():  
            # count time passed from the start of Normal state
            if count == 1:
                init_time = rospy.Time.now()
            count = count + 1
            current_time = rospy.Time.now()
            time_passed = current_time.secs - init_time.secs

            #if self.location_unknown:
                #return 'go_find'
            if (time_passed > random.randint(240,360)):
                ## after 4-6 minutes return to Normal state
                return 'stop_play'

            

            # loop 
            self.rate.sleep()

    ## method get_home_reached
    #
    # subscriber callback for room location knowledge
    def get_location_unknown(self, room):
        self.location_unknown = room.data
        
    
## function main 
#
# Initiaize the state machine
def main():
    rospy.init_node("behaviour_controller")

    # wait for the initialization of the system
    rospy.sleep(2)

    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_to_sleep':'SLEEP', 
                                            'go_play':'PLAY',
                                            'go_track':'TRACK'})

        smach.StateMachine.add('TRACK', Track(), 
                               transitions={'return_normal':'NORMAL'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop_play':'NORMAL'})

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute the state machine
    outcome = sm.execute()

    ## Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == "__main__":
    main()

