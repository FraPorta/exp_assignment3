#!/usr/bin/env python

## @package behaviour_controller
#
# state machine to control the behaviour of the pet
# States: NORMAL, SLEEP, PLAY, TRACKNORMAL, FIND, TRACKFIND

import os
import rospy
import smach
import smach_ros
import random
import roslaunch
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
            #########################################################################################################
            elif (random.randint(1,1000000) == 1 and time_passed > 30): ######### RICORDATI DI CAMBIARE RATE !!!!!!
                #####################################################################################################
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

            

## class state TrackNormal
#
# track behaviour of the pet when arriving from Normal state
class TrackNormal(smach.State):
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
        rospy.loginfo('Executing state TRACK NORMAL')
        pub_state.publish("track_normal")
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


## class state TrackFind
#
# track behaviour of the pet when arriving from Find state
class TrackFind(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['return_find', 'return_play']
                            )
        
        
        self.rate = rospy.Rate(20)  # Loop at 20Hz

    ## method execute
    #
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK FIND')
        pub_state.publish("track_find")
        self.ball_reached = False
        self.room_found = False

        ## check if the ball is reached
        rospy.Subscriber("/ball_reached", Bool, self.get_ball_reached)
        ## check if the room found is correct
        rospy.Subscriber("/room_found", Bool, self.get_room_found)

        while not rospy.is_shutdown():  

            if (self.ball_reached and not self.room_found):
                ## if the room was not correct, continue to search
                return 'return_find'
            elif (self.ball_reached and self.room_found):
                ## if the room was found, return to Play behaviour
                return 'return_play'

            self.rate.sleep()
    
    ## method get_ball_detection
    #
    # subscriber callback for ball detection
    def get_ball_reached(self, ball):
        self.ball_reached = ball.data
    
    ## method get_room_found
    #
    # subscriber callback for ball detection
    def get_room_found(self, room):
        self.room_found = room.data
        


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
        rospy.Subscriber("/home_reached", Bool, self.get_home_reached)
        
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
    def get_home_reached(self,home):    
        self.home_reached = home.data
    

## class state Play
#
# Play behaviour of the pet
class Play(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stop_play', 'go_find'],
                            )
        self.rate = rospy.Rate(20)

    ## method execute
    #
    # state execution
    def execute(self,userdata):
        rospy.loginfo('Executing state PLAY')
        pub_state.publish("play")
        
        self.location_unknown = False
        rospy.Subscriber("/no_room", Bool, self.get_location_unknown)
        # init timer
        count = 0
        init_time = rospy.Time.now()

        while not rospy.is_shutdown():  
            # count time passed from the start of PLAY state
            if count == 1:
                init_time = rospy.Time.now()
            count = count + 1
            current_time = rospy.Time.now()
            time_passed = current_time.secs - init_time.secs

            if self.location_unknown:
                ## if the location sent by the human is unknown, go to the FIND state
                return 'go_find'
            if (time_passed > random.randint(240,360)):
                ## after 4-6 minutes return to Normal state
                return 'stop_play'

            # loop 
            self.rate.sleep()

    ## method get_location_unknown
    #
    # subscriber callback for room location knowledge
    def get_location_unknown(self, room):
        self.location_unknown = room.data

## class state Find
#
# Find behaviour of the pet
class Find(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['return_play', 'go_track'],
                            )
        self.rate = rospy.Rate(20)

    ## method execute
    #
    # state execution
    def execute(self,userdata):
        rospy.loginfo('Executing state FIND')
        pub_state.publish("find")
        
        
        self.ball_detected = False
        #self.room_unknown = False

        rospy.Subscriber("/ball_detected", Bool, self.get_ball_detection)
        #rospy.Subscriber("/no_room", Bool, self.get_no_room)

        rospy.loginfo("Path is: %s", os.path.dirname(os.path.abspath(__file__)))
        rospy.loginfo("Path is: %s", os.path.abspath(os.getcwd()))
        

        ## launch explore-lite package
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/francesco/expRob_ws/src/exp_assignment3/launch/explore.launch"])
        launch.start()
        rospy.loginfo("explore_lite started")

       
        
        

        # init timer
        count = 0
        init_time = rospy.Time.now()

        while not rospy.is_shutdown():  
            # count time passed from the start of PLAY state
            if count == 1:
                init_time = rospy.Time.now()
            count = count + 1
            current_time = rospy.Time.now()
            time_passed = current_time.secs - init_time.secs

            if (time_passed > random.randint(300,600)):
                # stop explore-lite
                launch.shutdown()
                rospy.loginfo("Stopped explore_lite")
                ## after 4-6 minutes return to Play state
                return 'return_play'

            if self.ball_detected:
                # stop explore-lite
                launch.shutdown()
                rospy.loginfo("Stopped explore_lite")
                
                ## If the robot sees the ball goes to the Track substate
                return 'go_track'      

            # loop 
            self.rate.sleep()
        
    ## method get_ball_detection
    #
    # subscriber callback for ball detection
    def get_ball_detection(self, ball):
        self.ball_detected = ball.data
        
        #def get_no_room(self, room):
        #    self.room_unknown = room.data
    
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
                                            'go_track':'TRACKNORMAL'})

        smach.StateMachine.add('TRACKNORMAL', TrackNormal(), 
                               transitions={'return_normal':'NORMAL'})

        smach.StateMachine.add('TRACKFIND', TrackFind(), 
                               transitions={'return_find':'FIND',
                                            'return_play':'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop_play':'NORMAL', 
                                            'go_find':'FIND'})

        smach.StateMachine.add('FIND', Find(), 
                               transitions={'return_play':'PLAY',
                                            'go_track':'TRACKFIND'})
        

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

