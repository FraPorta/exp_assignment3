#!/usr/bin/env python3

# @package ball_tracking
#
# uses cv2 libraries to track the balls in the map follow them and save the positions of the rooms

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
from random import randint
import math

VERBOSE = False

# define colours limits
greenLower = (50, 50, 20)
greenUpper = (70, 255, 255)

blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)

redLower = (0, 50, 50)
redUpper = (5, 255, 255)

yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)

blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)

magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)


# class ball_tracking
#
# implements a ball tracking algorithm using cv2
class ball_tracking:
    # method __init__
    #
    # initialization of the ball_tracking class
    def __init__(self):
        # variables initialization
        self.ball_detected = False
        self.near_ball = False
        self.center = None
        self.radius = None
        self.behaviour = None
        self.ball_reached = False
        self.colour = None
        self.current_pos = None

        # publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pubBall = rospy.Publisher("/ball_detected", Bool, queue_size=1)
        self.pub_reach = rospy.Publisher("/ball_reached", Bool, queue_size=1)

        # subscriber to camera
        self.cam_sub = rospy.Subscriber(
            "/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # subscriber to current behaviour
        rospy.Subscriber("/behaviour", String, self.get_behaviour)

    # method get_behaviour
    #
    # subscriber callback to the behaviour topic

    def get_behaviour(self, state):
        self.behaviour = state.data

    # method get_behaviour
    #
    # subscriber callback to the behaviour topic
    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position

    # method get_mask_colour
    #
    # method to decide which colour mask will be applied
    def get_mask_colour(self, maskGreen, maskBlack, maskRed, maskYellow, maskBlue, maskMagenta):
        sumGreen = np.sum(maskGreen)
        sumBlack = np.sum(maskBlack)
        sumRed = np.sum(maskRed)
        sumYellow = np.sum(maskYellow)
        sumBlue = np.sum(maskBlue)
        sumMagenta = np.sum(maskMagenta)

        # rospy.loginfo([sumGreen, sumBlack, sumRed, sumYellow, sumBlue, sumMagenta])
        sumArray = np.array([sumGreen, sumBlack, sumRed,
                            sumYellow, sumBlue, sumMagenta])
        max_ind = np.argmax(sumArray)

        # return the mask related to the colour with higher value of detection
        if max_ind == 0:
            return [maskGreen, 'Green']
        elif max_ind == 1:
            return [maskBlack, 'Black']
        elif max_ind == 2:
            return [maskRed, 'Red']
        elif max_ind == 3:
            return [maskYellow, 'Yellow']
        elif max_ind == 4:
            return [maskBlue, 'Blue']
        elif max_ind == 5:
            return [maskMagenta, 'Magenta']
        else:
            return [maskGreen, 'None']    # default (the masks are all zeroes)

    '''
    # method follow_ball
    #
    # publish velocities to follow the ball
    def follow_ball(self):
        if self.ball_reached:
            # if the ball stops, stop completely the robot
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.vel_pub.publish(twist_msg)
        else:
            # if the ball is detected go towards it and start following it
            if self.ball_detected:
                if self.near_ball:
                    # if near enough to the ball start following it
                    twist_msg = Twist()
                    twist_msg.angular.z = -0.003*(self.center[0] - 400)
                    twist_msg.linear.x = -0.01*(self.radius - 100)
                    self.vel_pub.publish(twist_msg)
                else:
                    # if not near enough go towards the ball
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.4
                    self.vel_pub.publish(twist_msg)
    '''

    # method follow_ball
    #
    # publish velocities to follow the ball
    def follow_ball(self):
        if self.near_ball:
            # if near enough to the ball start following it
            twist_msg = Twist()
            twist_msg.angular.z = -0.003*(self.center[0] - 400)
            twist_msg.linear.x = -0.01*(self.radius - 100)
            self.vel_pub.publish(twist_msg)
        else:
            # if not near enough go towards the ball
            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            self.vel_pub.publish(twist_msg)
        
    # method callback
    #
    # Callback function of subscribed topic.
    # Here images get converted and features detected
    def callback(self, ros_data):

        if VERBOSE:
            print('received image of type: "%s"' % ros_data.format)

        angular_z = None
        linear_x = None
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # create masks for all colours
        maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
        maskBlack = cv2.inRange(hsv, blackLower, blackUpper)
        maskRed = cv2.inRange(hsv, redLower, redUpper)
        maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
        maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
        maskMagenta = cv2.inRange(hsv, magentaLower, magentaUpper)
        # choose the correct mask
        mask_colour = self.get_mask_colour(maskGreen, maskBlack, maskRed, maskYellow, maskBlue, maskMagenta)

        mask = cv2.erode(mask_colour[0], None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        self.center = None

        # rospy.loginfo(mask_colour[1])

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            self.radius = radius
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            self.ball_detected = True

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, self.center, 5, (0, 0, 255), -1)

                self.near_ball = True
            else:
                self.near_ball = False
            
        else:
            self.ball_detected = False

        # publish if the ball has been detected
        # self.pubBall.publish(self.ball_detected)

        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # if behaviour is track, go near the ball
        if self.behaviour == "track_normal":
            if self.ball_detected or self.near_ball:
                angular_z = -0.003*(self.center[0] - 400)
                linear_x = -0.01*(self.radius - 100)
                # if the robot is almost still (in front of the ball)
                if abs(angular_z) < 0.02 and abs(linear_x) < 0.02:
                    # signal that the ball has been reached
                    self.ball_reached = True
                    self.colour = mask_colour[1]
                    
                    # save information about ball position
                    self.save_info(self.colour)
                    rospy.loginfo("The ball has been reached!")
                    # publish that the ball has been reached
                    self.pub_reach.publish(self.ball_reached)

                    # reinitialize variable
                    self.ball_reached = False

                # else follow the ball
                else: 
                    # track the ball 
                    self.follow_ball()
            

        # publish if the ball has been detected
        if mask_colour[1] != self.colour:
            self.pubBall.publish(self.ball_detected)
        else:
            self.pubBall.publish(False)


    # method save_info
    #
    # Saves the position of the tracked ball
    def save_info(self, colour):
        rospy.loginfo("Saving position of the %s ball", colour)
       
        ball_pos = [self.current_pos.x, self.current_pos.y]
        rospy.loginfo("Ball Position: %s", str(ball_pos))

        rospy.set_param(colour, ball_pos)


# function main
#
# init node and ball tracking class, and checks when the robot head should be moved
def main(args):
    # initialize ball tracking node
    rospy.init_node('ball_tracking', anonymous=True)

    rate = rospy.Rate(100)

    # Initializes class
    bt = ball_tracking()

    while not rospy.is_shutdown():

        rate.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
