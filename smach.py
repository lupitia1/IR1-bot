#!/usr/bin/env python

import rospy
import smach

import numpy as np
import cv2  #opencv-python (cv_bridge)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#To work with images
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


################
#In our previous example, we had global variables, in this case these variables
#will be stored in the blackboard (userdata)
#we create an empty global variable
#laser=None
#frame=None
#encoding=None
#########


def img_callback(data, userdata):
    bridge = CvBridge()
    #imgmsg_to_cv2 allows converting ROS messages into image matrices used by OpenCV
    # (numpy.ndarray)
    frame = bridge.imgmsg_to_cv2(data)
    encoding = data.encoding

    #IMPORTANT FOR STUDENT ....
    # COMMENT THE NEXT LINES WHEN YOU DO NOT NEED TO SEE THE IMAGE ANY MORE....
    #the image is shown
    cv2.imshow('Image viewer', frame)
    cv2.waitKey(1)

    #we can change the encoding..., by defautl the message Image is in mode RGB
    #by default this encoding is preserved
    #but it is possible to change it to BGR which is the encoding used by OpenCV
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    encoding = 'BGR'

    ############
    #HERE IS WHERE WE PUT THE INFORMATION IN THE BLACKBOARD
    ##########

    userdata.frame = frame


def scan_callback(msg, userdata):
    #######
    #HERE IS WHERE WE PUT THE INFORMATION IN THE BLACKBOARD
    ########

    userdata.laser = msg
    userdata.laser_available = True


#definition of the states
class move_straight(smach.State):

    def __init__(self):
        ### initializing the state, pay attention to the outcomes and input keys,
        #input keys are to declare which information of the blackboard is going to be used
        #outcomes are the possible outcomes of the state... (I don's use of all them)

        smach.State.__init__(self, outcomes=['success', 'failure', 'obstacle_detected','highway_free'],
                             input_keys=['laser', 'frame', 'laser_available'])

    def execute(self, userdata):
        ##### we can use global variables without any problem....besides the blackboard...
        # ... but blackboard is a more controlled way of sharing....

        global pub
        move = Twist()
        #The number withing brackets represent the control frequency (in this example is 4Hz)
        rate = rospy.Rate(4)

        d = 15

        while d > 0.55:  #This is the control cycle, it keeps moving the robot until something is close (below 0.75 m)

            if (userdata.laser_available):
                n = len(userdata.laser.ranges)
                first_quarter = min(userdata.laser.ranges[:n // 4])
                fourth_quarter = min(userdata.laser.ranges[3 * n // 4:])
                d = min(first_quarter, fourth_quarter)

                # Check if highway is free (all distances are large enough)
                if d > 1.0:  #threshold for "highway free"
                    move.linear.x = 0.0  # Stop the robot before speeding up
                    pub.publish(move)
                    return 'highway_free'

            move.linear.x = 0.25
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()

        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)

        #it exits the state with this outcome
        return ('obstacle_detected')


class turn_around(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'free_space_detected'],
                             input_keys=['laser', 'laser_available'])

    def execute(self, userdata):
        #print ("I am going to turn around")
        #I am going to turn the robot until there isn't anything in front

        global pub
        move = Twist()
        #The number withing brackets represent the control frequency (in this example is 4Hz)
        rate = rospy.Rate(4)

        d = 0
        while d < 0.35:  #This is the control cycle, it keeps turning the robot until there is not obstacle in front

            if (userdata.laser_available):
                n = len(userdata.laser.ranges)
                first_quarter = min(userdata.laser.ranges[:n // 4])
                fourth_quarter = min(userdata.laser.ranges[3 * n // 4:])
                d = min(first_quarter, fourth_quarter)

            move.linear.x = 0.0
            move.angular.z = 0.35
            pub.publish(move)
            rate.sleep()

        move.linear.x = 0.0
        move.angular.z = 0.0
        pub.publish(move)

        #it leaves the state with this outcome
        return ('free_space_detected')

class SpeedUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['slow_down', 'failure'],
                             input_keys=['laser', 'laser_available'])

    def execute(self, userdata):
        global pub
        move = Twist()
        rate = rospy.Rate(4)  # Control frequency: 4 Hz

        if not userdata.laser_available:
            rospy.loginfo("Laser data unavailable.")
            return 'failure'

        rospy.loginfo("Speeding up...")

        while True:
            if userdata.laser_available:
                # Calculate minimum distance in the front
                n = len(userdata.laser.ranges)
                first_quarter = min(userdata.laser.ranges[:n // 4])
                fourth_quarter = min(userdata.laser.ranges[3 * n // 4:])
                d = min(first_quarter, fourth_quarter)

                # Check if distance is below 0.80 meters to slow down
                print('distance is:',d)
                if d < 0.80:
                    rospy.loginfo("Obstacle detected at {:.2f}m. Slowing down...".format(d))
                    move.linear.x = 0.0
                    pub.publish(move)
                    return 'slow_down'

            # Increase speed when no obstacles are near
            move.linear.x = 0.5  # Faster speed
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()


## principal module


rospy.init_node('smatch_arquitecture')

#we now create the machine
machine = smach.StateMachine(outcomes=['success', 'failure'])
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback, callback_args=machine.userdata)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
machine.userdata.laser = LaserScan()
machine.userdata.laser_available = False

#### IMPORTANT FOR THE STUDENT
#if you do not wish to work with images comment the next line...
img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback, callback_args=machine.userdata)

#we create the states
move_straight_state = move_straight()
turn_around_state = turn_around()
speed_up_state = SpeedUp()
#stop_and_wait_state = StopAndWait()
#explore_randomly_state = ExploreRandomly()
#wall_follow_state = WallFollow()

with machine:
    #we add the states to the machine...
    smach.StateMachine.add('MOVE_STRAIGHT', move_straight_state,
                           transitions={'success': 'success',
                                        'failure': 'failure',
                                        'obstacle_detected': 'TURN_AROUND',
                                        'highway_free': 'SPEED_UP'})

    smach.StateMachine.add('TURN_AROUND', turn_around_state,
                           transitions={'success': 'success',
                                        'failure': 'failure',
                                        'free_space_detected': 'MOVE_STRAIGHT'})

    smach.StateMachine.add('SPEED_UP', speed_up_state,
                           transitions={'slow_down': 'MOVE_STRAIGHT',
                                        'failure': 'failure'})

    #smach.StateMachine.add('STOP_AND_WAIT', stop_and_wait_state,
    #                       transitions={'continue': 'MOVE_STRAIGHT'})

    #smach.StateMachine.add('EXPLORE_RANDOMLY', explore_randomly_state,
    #                       transitions={'success': 'MOVE_STRAIGHT',
    #                                    'failure': 'failure'})

    #smach.StateMachine.add('WALL_FOLLOW', wall_follow_state,
    #                       transitions={'success': 'MOVE_STRAIGHT',
    #                                    'failure': 'TURN_AROUND'})

#we run the machine
machine.execute()
