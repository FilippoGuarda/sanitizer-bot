#!/usr/bin/python
# -*- coding: utf-8 -*-

from importlib.resources import read_text
from operator import sub
from unittest import result
import rospy
import roslaunch
import numpy as np
import actionlib
import time
import math
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, \
    Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from uv_visualization.msg import lowestIrradiation, subMapCoords
from localization.msg import local


class Disinfection:

    ########## INITIALIZATION ##########

    def __init__(self):

        # Variables/Flags

        self.initFlag = 1
        self.finishedFlag = 0
        self.currentPose = [0, 0]
        self.laser_scan_values = []
        self.angle = []
        self.data = []
        self.greyscale = []
        self.goalToPublish = PoseWithCovarianceStamped()
        self.wait = []
        self.client = []
        self.msg1 = Twist()
        self.state = []
        self.flag1 = True
        self.flag2 = True
        self.localization_ended = False


        # Localization
        self.localization_subscriber = rospy.Subscriber("/localization", 
                local, self.localization_callback)

        # Occupancy

        self.occupancy_subscriber = rospy.Subscriber('/map',
                OccupancyGrid, self.occupancy_callback)

        # Odometry

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry,
                self.odom_callback)

        # Laser

        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan,
                self.laser_callback)

        # uv_map

        self.uv_map_subscriber = rospy.Subscriber('/lowestIrradiation', lowestIrradiation,
                self.irradiation_callback)

        self.sub_pub = rospy.Publisher("/submap", subMapCoords, queue_size=10)

        # Movebase

        self.pub = rospy.Publisher('goal_minimum_energy',
                                   PoseWithCovarianceStamped,
                                   queue_size=1)

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.movebase_subscriber = rospy.Subscriber('goal_minimum_energy',
                             PoseWithCovarianceStamped,
                             self.movebase_client)

########## MOVE_BASE ##########

    def movebase_client(self, msg):
        # we create an instance of movebasegoal based on a simple goal in form of (x, y) pos

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.x = msg[0]
        self.y = msg[1]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(goal)
        self.client.send_goal(goal)
        self.wait = self.client.wait_for_result()
        state = self.client.get_state()
        if not self.finishedFlag:

            # If the result doesn't arrive, assume the Server is not available

            if not self.wait:
                rospy.logerr('Action server not available!')
                rospy.signal_shutdown('Action server not available!')
            else:

                # Result of executing the action

                self.state = self.client.get_state()
                rospy.loginfo('STATE OF GOAL' + str(state))
                return self.client.get_result()
        else:

            rospy.loginfo('Coronavirus destroyed!')
            rospy.signal_shutdown('Coronavirus destroyed!')

########## ODOMETRY ##########

    def odom_callback(self, msg):
        self.currentPose[0] = msg.pose.pose.position.x
        self.currentPose[1] = msg.pose.pose.position.y

        # Orientation

        self.angle = msg.pose.pose.orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        yaw_deg = math.degrees(yaw)
        self.currentAngle = yaw_deg  # rotation in degrees of the robot (0-360)

    def laser_callback(self, msg):
        self.laser_scan_values = msg.ranges
        laser1 = np.asarray(self.laser_scan_values)
        laser2 = np.concatenate((laser1[0:37], laser1[323:359]),
                                axis=None)
        laser3 = np.concatenate((laser1[143:179], laser1[180:217]),
                                axis=None)
        laser4 = np.concatenate((laser1[0:25], laser1[335:359]),
                                axis=None)
        laser5 = np.concatenate((laser1[155:179], laser1[180:1204]),
                                axis=None)
        laser = np.concatenate((laser2, laser3), axis=None)
        if (any(laser2 < 0.3) or any(laser3 < 0.3)) \
            and not self.initFlag:
            self.client.cancel_all_goals()
            self.msg1.linear.x = 0.0
            self.msg1.angular.z = 0.2
            self.vel_pub.publish(self.msg1)
            if all(laser4 > 0.25) and self.flag2 == True:
                self.msg1.linear.x = 0.1
                self.msg1.angular.z = 0.0
                self.vel_pub.publish(self.msg1)
                self.flag1 = False
            elif all(laser5 > 0.25) and self.flag1 == True:
                self.msg1.linear.x = -0.1
                self.msg1.angular.z = 0.0
                self.vel_pub.publish(self.msg1)
                self.flag2 = False

######### OCCUPANCY ##########

    def occupancy_callback(self, msg):
        self.data = np.asarray(msg.data,
                               dtype=np.int8).reshape(msg.info.height,
                msg.info.width)
        self.greyscale = np.copy(self.data).astype(np.uint8)
        self.greyscale[self.greyscale == 255] = 0
        self.greyscale[self.greyscale == 0] = 0
        self.greyscale[self.greyscale == 100] = 255

######### LOWEST IRRADIATION ##########

    def irradiation_callback(self, msg):
        self.lowest_x = msg.lowest_x
        self.lowest_y = msg.lowest_y
        self.roomDone = msg.room_done

########## LOCALIZATION ##########

    def localization_callback(self, msg):
        self.localization_ended = msg.loc_done


########## MAIN ##########

def main():
    obc = Disinfection()

    
    #TODO: move localization to separate file
    rospy.init_node('simple_class', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    cellSize = 0.2  # Size of a grid's cell (m)

    # the following is a semaphore-like waiting block
    # that ensures localization execution

    # waits for the start of the localization node
    while obc.localization_ended:
        rate.sleep()
    rospy.loginfo("Localization Started")   

    # waits for the end of the localization node
    while not obc.localization_ended: 
        rate.sleep()
    rospy.loginfo("Localization ended")


    # Read the rooms file

    rooms = open('/home/watch/Documents/uv_bot/src/sanitizer_navigation/src/rooms.txt', 'r')

    for (num, room) in enumerate(rooms, start=1):

        obc.initFlag = 1

        # Printing the goal

        rospy.loginfo('Room ' + str(num) + ' received!')

        # Split the string when ", " is found and create a list with the divided strings

        roomList = room.split(', ')

        # Top-right corner coordinates

        x1 = float(roomList[2])
        y1 = float(roomList[3])

        # Bottom-left corner coordinates

        x2 = float(roomList[0])
        y2 = float(roomList[1])

        coords = subMapCoords()
        coords.x1 = x1 
        coords.y1 = y1 
        coords.x2 = x2 
        coords.y2 = y2 

        rospy.loginfo(coords)
        obc.sub_pub.publish(coords)

        # Select as starting point the center of the room

        start_x = (x2 + x1)/ 2  
        start_y = (y2 + y1)/ 2 

        # Call the movebase function to reach the initial point in the room
        rospy.loginfo(f"center of the room is at {start_x} and {start_y}")
        result_init = obc.movebase_client((start_x, start_y))
        if result_init:
            rospy.loginfo("I'm in the room")
        time.sleep(1)
        
        #obc.initFlag = 0
        input("Enter to proceed \n")
        # Kill coronavirus in the room

        tim = time.time()
        nTarget = 0
        while not rospy.is_shutdown() and not obc.roomDone:

            x_goal = obc.lowest_x
            y_goal = obc.lowest_y

            # Publish the goal to reach

            rospy.loginfo(str(x_goal) + " " + str(y_goal) + " " + str(obc.roomDone))

            # Goal published synchronised with the timer initialized before

            rospy.loginfo(obc.wait)
            """ if time.time() - tim > 4:
                obc.goalToPublish.pose.pose.position.x = x_goal
                obc.flag2 = True
                obc.flag1 = True
                obc.goalToPublish.pose.pose.position.y = y_goal
                obc.pub.publish(obc.goalToPublish)
                tim = time.time() """
            
            next_pos = obc.movebase_client((x_goal, y_goal))
            if next_pos:
                rospy.loginfo(f"target {nTarget} reached")
                nTarget+1 


            rate.sleep()

    # Set flag to terminate the node

    obc.finishedFlag = 1


if __name__ == '__main__':
    main()
