#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import actionlib
import time
import math
import cv2
import random
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
import sys
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry, OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, \
    Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from uv_visualization.msg import lowestIrradiation, subMapCoords


# from nav_msgs.srv import GetPlan
# from rosservice import *

class Disinfection:

    # ######### INITIALIZATION ##########

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

        self.uv_map.subscriber = rospy.Subscriber('/lowestIrradiation',
                self.irradiation_callback)

        # Movebase

        self.pub = rospy.Publisher('goal_minimum_energy',
                                   PoseWithCovarianceStamped,
                                   queue_size=1)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.movebase_subscriber = \
            rospy.Subscriber('goal_minimum_energy',
                             PoseWithCovarianceStamped,
                             self.movebase_client)

########## MOVE_BASE ##########

    def movebase_client(self, msg):
        if self.initFlag:
            x_goal = msg[0]
            y_goal = msg[1]
        else:
            x_goal = msg.pose.pose.position.x
            y_goal = msg.pose.pose.position.y

        # Create an action client called "move_base" with action definition file "MoveBaseAction"

        self.client = actionlib.SimpleActionClient('move_base',
                MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.

        self.client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move 0.5 meters forward along the x axis of the "map" coordinate frame

        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal

        # No rotation of the mobile base frame w.r.t. map frame

        goal.target_pose.pose.orientation.w = 1.0
        state = self.client.get_state()

##########################################
        # We didn't use the replanner because is difficult to manage he behaviour and sometimes the client crashes without any reason

        # get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        # get_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        # print(resp)
        # Sends the goal to the action server.
        # state = client.get_state()
        # Waits for the server to finish performing the action.
        # wait = client.wait_for_result(rospy.Duration(20))
        # self.wait = client.wait_for_result()
        # self.tim2=time.time()
##########################################

        if self.initFlag:
            self.client.send_goal(goal)
            self.wait = self.client.wait_for_result()
        else:
            if abs(y_goal - self.currentPose[1]) >= abs(x_goal
                    - self.currentPose[0]):
                goal.target_pose.pose.orientation.z = np.sign(y_goal
                        - self.currentPose[1]) * 0.7071
                goal.target_pose.pose.orientation.w = 0.7071
            elif abs(x_goal - self.currentPose[0]) > abs(y_goal
                    - self.currentPose[1]):
                if x_goal - self.currentPose[0] < 0:
                    goal.target_pose.pose.orientation.z = 1
                    goal.target_pose.pose.orientation.w = 0

            self.client.send_goal(goal)
            self.wait = self.client.wait_for_result()

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


########## MAIN ##########

def main():
    obc = Disinfection()

    # Node initialization

    rospy.init_node('simple_class', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Parameters

    Pl = 0.1  # Power (mW)
    sampleTime = 0.1  # Sample time (s)
    minEnergy = 0  # Initial minimum energy (mJ)
    cellSize = 0.2  # Size of a grid's cell (m)
    grow = 20  # increasing of the size of the image

    # Read the rooms file

    rooms = \
        open('/home/carlopellettieri/catkin_ws/src/kill_virus/src/rooms.txt'
             , 'r')
    for (num, room) in enumerate(rooms, start=1):

        obc.initFlag = 1
        minEnergy = 0

        # Printing the goal

        rospy.loginfo('Room ' + str(num) + ' received!')

        # Split the string when ", " is found and create a list with the divided strings

        roomList = room.split(', ')

        # Top-right corner coordinates

        x1 = float(roomList[0])
        y1 = float(roomList[1])

        # Bottom-left corner coordinates

        x2 = float(roomList[2])
        y2 = float(roomList[3])

        # Create the maps to visualize the object position of the complete map, and of the room

        prova = obc.greyscale
        resized2 = cv2.flip(cv2.resize(np.copy(prova), dsize=(96, 96),
                            interpolation=cv2.INTER_CUBIC), 0)
        resized2[resized2 > 0] = 255
        resized = cv2.flip(prova, 0)
        cv2.imshow('MAP', cv2.resize(np.copy(resized2), (10
                   * resized2.shape[1], 10 * resized2.shape[0])))

        # Object detection - formation of a second cropped matrix related to the room to see where obstacles are

        center = [100 * 4 / 2 - 4, 96 * 4 / 2 - 8]
        center1 = [100 / 2 - 1, 46 - 1]
        top_right = [round(center[0] + 4 * x1 / cellSize),
                     round(center[1] - 4 * y1 / cellSize)]
        bottom_left = [round(center[0] + 4 * x2 / cellSize),
                       round(center[1] - 4 * y2 / cellSize)]

        top_right1 = [math.ceil(center1[0] + x1 / cellSize),
                      math.floor(center1[1] - y1 / cellSize)]
        bottom_left1 = [math.floor(center1[0] + x2 / cellSize),
                        math.ceil(center1[1] - y2 / cellSize)]

        sub_map = resized[top_right[1]:bottom_left[1], bottom_left[0]:
                          top_right[0]]
        img = np.copy(sub_map)
        img1 = resized2[top_right1[1]:bottom_left1[1], bottom_left1[0]:
                        top_right1[0]]
        cv2.imshow('Obstacles_map', cv2.resize(np.copy(img), (5
                   * img.shape[1], 5 * img.shape[0])))
        matrix_i = img1.shape[0]
        matrix_j = img1.shape[1]

        # Select as starting point the center of the room

        start_x = x2 + matrix_j / 2 * cellSize
        start_y = y2 + matrix_i / 2 * cellSize

        # Energy matrix of the room(mJ)

        energyUV = np.zeros((matrix_i, matrix_j))

        # Call the movebase function to reach the initial point in the room

        result_init = obc.movebase_client((start_x, start_y))
        if result_init:
            rospy.loginfo("I'm in the room")
        time.sleep(1)
        obc.initFlag = 0

        # Initialization of opencv image resized

        enel_img = np.zeros([grow * matrix_i, grow * matrix_j, 3])
        cv2.imshow('map resized', cv2.resize(np.copy(img1), (grow
                   * img1.shape[1], grow * img1.shape[0])))
        cv2.waitKey(40)
        img21 = cv2.dilate(resized2,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                           (3, 3)), iterations=1)
        img2 = img21[top_right1[1]:bottom_left1[1], bottom_left1[0]:
                     top_right1[0]]

        cv2.imshow('map resized and dilated', cv2.resize(np.copy(img2),
                   (grow * img2.shape[1], grow * img2.shape[0])))

        # Kill coronavirus in the room
        # The timer is used to send the goal with regularity

        tim = time.time()
        prev_x = []
        prev_y = []
        counter = 1
        while not rospy.is_shutdown() and minEnergy <= 10:

            # Arbitrarly high value

            minEnergy = 10000
            (i_obj_vec, j_obj_vec) = np.where(img1 > 0)
            energyUV[i_obj_vec, j_obj_vec] = 11
            robot_index_x = math.floor((obc.currentPose[0] - x2)
                    / cellSize)
            robot_index_y = math.floor(matrix_i - 1
                    - math.floor((obc.currentPose[1] - y2) / cellSize))

            # This condition is used in case the approximation of the robot position goes out of the window matrix.
            # and so in this case, to avoid errors, we cause the old position of the robot

            if robot_index_x not in range(0, matrix_j):
                robot_index_x = np.copy(prev_x)
            if robot_index_y not in range(0, matrix_i):
                robot_index_y = np.copy(prev_y)

            # ###############################################################
            # Calculation done with matrix form (without the double for loop). We doesn't use it because is too slow in representation.

            # #################################################################################
            # Double for loop to do calculation for the object detection and update the Energy UV matrix

            for i in range(0, matrix_i):
                for j in range(0, matrix_j):

                    # Calculation of central position of the cell

                    y_cell = (matrix_i - 1 - i) * cellSize + y2 \
                        + cellSize / 2
                    x_cell = j * cellSize + x2 + cellSize / 2

                    # Calculating the absolute distance to the cell

                    dist_x = x_cell - obc.currentPose[0]
                    dist_y = y_cell - obc.currentPose[1]

                    # Calculating the angle between robot and cell position (THETA)

                    if obc.currentPose[0] >= 0:
                        dist_x = abs(dist_x)
                    else:
                        dist_x = -abs(dist_x)

                    if obc.currentPose[1] >= 0:
                        dist_y = abs(dist_y)
                    else:
                        dist_y = -abs(dist_y)

                    angle_rad_cell = math.atan2(dist_y, dist_x)
                    theta = math.degrees(angle_rad_cell)

                    # Orientation f the robot (GAMMA)

                    gamma = obc.currentAngle

                    # Calculating the angle for the laser scanner (counterclockwise)

                    if gamma < 0 and theta > 0:
                        if abs(gamma) < 180 - theta:
                            angle_deg = int(180 + theta + abs(gamma))
                        else:
                            angle_deg = int(abs(gamma) - (180 - theta))

                    if gamma > 0 and theta > 0:
                        if gamma > theta:
                            angle_deg = int(180 - gamma + theta)
                        else:
                            angle_deg = int(180 - gamma + theta)

                    if gamma < 0 and theta < 0:
                        if abs(gamma) > abs(theta):
                            angle_deg = int(180 + abs(gamma)
                                    - abs(theta))
                        else:
                            angle_deg = int(180 - abs(theta)
                                    + abs(gamma))

                    if gamma > 0 and theta < 0:
                        if gamma < 180 - abs(theta):
                            angle_deg = int(180 - (abs(theta) + gamma))
                        else:
                            angle_deg = int(360 - gamma + 180
                                    - abs(theta))

                    # Distance to the nearest obstacle in the direction of the cell

                    dist2obs = obc.laser_scan_values[angle_deg] + 0.5
                    dist2cell = math.sqrt(dist_x ** 2 + dist_y ** 2)

                    # Sum energy if the cell is farther than 0.1

                    if (abs(x_cell - obc.currentPose[0]) > 0.1
                        or abs(y_cell - obc.currentPose[1]) > 0.1) \
                        and energyUV[i, j] <= 10 and dist2obs \
                        > dist2cell:
                        energyUV[i, j] = energyUV[i, j] + Pl \
                            * sampleTime / ((x_cell
                                - obc.currentPose[0]) ** 2 + (y_cell
                                - obc.currentPose[1]) ** 2)

            # Find the minimum local point to go

            prev_x = np.copy(robot_index_x)
            prev_y = np.copy(robot_index_y)
            img3 = np.zeros((matrix_i, matrix_j))

            # Creation of the energy irradiation map

            img3[robot_index_y, robot_index_x] = 255
            img4 = img3 + img1
            energyUV_temp = np.copy(energyUV) + np.copy(img2) \
                + np.copy(img3)
            cv2.imshow('map resized and dilated',
                       cv2.resize(np.copy(img4), (grow * img4.shape[1],
                       grow * img4.shape[0])))
            minEnergy = energyUV_temp.min()
            img3[robot_index_y, robot_index_x] = 0
            (i_min_vector, j_min_vector) = np.where(energyUV_temp
                    == minEnergy)
            window = 5
            index_x = random.randint(0, len(i_min_vector) - 1)
            index_y = index_x
            imin = i_min_vector[index_x]
            jmin = j_min_vector[index_y]

            # This is a submatrix of the room where we check if is present a global min nearly to the robot, for a faster response of the robot

            for pra in i_min_vector:
                for it in j_min_vector:
                    if robot_index_x - window < it < robot_index_x \
                        + window and robot_index_y - window < pra \
                        < robot_index_y + window:
                        imin = pra
                        jmin = it

            rospy.loginfo('min_energy is: ' + str(minEnergy))

            # Putting to zero the energy associated to obstacles before plotting

            energyUV[i_obj_vec, j_obj_vec] = 0

            # Color the image splitting the matrix in 3 to have RGB color matrix from a grey matrix

            imS = cv2.resize(energyUV, (grow * matrix_j, grow
                             * matrix_i))
            grey_img = obc.linear_stretching(np.copy(imS), 10, 0)
            R = np.zeros((grow * matrix_i, grow * matrix_j))
            G = np.zeros((grow * matrix_i, grow * matrix_j))
            B = np.zeros((grow * matrix_i, grow * matrix_j))
            B = np.copy(grey_img)
            B[grey_img < 80] = np.copy(grey_img[grey_img < 80])
            G[grey_img >= 80] = np.copy(grey_img[grey_img >= 80])
            R[grey_img > 50] = np.copy(grey_img[grey_img > 50])
            enel_img[:, :, 0] = R
            enel_img[:, :, 1] = G
            enel_img[:, :, 2] = B

            # UV energy plotting

            image_cm = cv2.applyColorMap(enel_img.astype(np.uint8),
                    cv2.COLORMAP_JET)
            cv2.imshow('Energy UV irradiated', image_cm)
            cv2.waitKey(10)

            # Find the goal position, point having minimum of energy

            x_goal = float(jmin * cellSize + x2)
            y_goal = float((matrix_i - 1 - imin) * cellSize + y2)

            # Publish the goal to reach

            rospy.loginfo(str(x_goal) + str(y_goal))

            # Goal published synchronised with the timer initialized before

            rospy.loginfo(obc.wait)
            if time.time() - tim > 4:
                obc.goalToPublish.pose.pose.position.x = x_goal
                obc.flag2 = True
                obc.flag1 = True
                obc.goalToPublish.pose.pose.position.y = y_goal
                obc.pub.publish(obc.goalToPublish)
                tim = time.time()

            rate.sleep()

    # Set flag to terminate the node

    obc.finishedFlag = 1


if __name__ == '__main__':
    main()
