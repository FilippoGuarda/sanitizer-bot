#!/usr/bin/env python3


from turtle import speed
import rospy
import numpy as np
import actionlib
import time
import math
from std_msgs.msg import Int64
from std_srvs.srv import SetBool, Empty
import sys
from std_msgs.msg import Float64MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosservice import *
from localization.msg import local as local_msg


class Localization:

    ########## INITIALIZATION ##########

    def __init__(self):

        # Laser
        self.laser_scan_values = []
        self.laser_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.laser_callback)

        # Move base
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Localization ended ack
        self.localization_pub = rospy.Publisher("/localization", local_msg, queue_size=1)

########## LASER SCAN #########

    def laser_callback(self, msg):
        self.laser_scan_values = msg.ranges


    



########## MAIN ##########


def main():
    loc = Localization()

    

    # Node initialization
    rospy.init_node('localize_itself', anonymous=True)
    is_loc_done = local_msg()
    is_loc_done.loc_done = False
    rospy.loginfo(is_loc_done)
    loc.localization_pub.publish(is_loc_done)

    rate = rospy.Rate(10)  # 10hz

    g = rospy.ServiceProxy('/global_localization', Empty)
    t = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    
    start_time = time.time()
    change_done = False
    tt = 30
    turn_rate = 1
    speed_forward = 0.5

    # navigate map using bug algorithm without a target,
    # this makes the turtlebot follow the perimeter of the building
    # even whithout knowing its actual position

    while not rospy.is_shutdown():
        msg = Twist()
        elapsed_time = time.time()-start_time
        # Ask to terminal to restart montecarlo or start after
        if (elapsed_time) > tt:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            loc.vel_pub.publish(msg)
            reboot = input("Reboot Montecarlo? Y or N or Keep going.\nPress C to clear costmap.\n ") 
            start_time = time.time()
            if reboot == ("Y"):
                g()
                t()
                # msg.angular.z = 0
                # msg.linear.x = speed_forward
                loc.vel_pub.publish(msg)
            elif reboot == ("N"):
                t()
                break
            elif reboot == ("C"):
                t()
                # msg.linear.x = speed_forward
                # msg.angular.z = 0.0
                loc.vel_pub.publish(msg)
            else:
                # msg.linear.x = speed_forward
                # msg.angular.z = 0.0
                loc.vel_pub.publish(msg)
                pass

        # Definition of speed of motion
        msg.linear.x = speed_forward      
        msg.angular.z = 0
        loc.vel_pub.publish(msg)
        #front left
        angle3 = loc.laser_scan_values[0:20]
        #front righ
        angle2 = loc.laser_scan_values[340:360]
        angle = angle3+angle2
        rsAngle = loc.laser_scan_values[20:40]
        lsAngle = loc.laser_scan_values[320:340]
        #right side
        angle4 = loc.laser_scan_values[270:290]
        angle5 = loc.laser_scan_values[250:270]
        turning = False
        # Publishing of the velocity
        # turn away from a wall (left) in front of robot
        for i in range(len(angle)):
            if angle[i] <= 0.5:
                msg.linear.x = 0.0
                msg.angular.z = turn_rate
                loc.vel_pub.publish(msg)
                turning = True
                break
            else:
                turning = False
        if not turning:
            #turn a bit left when approaching wall from right
            for i in range(len(rsAngle)):
                if rsAngle[i] <= 0.5:
                    msg.linear.x = speed_forward
                    msg.angular.z = -turn_rate/2
                    loc.vel_pub.publish(msg)
                    turning = True
                    break
        if not turning:
            #turn a bit right when approaching wall from left
            for i in range(len(lsAngle)):
                if lsAngle[i] <= 0.5:
                    msg.linear.x = speed_forward
                    msg.angular.z = turn_rate/2
                    loc.vel_pub.publish(msg)
                    turning = True
                    break
        
        if not turning:
            maxrAngle = 0
            maxlAngle = 0
            for i in range(len(angle4)):
                if angle4[i] > maxlAngle:
                    maxlAngle = angle4[i]
            for i in range(len(angle5)):
                if angle5[i] > maxrAngle:
                    maxrAngle = angle5[i]
            if maxrAngle > 3 and maxlAngle < 3:
                msg.angular.z = -turn_rate
                msg.linear.x = 0
            elif maxrAngle > maxlAngle:
                msg.angular.z = turn_rate/2
                msg.linear.x = speed_forward
            elif maxrAngle < maxlAngle:
                msg.angular.z = -turn_rate/2
                msg.linear.x = speed_forward
            else:
                msg.angular.z = 0
                msg.linear.x = speed_forward
            loc.vel_pub.publish(msg)

        """ # turning based on lateral distance from walls
        if not turning:
            for i in range(len(angle4)):
                if angle4[i] > 1:
                    msg.linear.x = speed_forward
                    msg.angular.z = 0
                    loc.vel_pub.publish(msg)
                # a bit right to keep close to wall 
                if angle4[i] <= 1.5 and angle4[i] > 1:
                    msg.linear.x = speed_forward
                    msg.angular.z = -turn_rate/4
                    loc.vel_pub.publish(msg)
                if angle4[i] <= 1 and angle4[i] > 0.5:
                    msg.linear.x = speed_forward
                    msg.angular.z = 0
                    loc.vel_pub.publish(msg)
                # a bit left to not crash
                if angle4[i] <= 0.5:
                    msg.linear.x = speed_forward
                    msg.angular.z = turn_rate/4
                    loc.vel_pub.publish(msg)
                    print(f"a bit close {elapsed_time} \n")     """
        rate.sleep()
    
    is_loc_done.loc_done = True
    rospy.loginfo(is_loc_done)
    loc.localization_pub.publish(is_loc_done)
    rospy.signal_shutdown("localization ended")


if __name__ == '__main__':
    main()
