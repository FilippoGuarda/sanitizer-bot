#!/usr/bin/env python3
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


class Localization:

    ########## INITIALIZATION ##########

    def __init__(self):

        # Laser
        self.laser_scan_values = []
        self.laser_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.laser_callback)

        # Move base
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

########## LASER SCAN #########

    def laser_callback(self, msg):
        self.laser_scan_values = msg.ranges



########## MAIN ##########


def main():
    loc = Localization()

    # Node initialization
    rospy.init_node('localize_itself', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    g = rospy.ServiceProxy('/global_localization', Empty)
    t = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    g()
    start_time = time.time()
    change_done = False
    tt = 30
    turn_rate = 1

    # navigate map using bug algorithm in spiralling patterns
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
                msg.linear.x = 0.5
                msg.angular.z = 0.2
                loc.vel_pub.publish(msg)
            elif reboot == ("N"):
                t()
                break
            elif reboot == ("C"):
                t()
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                loc.vel_pub.publish(msg)
            else:
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                loc.vel_pub.publish(msg)
                pass

        # Definition of speed of motion
        msg.linear.x = 0.5      
        #msg.angular.z = -turn_rate/5
        loc.vel_pub.publish(msg)
        angle3 = loc.laser_scan_values[0:25]
        angle2 = loc.laser_scan_values[335:360]
        angle4 = loc.laser_scan_values[250:290]
        angle = angle3+angle2

        # Publishing of the velocity
        for i in range(len(angle)):
            if angle[i] <= 0.75:
                msg.linear.x = 0.0
                msg.angular.z = turn_rate
                loc.vel_pub.publish(msg)
                turning = True
            else:
                turning = False
        # keep distance from wall
        for i in range(len(angle4)):
            if angle4[i] >= 1 and not turning:
                msg.linear.x = 0.5
                msg.angular.z = -turn_rate
                loc.vel_pub.publish(msg)
            if angle4[i] <= 1 and angle4[i] >= 0.5 and not turning:
                msg.linear.x = 0.5
                msg.angular.z = 0
                loc.vel_pub.publish(msg)
            if angle4[i] <= 0.5 and not turning:
                msg.linear.x = 0.5
                msg.angular.z = turn_rate
                loc.vel_pub.publish(msg)
                print(f"a bit close {elapsed_time} \n")
        rate.sleep()
    
    rospy.signal_shutdown("localization ended")


if __name__ == '__main__':
    main()
