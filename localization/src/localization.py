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
                        
        #Laser
        self.laser_scan_values = []
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        #Move base
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
 
########## LASER SCAN #########

    def laser_callback(self, msg):
    	self.laser_scan_values  = msg.ranges
    
########## MAIN ##########    
    
def main():
    obc = Localization()
    
    # Node initialization
    rospy.init_node('localize_itself', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    g=rospy.ServiceProxy('/global_localization', Empty)
    t=rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    g()
    start_time = time.time()
    tt=25

    while not rospy.is_shutdown():
    	msg = Twist()
    	# Ask to terminal to restart montecarlo or start after
    	if (time.time()-start_time)>tt:
    		msg.linear.x = 0.0
    		msg.angular.z = 0.0
    		obc.vel_pub.publish(msg)
    		reboot=input("Reboot Montecarlo? Y or N or Keep going ")
    		start_time=time.time()
    		if reboot==("Y"):
    			g()
    			msg.linear.x = 0.5
    			msg.angular.z = 0.2
    			obc.vel_pub.publish(msg)
    		elif reboot==("N"):
    			t()
    			break
    		else :
    			msg.linear.x = 0.5
    			msg.angular.z = 0.0
    			obc.vel_pub.publish(msg)
    			pass

    	# Definition of speed of motion
    	msg.linear.x = 0.5
    	msg.angular.z = -0.2
    	obc.vel_pub.publish(msg)
    	angle3 = obc.laser_scan_values[0:25]
    	angle2 = obc.laser_scan_values[335:360]
    	angle = angle3+angle2

        # Publishing of the velocity
    	for i in range(len(angle)):
    		if angle[i] <= 0.5:
    			msg.linear.x = 0.0
    			msg.angular.z = 1
    			obc.vel_pub.publish(msg)
    	rate.sleep()

if __name__ == '__main__':
	main()

