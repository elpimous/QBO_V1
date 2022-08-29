#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
*************************************************************************
    A RETURN TO DOCK STATION WITH HELP OF AR-TAGS

    BIG THANKS TO PATRICK GOEBEL FOR TUTOS AND PY PROGRAMS
    SEE HIM ON : Pi Robot Project: http://www.pirobot.org

                         Vincent FOUCAULT       January 2016
*************************************************************************
'''

import os, sys
import rospy, subprocess
import smach
import smach_ros
import math

from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist, Point, PoseStamped


class Searching(smach.State):

    def __init__(self):

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # Intialize the movement command
        self.move_cmd = Twist()

        self.rate = rospy.get_param("~rate", 10)
        self.r = rospy.Rate(self.rate)


    def Execute(self):
            print " turning 180°"
            self.move_cmd.angular.z = 0.485 # 0.485 # Normally it should be same value than "speed" (Angular : 0.45 m/s), but minor mod is necessary to correct robot moves precision !
            self.move_cmd.linear.x = 0
            goal_angle = math.pi
            speed = goal_angle/0.45
            rDuration = int(speed*self.rate)
            for t in range(rDuration) :
              self.cmd_vel_pub.publish(self.move_cmd)
              self.r.sleep()
            rospy.sleep(1)
            return 'next_step'




if __name__ == '__main__':
    try:
        rospy.init_node("ar_RTH")
        a = Searching()
        a.Execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("RTH node terminated.")
