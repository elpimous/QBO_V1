#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import rospy
from geometry_msgs.msg import Twist

class LeaveDock():
    def __init__(self):
        rospy.init_node('LeavingDock', anonymous=True)
        self.rate = rospy.get_param("~rate", 10)
        self.r = rospy.Rate(self.rate)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.move_cmd = Twist()
        self.dist1 = (0.35)
        self.dist2 = (0.65)
        self.speed1 = 0.03
        self.speed2 = 0.15
        self.Move1 = int(self.dist1/self.speed1)
        self.Move2 = int(self.dist2/self.speed2)
        self.Duration1 = int(self.Move1*self.rate)
        self.Duration2 = int(self.Move2*self.rate)
        self.move_cmd.angular.z = 0
        self.move_cmd.linear.x = self.speed1
        for t in range(self.Duration1) :
          self.cmd_vel_pub.publish(self.move_cmd)
          self.r.sleep()
        self.move_cmd.linear.x = self.speed2
        for t in range(self.Duration2) :
          self.cmd_vel_pub.publish(self.move_cmd)
          self.r.sleep()
        sys.exit()

if __name__ == '__main__':
    try:
        LeaveDock()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("I left Dock !.")
