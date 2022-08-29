#!/usr/bin/env python
# -*- coding: utf-8 -*-

######################################################################
#
#  receives topic from ros_caffé package,
#  search hand forms (stone, paper, scisors)
#  helped with one homemade caffe deep-learner model
#  plays chifumi with user, helped with a bit of random process
#  see readme.md for dependencies
#
#                   Vincent FOUCAULT 18 Mai 2016
######################################################################


import os
import random
import subprocess
import time
import rospy
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String


class Chifumi():
    def __init__(self):
        self.node_name ="alfred_chifumi"
        rospy.init_node(self.node_name)
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        rospy.loginfo('tag messages ok')
        # Subscribe to the ar_pose_marker topic to get the image width and height
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_roi)




    def marker_roi(self, msg):
        try:
            for tag in msg.markers:
                print tag.id
        except: pass


if __name__ == '__main__':
    try:
        a = Chifumi()
        rospy.spin()
        # surtout pas de rospy.spin(), sinon augmentation des boucles et surcharges...
    except rospy.ROSInterruptException: pass











