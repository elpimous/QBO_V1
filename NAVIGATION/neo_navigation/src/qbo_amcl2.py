#!/usr/bin/env python
# -*- coding: utf-8 -*-


import roslib
import rospy
import actionlib
import subprocess
import random
import os
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from qbo_talk.srv import Text2Speach
#from random import sample
from math import pow, sqrt


def run_process(command = ""):
    if command != "":
        return subprocess.Popen(command.split())
    else:
        return -1


class NavAmcl():
    def __init__(self):
        rospy.init_node('nav_amcl', anonymous=True)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 1)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        """
        self.locations = dict()
        
        self.locations['s.a.m fenetres'] = Pose(Point(0.186, -1.506, 0.000), Quaternion(0.000, 0.000, 0.997, -0.072))
        self.locations['couloir_entrée'] = Pose(Point(2.568, 1.554, 0.000), Quaternion(0.000, 0.000, 0.914, 0.405))
        self.locations['couloir_cuisine'] = Pose(Point(2.710, 2.746, 0.000), Quaternion(0.000, 0.000, 0.607, 0.795))
        self.locations['salon_tv'] = Pose(Point(3.120, -0.892, 0.000), Quaternion(0.000, 0.000, -0.815, 0.580))
        self.locations['salon_canapé_face'] = Pose(Point(2.806, -0.877, 0.000), Quaternion(0.000, 0.000, 0.514, 0.858))
        self.locations['salon_canapé_côté'] = Pose(Point(2.145, 0.405, 0.000), Quaternion(0.000, 0.000, 0.086, 0.996))
        self.locations['face_tag_gauche'] = Pose(Point(1.205, -0.133, 0.000), Quaternion(0.000, 0.000, -0.778, 0.628))
        """
        
        self.waypoints = [
		[(5.196, -1.543, 0.000),(0.000, 0.000, -0.505, 0.863)],
		[(6.088, -0.356, 0.000),(0.000, 0.000, 0.238, 0.971)],
		[(2.568, 1.554, 0.000),(0.000, 0.000, 0.914, 0.405)],
		[(2.710, 2.746, 0.000),(0.000, 0.000, 0.607, 0.795)],
		[(3.120, -0.892, 0.000),(0.000, 0.000, -0.815, 0.580)],
		[(2.806, -0.877, 0.000),(0.000, 0.000, 0.514, 0.858)],
		[(2.145, 0.405, 0.000),(0.000, 0.000, 0.086, 0.996)],
		[(1.205, -0.133, 0.000),(0.000, 0.000, -0.778, 0.628)]
			 ] # 8 waypoints

        #run_process("roslaunch qbo_navigation QBO_Neo_amcl.launch") # launch robot params(bringup_minimal), openni2, the map, amcl and move_base

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("En attente du serveur move_base...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(20)) # 60
        
        rospy.loginfo("Robot connecté au serveur move_base.")

        # Get the initial pose from the user
        rospy.loginfo("*** Starting...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()

        rospy.loginfo("Début du test d'autonomie...")
        
        # Begin the main loop and run through a sequence of locations
        i = 0
        while i < len(self.waypoints):

            # Set up the next goal location
            rospy.sleep(1)

            # Start the robot toward the next location
            for i in range(0,len(self.waypoints)):
                self.targetPose = self.waypoints[i]
	        self.goal_pose = MoveBaseGoal()    
	        self.goal_pose.target_pose.header.frame_id = 'map'    
        	self.goal_pose.target_pose.pose.position.x = self.targetPose[0][0]    
        	self.goal_pose.target_pose.pose.position.y = self.targetPose[0][1]    
        	self.goal_pose.target_pose.pose.position.z = self.targetPose[0][2]    
        	self.goal_pose.target_pose.pose.orientation.x = self.targetPose[1][0]    
	        self.goal_pose.target_pose.pose.orientation.y = self.targetPose[1][1]    
	        self.goal_pose.target_pose.pose.orientation.z = self.targetPose[1][2]    
	        self.goal_pose.target_pose.pose.orientation.w = self.targetPose[1][3]

            	# improving actual_pose
    	    	speed_command=Twist()
	    	speed_command.linear.x=0.0
	    	speed_command.angular.z=0.3
            	t = 0
            	for t in range (0, 400000):
			self.cmd_vel_pub.publish(speed_command)
                	t+=1

	    	speed_command.linear.x=0.0
	    	speed_command.angular.z=-0.3
            	t = 0
            	for t in range (0, 600000):
			self.cmd_vel_pub.publish(speed_command)
                	t+=1

            	# Let the user know where the robot is going next
            	rospy.loginfo("*** En direction de l'emplacement " + str(i) + " ***")

                self.move_base.send_goal(self.goal_pose)

                # Allow 3 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(120)) # 180
            
                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Temps limite dépassé !")
                else:
                    state = self.move_base.get_state()

                i += 1

        #return 'succeded'
        self.shutdown()


    def shutdown(self):
       rospy.loginfo("Arrêt du robot...")
       self.move_base.cancel_goal()
       rospy.sleep(2)
       self.cmd_vel_pub.publish(Twist())
       rospy.sleep(1)

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])       


if __name__ == '__main__':
    try:
        NavAmcl()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Fin du test de navigation AMCL.")
