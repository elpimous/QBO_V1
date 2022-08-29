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
from random import sample, choice
from math import pow, sqrt

def run_process(command = ""):
    if command != "":
        return subprocess.Popen(command.split())
    else:
        return -1

def run_all_process(all_commands):
    proc=[]
    for command in all_commands:
        proc.append(subprocess.Popen(command.split()))
    return proc

def kill_all_process(processes):
    for process in processes:
        process.send_signal(signal.SIGINT)

def speak_this(text): # for qbo_talk
    global client_speak
    client_speak(str(text))


class NavAmcl():
    def __init__(self):
        global client_speak
        client_speak = rospy.ServiceProxy("/say_fr1", Text2Speach)
        rospy.init_node('nav_amcl', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 1)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        locations = dict()
        
        locations['Face au marqueur gauche'] = Pose(Point(2.863, 1.519, 0.000), Quaternion(0.000, 0.000, 0.927, 0.375))
        locations['Face à la télé'] = Pose(Point(1.591, 0.056, 0.000), Quaternion(0.000, 0.000, 0.928, 0.374))
        locations['Face au canapé'] = Pose(Point(1.934, 0.252, 0.000), Quaternion(0.000, 0.000, -0.419, 0.908))
        locations['Salle à manger, face aux fenêtres'] = Pose(Point(-1.593, -2.436, 0.000), Quaternion(0.000, 0.000, 0.941, 0.337))
        #locations['Salle à manger, côté chambre'] = Pose(Point(0.940, -2.603, 0.000), Quaternion(0.000, 0.000, -0.650, 0.760))
        locations["Face à la porte d'entrée"] = Pose(Point(4.137, -0.384, 0.000), Quaternion(0.000, 0.000, 0.157, 0.988))
        locations['Face à la porte de la cuisine'] = Pose(Point(5.243, -0.840, 0.000), Quaternion(0.000, 0.000, -0.282, 0.960))
        #locations["Au fond du couloir, face à la porte d'entrée"] = Pose(Point(6.626, 1.651, 0.000), Quaternion(0.000, 0.000, 0.984, 0.180))

        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        """
        self.launchers=["roslaunch qbo_bringup minimal.launch"]
        pids=run_all_process(self.launchers)
        rospy.sleep(3) # 3
        
        self.launchers=["roslaunch qbo_navigation qbo_amcl.launch"]
        pids=run_all_process(self.launchers)
        speak_this("je démarre la carte et Aaimsséaile")
        
        rospy.sleep(2)  # 5
        
        self.launchers=["rosrun map_server map_server /home/neo/catkin_ws/src/qbo_navigation/maps/boncourt_rdc001.yaml"]
        #speak_this("Je vais tourner sur moi-maime pour me localiser dans zune piaice")
        #self.rotation()
        
        #self.launchers=["roslaunch qbo_navigation move_base_qbo.launch"]
        #pids=run_all_process(self.launchers)
        
        #speak_this("daimarage du mouve_base")
        """


        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("En attente du serveur move_base...")
        
        #initial_pose = PoseWithCovarianceStamped()
        #speak_this("Je quitte ma station de charge")
        #rospy.sleep(2)  # 
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(20)) # 60
        
        rospy.loginfo("Robot connecté au serveur move_base.")
        
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""

        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)


        """          
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_initial_pose)
        """  

      
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
        
        rospy.loginfo("Début du test d'autonomie...")
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                sequence = sample(locations, n_locations)
                # Skip over first location if it is the same as
                # the last location
                if sequence[0] == last_location:
                    i = 1
            
            # Get the next location in the current sequence
            location = sequence[i]
                        
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("*** mise à jour de l'initial_pose !!! ***")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
            # Increment the counters
            i += 1
            n_goals += 1
        
            # Set up the next goal location
            rospy.sleep(1)
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            #speak_this("Je me dirige vers " + str(location))
            #rospy.loginfo("En direction de l'emplacement : " + str(location))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 3 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(120)) # 180
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                speak_this("Temps limite dépassé !")
                rospy.loginfo("Temps limite dépassé !")
            else:
                state = self.move_base.get_state()
        self.cmd_vel_pub.publish(rotdroite)


    def recul(self): # boucle de recul sur 2 secondes
      start_time = rospy.Time.now()
      time_rot_AMCL = 0
      recule = 3
      while not time_rot_AMCL > recule :
        time_rot_AMCL = rospy.Time.now() - start_time
        time_rot_AMCL = time_rot_AMCL.secs                 
        Recule = Twist()
        Recule.linear.x=-0.05
        Recule.angular.z= choice([-0.4,0.4])
        self.cmd_vel_pub.publish(Recule)
     
    def update_initial_pose(self, initial_pose):
        if state == GoalStatus.SUCCEEDED:
          rospy.loginfo("        !!! Arrivé a destination !!!")
          #speak_this(str(location) + " rejoint !")
          n_successes += 1
          distance_traveled += distance
        else:
            #speak_this("Oula, " + str(goal_states[state]))
            rospy.loginfo("Echec: " + str(goal_states[state]))
            self.recul()
            time_rot = 20
        while not time_rot_AMCL > time_rot :
            time_rot_AMCL = rospy.Time.n
            #pids=run_all_process(self.launchers)

            
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("*****************************************************")
            rospy.loginfo("*  TAUX DE REUSSITE: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("*  TEMPS DE FONCTIONNEMENT total: " + str(trunc(running_time, 1)) + " min")
            rospy.loginfo("*  DISTANCE PARCOURUE: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.loginfo("*****************************************************")
            rospy.sleep(self.rest_time)


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
