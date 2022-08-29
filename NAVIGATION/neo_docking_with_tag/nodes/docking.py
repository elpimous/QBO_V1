#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Authors: Leon Jung
# modified for Neo robot (vincent foucault - juin 2019)
# added : back facing, docking, some minor updates

import rospy
import os
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from math import pi as PI
import time

# Marker ID to track
MARKER_ID_DETECTION = rospy.get_param("~marker_id", 30)
# Max linear speed
MAX_LINEAR_SPEED = rospy.get_param("~max_linear_speed", 0.1)
# Max angular speed
MAX_ANGULAR_SPEED = rospy.get_param("~max_angular_speed", 0.2)
# final distance to tag, in cm
FINAL_DISTANCE = rospy.get_param("~final_distance_to_tag", 0.30)

# D435 RGB Y D435_Y_CORRECTION (On this product,the RGB camera is placed totally to the right (faced to camera) Need to center it for better moves)
D435_Y_CORRECTION = 0.025

class AutomaticDocking():
    def __init__(self):
        rospy.loginfo('Waiting for /odom topic...')
        rospy.wait_for_message('/odom', Odometry)
        rospy.loginfo('Done')
        self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.GetRobotOdometry, queue_size = 1)
        self.RStatut = rospy.Subscriber('/qbo_arduqbo/status', Int8, self.GetRobotBatteryStatut, queue_size = 1)
        rospy.loginfo('Waiting for /ar_pose_marker topic...')
        rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
        rospy.loginfo('Done')
        self.sub_info_marker = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.GetMarkerOdom, queue_size = 1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.ParkingSequence = Enum('ParkingSequence', 'searching_parking_lot changing_direction moving_nearby_parking_lot approaching_marker backfacing_tag backward_move recovery_mode stop finished')
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        self.current_parking_sequence = self.ParkingSequence.searching_parking_lot.value

        self.robot_pose_x = .0
        self.robot_pose_y = .0
        self.robot_theta = .0
        self.marker_pose_x = .0
        self.marker_pose_y = .0
        self.marker_theta = .0

        self.previous_robot_theta = .0
        self.total_robot_theta = .0
        self.is_triggered = False

        self.is_sequence_finished = False

        self.is_odom_received = False
        self.is_marker_pose_received = False
                
        loop_rate = rospy.Rate(20) # 10hz
        while not rospy.is_shutdown():
            if self.is_odom_received is True:
                self.DockingProcess()

            loop_rate.sleep()

        rospy.on_shutdown(self.ShutDown)

    def GetRobotOdometry(self, robot_odom_msg):
      try:
        if self.is_odom_received == False:
            self.is_odom_received = True

        pos_x, pos_y, theta = self.GetRobotPose(robot_odom_msg)

        self.robot_pose_x = pos_x
        self.robot_pose_y = pos_y
        self.robot_theta = theta

        if (self.robot_theta - self.previous_robot_theta) > 5.:
            d_theta = (self.robot_theta - self.previous_robot_theta) - 2 * PI
        elif (self.robot_theta - self.previous_robot_theta) < -5.:
            d_theta = (self.robot_theta - self.previous_robot_theta) + 2 * PI
        else:
            d_theta = (self.robot_theta - self.previous_robot_theta)

        self.total_robot_theta = self.total_robot_theta + d_theta
        self.previous_robot_theta = self.robot_theta

        self.robot_theta = self.total_robot_theta
      except : pass

    def GetMarkerOdom(self, markers_odom_msg):
        for marker_odom_msg in markers_odom_msg.markers:
            if marker_odom_msg.id == MARKER_ID_DETECTION:
                if self.is_marker_pose_received == False:
                    self.is_marker_pose_received = True

                pos_x, pos_y, theta = self.GetMarkerPose(marker_odom_msg)

                self.marker_pose_x = pos_x
                self.marker_pose_y = pos_y + D435_Y_CORRECTION
                self.marker_theta = theta - PI

    def GetRobotBatteryStatut(self, battStatut):
        self.battery_stat = str(battStatut.data)

    def DockingProcess(self):
        if self.current_parking_sequence == self.ParkingSequence.searching_parking_lot.value:
            self.is_sequence_finished = self.SearchingMarker()

            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.changing_direction.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.changing_direction.value:
            self.is_sequence_finished = self.ChangingDirection()

            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.moving_nearby_parking_lot.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            self.is_sequence_finished = self.MovingNearbyMarker()

            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.approaching_marker.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.approaching_marker.value:
            self.is_sequence_finished = self.MovingNearTag()

            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.backfacing_tag.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.backfacing_tag.value:
            self.is_sequence_finished = self.BackfacingTag()

            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.backward_move.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.backward_move.value:
            self.is_sequence_finished = self.backward_move()

            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False
                
            if self.is_sequence_finished == False:
                self.current_parking_sequence = self.ParkingSequence.recovery_mode.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.recovery_mode.value:
            self.is_sequence_finished = self.Recovery_mode()

            if self.is_sequence_finished == True:
                self.current_parking_sequence == self.ParkingSequence.searching_parking_lot.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.stop.value:
            self.StopRobot()
            self.current_parking_sequence = self.ParkingSequence.finished.value
            self.ShutDown()

    def SearchingMarker(self): # recherche tag
        if self.is_marker_pose_received is False:
            self.desired_angle_turn = -0.6
            self.Rotate(self.desired_angle_turn)
        else:
            rospy.loginfo('*** Tag seen !')
            self.StopRobot()
            return True

    def ChangingDirection(self): # tag ok, rotation du bon coté
        desired_angle_turn = -1. * (math.atan2(self.marker_pose_y - 0, self.marker_pose_x - 0))

        self.Rotate(desired_angle_turn)

        if abs(desired_angle_turn) < 0.05:
            self.StopRobot()
            return True
        else:
            return False

    def MovingNearbyMarker(self): # avance, pour se retrouver devant tag, puis rotation vers tag
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_theta
                self.initial_robot_pose_x = self.robot_pose_x
                self.initial_robot_pose_y = self.robot_pose_y
                self.initial_marker_pose_theta = self.marker_theta
                self.initial_marker_pose_x = self.marker_pose_x
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (PI / 2.0) + self.initial_marker_pose_theta - (self.robot_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(PI / 2.0) + self.initial_marker_pose_theta - (self.robot_theta - self.initial_robot_pose_theta)

            desired_angle_turn = -1. * desired_angle_turn

            self.Rotate(desired_angle_turn) # robot is turning, to be parallel to the marker
            
            if abs(desired_angle_turn) < 0.2: #0.05
                self.StopRobot()
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
                self.is_triggered = False

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            dist_from_start = self.CalcDistPoints(self.initial_robot_pose_x, self.robot_pose_x, self.initial_robot_pose_y, self.robot_pose_y)

            desired_dist = self.initial_marker_pose_x * abs(math.cos((PI / 2.) - self.initial_marker_pose_theta))
            remained_dist = desired_dist - dist_from_start

            self.GoStraight(remained_dist) # robot moves parallel to tag, to finish centered, uses an acceleration coeff regarding to the distance to travel
            
            if remained_dist < 0.01:
                self.StopRobot()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value

        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = -(PI / 2.0) + (self.robot_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = (PI / 2.0) + (self.robot_theta - self.initial_robot_pose_theta)

            self.Rotate(desired_angle_turn)

            if abs(desired_angle_turn) < 0.1: #0.05
                self.StopRobot()
                
                rospy.loginfo('*** Approaching tag !')
                #TODO recup angle du tag, et si superieur a x degres, recommencer (pour eviter les rotations du mauvais cote)
                #print('angle du tag')
                #print(self.initial_marker_pose_theta)
                
                self.current_nearby_sequence = self.NearbySequence.parking.value
                self.is_triggered = False
                return True

        return False

    def MovingNearTag(self): # avancee parallele au tag, pour se retrouver face au tag
        speed_coeff = self.marker_pose_x # This coeff. allows speed variations relative to distance (far->fast, near->low)
        if not abs(self.marker_pose_y - D435_Y_CORRECTION) <= 0.005: # if out of neutral tag centered position, rotation needed
            desired_angle_turn = math.copysign(0.015, self.marker_pose_y)
        else: # nearly centered, no rotation needed
            desired_angle_turn = 0.0
        self.TrackMarker(speed_coeff, desired_angle_turn)

        if abs(self.marker_pose_x) < FINAL_DISTANCE:
            
            rospy.loginfo('*** BackFacing tag !')
            
            self.StopRobot()
            return True
        else:
            return False

    def BackfacingTag(self): # turn 180° to present back of robot, for insertion on Dock_station
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_robot_pose_theta = self.robot_theta

        if self.initial_robot_pose_theta < 0.0:
            desired_angle_turn = -PI + (self.robot_theta - self.initial_robot_pose_theta)
        elif self.initial_robot_pose_theta > 0.0:
            desired_angle_turn =  PI + (self.robot_theta - self.initial_robot_pose_theta)

        self.HalfTurn(desired_angle_turn)

        if abs(desired_angle_turn) < 0.05: # 0.05
            
            rospy.loginfo('*** Entering Dockstation !')
            self.StopRobot()
            self.current_parking_sequence = self.ParkingSequence.backward_move.value
            self.is_triggered = False
            return True

    def backward_move(self): # move backward until robot detects Dock_station power contacts
        distance = (-0.40)
        self.lD = int(distance/-(MAX_LINEAR_SPEED*1.5))
        self.lDuration = int(self.lD*10)
        self.r = rospy.Rate(10)
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = -(MAX_LINEAR_SPEED*1.5)
        for t in range(self.lDuration) :
            self.pub_cmd_vel.publish(twist)
            self.r.sleep()
            
        rospy.sleep(4) # wait a bit for new battery statut
        charging = self.RobotStatut() # check if robot is charging
        if charging :        
            return True
        else :
            return False

    def Recovery_mode(self): # bad insertion : leaving dock for next try
        rospy.loginfo('recovey mode')
        distance = (1.8) # 1.80m
        self.lD = int(distance/(MAX_LINEAR_SPEED*1.5))
        self.lDuration = int(self.lD*10)
        self.r = rospy.Rate(10)
        twist = Twist()
        twist.angular.z = 0.05
        twist.linear.x = (MAX_LINEAR_SPEED*1.5)
        for t in range(self.lDuration) :
            self.pub_cmd_vel.publish(twist)
            self.r.sleep()
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        self.current_parking_sequence = self.ParkingSequence.searching_parking_lot.value
        self.is_marker_pose_received is False
        return True

            
    def StopRobot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(2)

    def Rotate(self, theta):
        angular_z = MAX_ANGULAR_SPEED * theta  # /1.5
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def GoStraight(self, lin_x_coeff):
        twist = Twist()
        twist.linear.x = (MAX_LINEAR_SPEED*2.5) * (lin_x_coeff*2)
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def TrackMarker(self, lin_x_coeff, theta):
        twist = Twist()
        twist.linear.x = (MAX_LINEAR_SPEED/6) * (lin_x_coeff*3)
        #twist.angular.z = (-MAX_ANGULAR_SPEED/3) * (theta)
        twist.angular.z = theta
        self.pub_cmd_vel.publish(twist)

    def HalfTurn(self, theta):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = (-MAX_ANGULAR_SPEED* theta)
        self.pub_cmd_vel.publish(twist)

    def RobotStatut(self):
        if self.battery_stat == 15:
            print("ca charge")
            #robotStatut = "charging"
            return True
        else:
            #robotStatut = "Docking ERROR"
            return False
        
    def GetRobotPose(self, robot_odom_msg):
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y, robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]

        if theta < 0:
            theta = theta + PI * 2
        if theta > PI * 2:
            theta = theta - PI * 2

        pos_x = robot_odom_msg.pose.pose.position.x
        pos_y = robot_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    def GetMarkerPose(self, marker_odom_msg):
        quaternion = (marker_odom_msg.pose.pose.orientation.x, marker_odom_msg.pose.pose.orientation.y, marker_odom_msg.pose.pose.orientation.z, marker_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]

        theta = theta + PI / 2.
        # rospy.loginfo("theta : %f", theta)

        if theta < 0:
            theta = theta + PI * 2
        if theta > PI * 2:
            theta = theta - PI * 2

        pos_x = marker_odom_msg.pose.pose.position.x
        pos_y = marker_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    def CalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def ShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Docking')
    node = AutomaticDocking()
    node.main()
