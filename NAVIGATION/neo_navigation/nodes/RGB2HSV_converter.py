#!/usr/bin/env python
# -*- coding: utf-8 -*-

######################################################################
#
#  opencv receives the D435 rectified color video topic stream,
#  image is converted to HSV
#  image is published in a new topic : 
#
#                   Vincent FOUCAULT 31 mai 2019
######################################################################

import rospy
import cv2
import imutils
from threading import Thread
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


video_topic = "/camera/color/image_raw"
pub = rospy.Publisher("hsv"+video_topic, Image, queue_size=10)

class Process():
    def __init__(self):
        self.node_name = "RGB2HSV_color_topic_converter"
        rospy.init_node(self.node_name)
        #rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(video_topic,Image,self.image_callback)
        rospy.wait_for_message(video_topic, Image)

    def image_callback(self, ros_image):
      try:
        self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
      except CvBridgeError as e:
        print(e)

      self.frame = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
      # launch threading process
      self.start()

    def start(self):
	# start the thread to read frames from the video stream
	Thread(target=self.process, args=()).start()
	return self

    def process(self):
		try:
			pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "hsv"))
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
    try:
        a = Process()
        rospy.spin()
    except rospy.ROSInterruptException: pass










