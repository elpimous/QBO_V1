#!/usr/bin/env python3
# -*- coding: utf-8 -*-

######################################################################
#
#  opencv receives the video topic stream,
#
#                   Vincent FOUCAULT 6 APRIL 2016
######################################################################

import rospy
import cv2
import sys
from threading import Thread
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import keras_preprocessing
from keras_preprocessing import image
import numpy as np

# Recreate the exact same model, including its weights and the optimizer
rospy.loginfo("Loading Tensorflow Chifoumi model...")
chifoumi_model = tf.keras.models.load_model('/home/nvidia/catkin_ws/src/QBO_Neo_play_games/src/TF_Chifoumi_model.h5')
rospy.loginfo("Model loaded.")

class colorGame():
    def __init__(self):
        self.node_name = "camera_view"
        rospy.init_node(self.node_name)
        print("node started")
        #rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.image_callback)
        rospy.wait_for_message("/image_raw", Image)
        self.pub = rospy.Publisher('/chifoumi_results', String, queue_size=1)
        print("video stream ok...")

    def image_callback(self, ros_image):
      try:
        self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
      except CvBridgeError as e:
        print(e)
      # show image
      #cv2.imshow(self.node_name, self.cv_image)
      #cv2.waitKey(3)

      # launch threading process
      self.start()

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.process, args=()).start()
        return self

    def process(self):
        self.rate = rospy.get_param("~rate", 3)
        self.r = rospy.Rate(self.rate) 
        """
        img = cv2.resize(self.image_sub, (150, 150))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        images = np.vstack([x])
        while True:
            prediction = chifoumi_model.predict(images, batch_size=3)
            print(prediction)
        """


if __name__ == '__main__':
    try:
        a = colorGame()
        rospy.spin()
    except rospy.ROSInterruptException: pass










