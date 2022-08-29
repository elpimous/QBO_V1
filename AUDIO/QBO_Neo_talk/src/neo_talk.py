#!/usr/bin/env python
# coding: utf-8

#*****************************************************************************************************
##
# This is the version 3.0 
#
# Use of Cereproce cerevoice  french male voice (the perfect one for Alfrd, LOL)
#
# Enjoy, friends.
#
# Author: Vincent FOUCAULT                       Oct 9 2018
#
#*****************************************************************************************************


import rospy
import roslib
import os
import time
import numpy as np

# read service content (words we want to be spoken)
from QBO_Neo_talk.srv import Text2Speach 

from time import sleep

# Cerevoice directory inside talk package
pkg_dir = roslib.packages.get_pkg_dir("QBO_Neo_talk")
cerevoice_dir = pkg_dir+"/params/cerevoice"
# parts of routine...
Cbinary = cerevoice_dir+"/txt2wav" # the binary

# The french male voice without accent
Cvoice = cerevoice_dir+"/cerevoice_laurent_4.0.0_48k.voice"
# The personal license : Do not share !!
Clicense = cerevoice_dir+"/CereVoiceLaurent.lic"

# swap text/audio files
Cxml = cerevoice_dir+"/temp.xml"
Cwav = cerevoice_dir+"/temp.wav"

# audio command
send_voice = "aplay "+Cwav


#--- Routine for cerevoice binary
Ccommand = Cbinary+" "+Cvoice+" "+Clicense+" "+Cxml+" "+Cwav

class talk():

    def __init__(self):

        rospy.init_node('talk', anonymous=True)
        fr1 = rospy.Service('say_fr1', Text2Speach, self.fr1_talk)
        #en1 = rospy.Service('say_en1', Text2Speach, self.en1_talk)
        
    def fr1_talk(self, data): 

        self.speak(data.sentence)
        return True



    def speak(self, datas): 
      	  self.Cut_mics()
          #sleep(0.1)
          with open(Cxml, 'w') as xml:
              xml.write(datas)
              xml.close()
          os.system(Ccommand) # create .wav voice file
          os.system(send_voice) # play .wav at 'same'time
          #sleep(0.1)
          self.Open_mics()
          return True

    def Cut_mics(self):
        # When speaking, micros cut (0 %)  /OFF
        os.system('amixer -D pulse set "Capture" nocap')
	pass

    def Open_mics(self):
        # When finished speaking, micros hear again (100 %)  /ON
        os.system('amixer -D pulse set "Capture" cap')
	pass
   

if __name__ == '__main__':
    try:
        talk = talk()
        rospy.spin()
    except rospy.ROSInterruptException: pass


