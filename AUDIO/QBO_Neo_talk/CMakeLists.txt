cmake_minimum_required(VERSION 2.8.3)
project(QBO_Neo_talk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  message_generation
)

## Generate services in the 'srv' folder
 add_service_files(
   DIRECTORY srv
   FILES Text2Speach.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_srvs
#   std_msgs  # Or other packages containing msgs
) 	 		

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qbo_talk
  CATKIN_DEPENDS rospy std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

