cmake_minimum_required(VERSION 2.8.3)
project(QBO_Neo_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base
  message_generation
)
generate_dynamic_reconfigure_options(cfg/CalibrateAngular.cfg cfg/CalibrateLinear.cfg)
catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

