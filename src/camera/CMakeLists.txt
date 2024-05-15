cmake_minimum_required(VERSION 3.0.2)
project(camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set(OpenCV_DIR /home/jetson/opencv_build/opencv/installation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES binocular_module
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
