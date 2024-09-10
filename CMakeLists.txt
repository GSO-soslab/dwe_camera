cmake_minimum_required(VERSION 3.0.2)
project(dwe_camera)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  std_msgs
  sensor_msgs
  message_generation
)

# add_service_files(
#    FILES
# )

generate_messages(
  DEPENDENCIES
  # std_msgs
)

catkin_package(
  CATKIN_DEPENDS
#  INCLUDE_DIRS include
#  LIBRARIES pos_nav
#  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)