cmake_minimum_required(VERSION 2.8.3)
project(ur10_robot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  io_lib
  ur_kinematics
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

set(SRC_LIST
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# *************************************************
# *************************************************

add_library(ur10-robot
  src/ur10_robot.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(ur10-robot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(ur10-robot
  ${catkin_LIBRARIES}
  armadillo
  io-lib
  ur10_kin
)


# *************************************************
# *************************************************
