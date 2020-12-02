cmake_minimum_required(VERSION 3.0.2)
project(hector_mapping)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs visualization_msgs tf message_filters laser_geometry tf_conversions message_generation)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HectorDebugInfo.msg
  HectorIterData.msg
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hector_mapping
  CATKIN_DEPENDS roscpp nav_msgs visualization_msgs tf message_filters laser_geometry tf_conversions message_runtime
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(include/hector_slam_lib)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(hector_mapping
  src/HectorDebugInfoProvider.h
  src/HectorDrawings.h
  src/HectorMappingRos.h
  src/HectorMappingRos.cpp
  src/main.cpp
  src/PoseInfoContainer.cpp
  src/PoseInfoContainer.h
)

add_dependencies(hector_mapping hector_mapping_generate_messages_cpp)

target_link_libraries(hector_mapping
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS hector_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Don't ask why it's hector_slam_lib, this was Stefan Kohlbrecher's first ROS package and a wrapper of a pre ROS header only library
install(DIRECTORY include/hector_slam_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
