cmake_minimum_required(VERSION 3.0.2)
project(hector_imu_tools)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs nav_msgs tf)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_and_orientation_to_imu_node src/pose_and_orientation_to_imu_node.cpp)

target_link_libraries(pose_and_orientation_to_imu_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS pose_and_orientation_to_imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
