cmake_minimum_required(VERSION 3.0.2)
project(hector_imu_attitude_to_tf)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_attitude_to_tf_node src/imu_attitude_to_tf_node.cpp)
target_link_libraries(imu_attitude_to_tf_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS imu_attitude_to_tf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
