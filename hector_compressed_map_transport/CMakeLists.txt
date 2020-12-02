cmake_minimum_required(VERSION 3.0.2)
project(hector_compressed_map_transport)

find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs hector_map_tools image_transport nav_msgs sensor_msgs)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_to_image_node src/map_to_image_node.cpp)
target_link_libraries(map_to_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS map_to_image_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
