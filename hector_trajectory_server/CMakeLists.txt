cmake_minimum_required(VERSION 3.0.2)
project(hector_trajectory_server)

find_package(catkin REQUIRED COMPONENTS roscpp hector_nav_msgs nav_msgs hector_map_tools tf)

catkin_package(
  CATKIN_DEPENDS roscpp hector_nav_msgs nav_msgs hector_map_tools tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hector_trajectory_server src/hector_trajectory_server.cpp)
add_dependencies(hector_trajectory_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_trajectory_server
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS hector_trajectory_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
