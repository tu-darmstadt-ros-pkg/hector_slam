cmake_minimum_required(VERSION 3.0.2)
project(hector_geotiff)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS hector_map_tools hector_nav_msgs nav_msgs pluginlib roscpp std_msgs)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geotiff_writer
  CATKIN_DEPENDS hector_map_tools hector_nav_msgs nav_msgs pluginlib roscpp std_msgs
  DEPENDS EIGEN3 Qt5Widgets
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(geotiff_writer src/geotiff_writer/geotiff_writer.cpp)
target_link_libraries(geotiff_writer ${catkin_LIBRARIES} Qt5::Widgets stdc++fs)
add_dependencies(geotiff_writer ${catkin_EXPORTED_TARGETS})

add_executable(geotiff_saver src/geotiff_saver.cpp)
target_link_libraries(geotiff_saver geotiff_writer)
add_dependencies(geotiff_saver ${catkin_EXPORTED_TARGETS})

add_executable(geotiff_node src/geotiff_node.cpp)
target_link_libraries(geotiff_node geotiff_writer)
add_dependencies(geotiff_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS geotiff_writer geotiff_saver geotiff_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY fonts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

