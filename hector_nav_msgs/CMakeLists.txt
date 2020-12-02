cmake_minimum_required(VERSION 3.0.2)
project(hector_nav_msgs)

find_package(catkin REQUIRED COMPONENTS nav_msgs geometry_msgs message_generation std_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetDistanceToObstacle.srv
  GetRecoveryInfo.srv
  GetRobotTrajectory.srv
  GetSearchPosition.srv
  GetNormal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs nav_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs geometry_msgs message_runtime std_msgs
)
