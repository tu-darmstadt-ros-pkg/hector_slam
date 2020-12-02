cmake_minimum_required(VERSION 3.0.2)
project(hector_geotiff_launch)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
