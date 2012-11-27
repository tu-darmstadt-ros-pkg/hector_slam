cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

SET(SRCS
)

SET(HDRS
)

SET(UIS
)

qt4_wrap_cpp(MOCS ${HDRS})
qt4_wrap_ui(UIHDRS ${UIS})

# find Eigen3
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(geotiff_writer src/geotiff_writer/geotiff_writer.cpp)
target_link_libraries(geotiff_writer ${QT_LIBRARIES})

rosbuild_add_executable(geotiff_saver src/geotiff_saver.cpp)
target_link_libraries(geotiff_saver geotiff_writer ${QT_LIBRARIES} ${GEOTIFF_LIBRARY})

rosbuild_add_executable(geotiff_node src/geotiff_node.cpp)
target_link_libraries(geotiff_node geotiff_writer ${QT_LIBRARIES} ${GEOTIFF_LIBRARY})
