cmake_minimum_required(VERSION 2.8)
find_package(SWIG REQUIRED)
find_package(Eigen3)

if(SWIG_FOUND)
  include(${SWIG_USE_FILE})
  include_directories(${CMAKE_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})

  # assuming you have installed https://svn.code.sf.net/p/swig/code/branches/gsoc2008-maciekd/
  set(SWIG_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/gsoc2008-maciekd/swig)
  set(ENV{PATH} ${CMAKE_CURRENT_SOURCE_DIR}/gsoc2008-maciekd/:$ENV{PATH})
  set(ENV{SWIG_LIB} ${CMAKE_CURRENT_SOURCE_DIR}/gsoc2008-maciekd/Lib)
  message(WARNING "-- If you have trouble on using swig, try export SWIG_LIB=$ENV{SWIG_LIB}")

  set(INTERFACE_FILES
    hector_mapping.i)

  set_source_files_properties(${INTERFACE_FILES} PROPERTIES
    CPLUSPLUS ON
    COMPILE_FLAGS "-fpermissive" # http://stackoverflow.com/questions/22411514/no-match-for-operator-error-in-c
    )
  set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(CMAKE_CURRENT_BINARY_DIR_TMP ${CMAKE_CURRENT_BINARY_DIR}) ## to output wrap file
  set(CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  swig_add_module(swig_library c ${INTERFACE_FILES})
  set(CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR_TMP})

  swig_link_libraries(swig_library)
  set_target_properties(swig_library PROPERTIES OUTPUT_NAME hector_mapping)
  add_dependencies(swig_library
    ${CMAKE_CURRENT_BINARY_DIR}/hector_mappingC_wrap.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/HectorSlam_proxy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/HectorSlam_proxy.h
    )
else()
  include_directories(${CMAKE_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR})
  add_library(swig_library hector_mappingC_wrap.cxx HectorSlam_proxy.c)
  set_target_properties(swig_library PROPERTIES OUTPUT_NAME hector_mapping)
endif()

# example code
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
add_executable(standalone standalone.cpp HectorSlam_proxy.c)
target_link_libraries(standalone libhector_mapping.so ${catkin_LIBRARIES})
add_dependencies(standalone swig_library)
  
