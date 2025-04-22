cmake_minimum_required(VERSION 3.0.2)
project(depth_obstacle_detect_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  camera_info_manager
  depth_obstacle_detect_ros_msgs
)

FIND_PACKAGE(Boost)

find_package(OpenCV)
if(OPENCV_FOUND OR
   OPENCV_CORE_FOUND) # we check for this because the OpenCVConfig.cmake(v2.4.9.1) used on dragonboards fails to set OPENCV_FOUND
  if(OpenCV_VERSION VERSION_LESS "3.0.0")
    message(STATUS "USING OPENCV Version 2")
    add_definitions(-DOPENCV2)
  endif(OpenCV_VERSION VERSION_LESS "3.0.0")
else()
  if (NOT OpenCV_INCLUDE_DIR OR NOT OpenCV_LIBS)
    message(FATAL_ERROR "find_package(openCV) failed. OpenCV_INCLUDE_DIR and OpenCV_LIBS need to be set.")
  else()
    message(STATUS "OpenCV_INCLUDE_DIR and OpenCV_LIBS have been set manually.")
  endif()
endif()## System dependencies are found with CMake's conventions

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_depth_obstacle_detect
#  CATKIN_DEPENDS roscpp rospy std_msgs obstacle_msg
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} )


## Declare a C++ library
 add_library(${PROJECT_NAME}_nodelet
   src/depth_obstacle_detect_ros_nodelet.cpp
 )

 add_library(depth_obstacle_detect src/depth_obstacle_detect_ros.cpp)

add_executable(${PROJECT_NAME}_node src/depth_obstacle_detect_ros_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node depth_obstacle_detect ${catkin_LIBRARIES} ${OpenCV_LIBS} )
 target_link_libraries(${PROJECT_NAME}_nodelet depth_obstacle_detect ${catkin_LIBRARIES} ${OpenCV_LIBS} )

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}_nodelet depth_obstacle_detect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

