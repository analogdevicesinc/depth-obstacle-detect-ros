cmake_minimum_required(VERSION 3.10)
project(depth_obstacle_detect_ros)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not set. Please source your ROS environment.")
endif()

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(NOT "${ROS_DISTRO}" STREQUAL "jazzy")
  add_definitions(-DROS_HUMBLE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depth_obstacle_detect_ros_msgs REQUIRED)

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

include_directories(include PUBLIC)
add_library(depth_obstacle_detect src/depth_obstacle_detect_ros.cpp)

add_executable(${PROJECT_NAME}_node src/depth_obstacle_detect_ros_node.cpp)
ament_target_dependencies(depth_obstacle_detect rclcpp image_transport cv_bridge sensor_msgs rclpy depth_obstacle_detect_ros_msgs)
target_link_libraries(${PROJECT_NAME}_node depth_obstacle_detect ${OpenCV_LIBS} )

add_library(${PROJECT_NAME}_component SHARED src/depth_obstacle_detect_ros_component.cpp)
ament_target_dependencies(${PROJECT_NAME}_component rclcpp_components)
target_link_libraries(${PROJECT_NAME}_component depth_obstacle_detect ${OpenCV_LIBS})

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "DepthObstacleDetect"
  EXECUTABLE obstacle_detect_component
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_obstacle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_dependencies(depth_obstacle_detect)
ament_package()
