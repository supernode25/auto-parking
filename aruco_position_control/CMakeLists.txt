cmake_minimum_required(VERSION 3.0.2)
project(aruco_position_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  cv_bridge  
  sensor_msgs 
  pcl_conversions
  tf
  tf2  
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_position_control
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs nav_msgs cv_bridge sensor_msgs tf2_ros
)

## Build an executable
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(aruco_position_control_node src/aruco_position_control_node.cpp)

## Add dependencies
add_dependencies(aruco_position_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries
target_link_libraries(aruco_position_control_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}  
  ${cv_bridge_LIBRARIES} 
)
