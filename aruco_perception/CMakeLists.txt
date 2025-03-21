cmake_minimum_required(VERSION 3.0.2)
project(aruco_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_parking_control
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_detector src/aruco_node.cpp)

# Link libraries for tf2
target_link_libraries(aruco_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  aruco
  ${TF2_LIBRARIES}
  ${TF2_ROS_LIBRARIES}
)
