cmake_minimum_required(VERSION 2.8.3)
project(scan_matching)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  visualization_msgs
  ukf_state_msg
  geodesy

  pcl_ros
  pcl_conversions
  PCL REQUIRED
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime ukf_state_msg geodesy
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_subdirectory(tools)
add_subdirectory(src)

