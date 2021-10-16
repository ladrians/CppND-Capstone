cmake_minimum_required(VERSION 2.8.3)
project(lsbot_gazebo_plugins)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wno-psabi ${CMAKE_CXX_FLAGS} -fpermissive")

find_package(catkin REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lsbot_msgs REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${lsbot_msgs_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_dev gazebo_ros roscpp tf2 tf2_ros tf2_geometry_msgs trajectory_msgs roscpp sensor_msgs nav_msgs lsbot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lsbot_gazebo_joint_plugin SHARED
  src/lsbot_gazebo_joint_plugin.cpp
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS
  lsbot_gazebo_joint_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
