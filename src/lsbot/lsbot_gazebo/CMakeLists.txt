cmake_minimum_required(VERSION 2.8.3)
project(lsbot_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wno-psabi ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp gazebo_dev gazebo_ros
)

install(DIRECTORY worlds launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
