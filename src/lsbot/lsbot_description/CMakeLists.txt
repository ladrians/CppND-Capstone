cmake_minimum_required(VERSION 2.8.3)
project(lsbot_description)

find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs xacro urdf tf2_ros tf2_geometry_msgs tf2 std_msgs sensor_msgs
)
