# CppND-Capstone

This repository is associated to the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213) Capstone project.

I have implemented a Slanted Lidar on top of a movable tower, based on ROS2 (which uses the C++ 14 standard).

![Sample execution](./data/slantedLidar01.gif)

Based on the desired velocity a Gazebo plugin will change dynamically the position of a servo to accomodate to different angle locations.

## Background

The `ls_bot` is an extension from the [rocker-bogie project](https://github.com/SyrianSpock/rover). Several changes were needed to make to compile using the ROS2 foxy default infrastructure. The autonomous rover includes two Lidar sensors, one at the botton and another movable (on top of a servo) on the tower.

![ls_bot in Rviz](./data/ls_bot_rviz01.png)

The objective is to deploy a `Gazebo plugin` to automatically change its `pitch` orientation based on the desired velocity.

## Results

The execution results can be checked in [this video](./data/slantedLidar01.mp4).

### Rubrik considerations

[Rubric](https://review.udacity.com/#!/rubrics/2533/view) brief review.

#### Compiling and Testing

##### Requisites

Make sure to install [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html) or higher, tested used an `Ubuntu 20.04` box.

##### Repository Setup

Create a Workspace, clone the repository and compile it.

```sh
# Create the workspace
mkdir ~/ws
mkdir ~/ws/src
cd ~/ws/src
# Clone the repository
git clone https://github.com/ladrians/CppND-Capstone
cd ~/ws
# Compile
colcon build --symlink-install
```

##### Execution

Before running the project make sure to source it:

```sh
# Source base installation
source /opt/ros/foxy/setup.bash
# Source working workspace
cd ~/ws/
source install/setup.bash
```

Run the sample with the following command:

```sh
ros2 launch lsbot_gazebo world.py
```

The expected behavior is to:

 * Spawn Gazebo with the [corridor_with_obstacles.world](src/lsbot/lsbot_gazebo/worlds/corridor_with_obstacles.world) predefined world.
 * Spawn [Rviz2](https://github.com/ros2/rviz) visualizer.
 * Spawn [rqt](https://docs.ros.org/en/foxy/Concepts/About-RQt.html), select the [rqt_robot_steering](http://wiki.ros.org/rqt_robot_steering) plugin to change rover velocity.

Changing the `linear velocity` of the rover will cause to change the position of the slanted Lidar.

From the previous images obtained in `Rviz2`, the `red` color details the low level Lidar at the botton of the rover, while the `green` line details the slanted one.

The following user input parameters can be added to the launcher using the `key:=value` format:

```sh
# Extra parameters
rviz:=true|false
gui:=true|false
rqt:=true|false
verbose:=true|false
# Minimal Sample changing parameters
ros2 launch lsbot_gazebo world.py rviz:=true gui:=false verbose:=true
```

#### Loops, Functions, I/O

Usage of several C++ functions and control structures can be checked on the [lsbot_gazebo_joint_plugin.cpp](src/lsbot/lsbot_gazebo_plugins/src/lsbot_gazebo_joint_plugin.cpp) source file.

#### Object Oriented Programming

Everything is organized into classes with class attributes to hold the data, and class methods to perform tasks.

All class data members are explicitly specified as public, protected, or private as detailed on the [lsbot_gazebo_joint_plugin.cpp](src/lsbot/lsbot_gazebo_plugins/include/lsbot_gazebo_plugins/lsbot_gazebo_joint_plugin.hpp) header file; all class member functions document their effects using `///` comments.

State is accessed via member functions.

Composition is used instead of inheritance. The `LsbotGazeboPluginRos` implements the Gazebo plugin which composes with the `LsbotGazeboPluginRosPrivate` class.

#### Memory Management

The project makes use of references in function declarations.

The project uses destructors appropriately.

The project uses smart pointers, in general a `shared_ptr` and other abstractions provided by the ROS2 frameword instead of raw pointers.

#### Concurrency

The project uses multiple threads in the execution by using timers and subscribing to different topics via callbacks. It can be validated by the subscription for `odom_shaft` and `cmd_vel` topics besides the `OnUpdate` where angle analysis is triggered.

Several mutexes are used to lock access to common resources.

```
/// Protect variables accessed on callbacks.
std::mutex vel_mutex_;
std::mutex odom_mutex_;
```

### Troubleshooting

In order to execute the project will need to install the pre-requisites, dependencies and compile this project.

## Discussion

The project implements the following packages

 * `lsbot_bringup` initial bootstrap scripts and configurations for the lsbot stack.
 * `lsbot_description` Sandbox Rover description package.
 * `lsbot_gazebo` Simulation environment for Gazebo.
 * `lsbot_gazebo_plugins` Core implementation for this project, using the Gazebo extensibility points.
 * `lsbot_msgs` Message and service data structures.

## Conclusion / Future Work

The project implements a basic use case for a movable slanted 2D lidar to help detecting front obstacles on top of a rover. 3D Lidars are still expensive, so a low cost solution using a 2D lidar + Servo (movability) could fill this gap.

The project involved developing the following stages

 * Create a Rover with the desired sensor data.
 * Create a World to explore the environment
 * Create a movable frame based on external feedback, add some heuristics to define its behavior.

The heuristics defined are encapsulated on the `checkFloorScanWithVelocity` where the position of the Lidar will be changed depending on the following ranges:

|Velocity Range (m/s)|Lidar Position (degrees)|
|--|--|
|-inf to 0.4|60|
|0.4 to 0.7|45|
|0.7 to +inf|30|

Parameters can be changed accordingly using:

 * Velocity
 ** floorscan_velocity_low_range_
 ** floorscan_velocity_middle_range_
 * Position
 ** floorscan_angle_low_
 ** floorscan_angle_middle_
 ** floorscan_angle_high_

The project could be extended to:

 * use more angle positions based on the desired one or mix it with the obtained velocity.
 * change the behavior not to take into account its velocity and move the slanted lidar on a regular basis, defining an artificial field of view, create a `PointCloud` representation.

## Resources

 * [Gazebo Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)
 * [Robotiq modular gripper](https://github.com/YueErro/robotiq_modular_gripper.git)
 * [MARA cobot](https://github.com/AcutronicRobotics/MARA)
