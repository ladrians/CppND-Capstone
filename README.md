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

Make sure to install [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html) or higher, it needs a `Ubuntu Linux - Focal Fossa (20.04)` version; tested with a `Ubuntu 20.04` virtual machine.

The default `Udacity workspace` is not valid because it uses a `16.04` version.

```sh
lsb_release -a
No LSB modules are available.
Distributor ID:     Ubuntu
Description:        Ubuntu 16.04.6 LTS
Release:  16.04
Codename: xenial
```

Use `docker` if you have another environment set. Follow these sample steps, tested with a `VMware Box` using `Ubuntu 18.04`.

```sh
# Install Docker first from https://docs.docker.com/engine/install/ubuntu/
# Clone the repository
git clone https://github.com/ladrians/CppND-Capstone ~/ws
cd ~/ws
# Build the Image and compile the project
sudo ./build.sh
```

For alternative docker command options check the [build](https://docs.docker.com/engine/reference/commandline/build/) and [run](https://docs.docker.com/engine/reference/commandline/run/) references.

##### Local Repository Setup

After installing the default `ROS2 foxy` version add the following packages:

```sh
sudo apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-gazebo-ros \
    ros-foxy-gazebo-dev \
    ros-foxy-gazebo-plugins \
    ros-foxy-tf2-ros \
    ros-foxy-tf2 \
    ros-foxy-rviz2 \
    ros-foxy-xacro \
    ros-foxy-urdf \
    ros-foxy-urdfdom \
    ros-foxy-robot-state-publisher \
    ros-foxy-rqt
    ros-foxy-rqt-robot-steering
```

Create a Workspace, clone the repository and compile it; skip this section if using `docker`.

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

###### Local

If using a local environment, before running the project make sure to source it:

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

###### Docker

If using `docker`:

```sh
cd ~/ws
# Run the sample
sudo ./run.sh
```

Execute `vncviewer`, `redmina` or any other VNC client pointing to `localhost`. From the `rqt` utility, select the `Robot Steering` option to change the linear velocity.

##### Behavior

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

Once running the expected nodes are detailed here:

```sh
ros2 node list
# Result
/differential_drive_controller
/gazebo
/gazebo_ros_floorscan_hokuyo_controller
/gazebo_ros_scanner_hokuyo_controller
/joint_state
/left_bogie_front_wheel
/left_bogie_rear_wheel
/left_rocker_rear_wheel
/lsbot_actuator_rotaryservo
/right_bogie_front_wheel
/right_bogie_rear_wheel
/right_rocker_rear_wheel
/robot_state_publisher
/rqt_gui
/rviz2
/transform_listener_impl_55a25a164aa0
```

The `lsbot_actuator_rotaryservo` detail:

```sh
ros2 node info /lsbot_actuator_rotaryservo 
# Result
/lsbot_actuator_rotaryservo
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /cmd_vel: geometry_msgs/msg/Twist
    /odom_shaft: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /floorscan/angle: lsbot_msgs/msg/Angle
    /lsbot_actuator_rotaryservo/state_axis1: lsbot_msgs/msg/StateRotaryServo
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /lsbot_actuator_rotaryservo/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lsbot_actuator_rotaryservo/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lsbot_actuator_rotaryservo/get_parameters: rcl_interfaces/srv/GetParameters
    /lsbot_actuator_rotaryservo/list_parameters: rcl_interfaces/srv/ListParameters
    /lsbot_actuator_rotaryservo/set_parameters: rcl_interfaces/srv/SetParameters
    /lsbot_actuator_rotaryservo/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /lsbot_actuator_rotaryservo/specs: lsbot_msgs/srv/SpecsRotaryServo
```

Subscribes to Odometry via `odom_shaft` and Velocity via `cmd_vel`, the main output is detailed on the `/floorscan/angle` and `lsbot_actuator_rotaryservo/state_axis1 publishers`.

Sample execution for the `/floorscan/angle` topic:

```
ros2 topic echo /floorscan/angle
# Result
header:
  stamp:
    sec: 645
    nanosec: 503000000
  frame_id: ''
status: 1
angle: 0.5235987901687622
```

Sample execution for the `lsbot_actuator_rotaryservo/state_axis1 publishers` topic:

```
ros2 topic echo lsbot_actuator_rotaryservo/state_axis1
# Sample Result
header:
  stamp:
    sec: 688
    nanosec: 704000000
  frame_id: ''
goal: 30.0
position: 0.5235987901161678
error: 29.476401209883832
velocity: 3.116157083742192e-07
effort: 0.0
load: 0.0
moving: false
fault: 0
control_type: 0
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

Tested on two environments, a VMware box using `18.04` plus `docker` and a `20.04` one.
The case could not be validated on a `16.04` environment, the following error appears related to `docker`:

```
Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?
```

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

The reasoning behing this decision is that the rover at low speeds will need to check obstacles close to it (higher servo angle position); but when moving at higher speeds it is reasonable to higher its position (lower servo angle position) to check obstacles closer to the horizon.

Parameters can be changed accordingly using:

* Velocity variables
  * `floorscan_velocity_low_range_`
  * `floorscan_velocity_middle_range_`
* Position variables
  * `floorscan_angle_low_`
  * `floorscan_angle_middle_`
  * `floorscan_angle_high_`

The project could be extended to:

 * use more angle positions based on the desired one or mix it with the obtained velocity.
 * change the behavior not to take into account its velocity and move the slanted lidar on a regular basis, defining an artificial field of view, create a `PointCloud` representation.

## Resources

* [Gazebo Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)
* [Robotiq modular gripper](https://github.com/YueErro/robotiq_modular_gripper.git)
* [MARA cobot](https://github.com/AcutronicRobotics/MARA)
* [vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace)