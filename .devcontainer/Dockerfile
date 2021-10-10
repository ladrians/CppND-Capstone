FROM ros:foxy

# install ros package and required software to open GUI in the container
RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros-pkgs \
    x11vnc \
    xvfb \
    openbox

# optimize  by deleting unneeded apt lists
RUN rm -rf /var/lib/apt/lists/*

# prepare to open GUI in the container
RUN mkdir ~/.vnc

# windowing for terminal gazebo window
RUN echo "openbox &" >> /.bashrc

#autostart gazebo and ros node
# cd to the project directory
RUN echo "cd ~/cppND && gazebo world_name & " >> /.bashrc
# cd to the project workspace and build it
RUN echo "cd ~/cppND/CppND-Capstone && colcon build --symlink-install" >> /.bashrc
# source the built packages to be able to use them
RUN echo "source install/setup.bash" >> /.bashrc
# start the simulation/nodes 
RUN echo "ros2 launch lsbot_gazebo world.py gui:=false rqt:=false rviz:=false" >> /.bashrc

