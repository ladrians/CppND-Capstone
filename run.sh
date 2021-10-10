#!/bin/bash
docker run --name capstone --rm -p 5900:5900 \
    -e HOME=/ \
    -v "$PWD":/cppND/CppND-Capstone:rw \
    gazebo_ros2:0.0 \
    x11vnc -forever -create

