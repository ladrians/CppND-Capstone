cmake_minimum_required(VERSION 3.5)
project(lsbot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY src include config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
