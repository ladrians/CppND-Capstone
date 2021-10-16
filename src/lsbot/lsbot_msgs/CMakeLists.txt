cmake_minimum_required(VERSION 2.8.3)

project(lsbot_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Angle.msg
  GoalRotaryServo.msg
  StateRotaryServo.msg
)

add_service_files(
  FILES
  SetAngle.srv
  SpecsRotaryServo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)