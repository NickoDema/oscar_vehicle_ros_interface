cmake_minimum_required(VERSION 2.8.3)
project(oscar_vehicle_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    std_msgs
)

add_message_files(
  FILES
  VehicleStatus.msg
  VehicleCmd.msg
  VehicleCmdRaw.msg
)

add_service_files(
  FILES
  Trigger.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
