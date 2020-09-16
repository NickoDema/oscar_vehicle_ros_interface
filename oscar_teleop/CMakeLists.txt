cmake_minimum_required(VERSION 2.8.3)
project(oscar_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    joy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tb
  CATKIN_DEPENDS oscar_vehicle_driver
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
