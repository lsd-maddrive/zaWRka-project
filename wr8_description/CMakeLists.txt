cmake_minimum_required(VERSION 2.8.3)
project(wr8_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
  realtime_tools
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CAR_INTERFACE_PLUGIN wr8_interface_plugin)

catkin_package(
  INCLUDE_DIRS plugins
  LIBRARIES ${CAR_INTERFACE_PLUGIN}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  plugins
)

add_library(${CAR_INTERFACE_PLUGIN}
  plugins/car_interface.cpp
)

target_link_libraries(${CAR_INTERFACE_PLUGIN}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

### Install

install(
  TARGETS 
  ${CAR_INTERFACE_PLUGIN}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    launch
    rviz
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
