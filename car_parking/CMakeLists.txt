cmake_minimum_required(VERSION 2.8.3)
project(car_parking)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
)

add_message_files( 
    FILES
    Point2D.msg
    Points2D.msg
    Polygons.msg
    Statuses.msg
)

generate_messages( 
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_parking src/parking_node.cpp src/parking_core.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  DIRECTORY
    launch
    scripts
    src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
