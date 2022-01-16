cmake_minimum_required(VERSION 2.8.3)
project(wr8_software)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

find_package(Boost REQUIRED)
catkin_package()

include_directories(
	${PROJECT_SOURCE_DIR}/../controller_wr_driver/include
	${catkin_INCLUDE_DIRS}
)

set(CAMERA_SENSER_NODE_NAME action_cam)
set(CAMERA_SENSER_NODE_SRC src/action_cam.cpp)

set(UC_ROS_LINK_NODE_NAME uc_ros_link)
set(UC_ROS_LINK_NODE_SRC src/uc_ros_link.cpp ${PROJECT_SOURCE_DIR}/../third_party/madproto/src/mproto.c)

add_executable(${UC_ROS_LINK_NODE_NAME} ${UC_ROS_LINK_NODE_SRC})
target_include_directories(${UC_ROS_LINK_NODE_NAME}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/../third_party/madproto/src
)
target_link_libraries(${UC_ROS_LINK_NODE_NAME}
  ${catkin_LIBRARIES}
)
