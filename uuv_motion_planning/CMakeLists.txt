cmake_minimum_required(VERSION 3.0.2)
project(uuv_motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    uuv_guidance
)

#add_message_files(
#   FILES
#   GuidanceWaypoints.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS ${LIBS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(uuv_motion_planning_node src/uuv_motion_planning_node.cpp)
target_link_libraries(uuv_motion_planning_node ${catkin_LIBRARIES})

