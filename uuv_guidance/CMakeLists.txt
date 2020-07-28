cmake_minimum_required(VERSION 3.0.2)
project(uuv_guidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
   FILES
   GuidanceWaypoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS ${LIBS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(uuv_guidance_node src/uuv_guidance_node.cpp src/uuv_guidance_controller.cpp)
target_link_libraries(uuv_guidance_node ${catkin_LIBRARIES})

