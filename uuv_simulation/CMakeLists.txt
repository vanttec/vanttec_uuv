cmake_minimum_required(VERSION 2.8.3)
project(uuv_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_generation
    uuv_control
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp std_msgs message_runtime uuv_control
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(uuv_simulation_node src/uuv_simulation_node.cpp src/uuv_dynamic_4dof_model.cpp)
target_link_libraries(uuv_simulation_node ${catkin_LIBRARIES})
