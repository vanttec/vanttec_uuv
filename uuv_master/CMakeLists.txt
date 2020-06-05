cmake_minimum_required(VERSION 3.0.2)
project(uuv_master)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    vehicle_user_control
)

add_message_files(
   FILES
   UUVMasterStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime vehicle_user_control
  DEPENDS ${LIBS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(uuv_master_node src/uuv_master_node.cpp src/master_node.cpp)
add_dependencies(uuv_master_node uuv_master_generate_messages_cpp)
target_link_libraries(uuv_master_node ${catkin_LIBRARIES})