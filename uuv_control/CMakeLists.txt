cmake_minimum_required(VERSION 3.0.2)
project(uuv_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_generation
)

add_message_files(
   FILES
   ThrustControl.msg
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

add_executable(uuv_odometry_node src/uuv_odometry_node src/odometry_calculator.cpp)
target_link_libraries(uuv_odometry_node ${catkin_LIBRARIES})

add_executable(uuv_control_node src/uuv_control_node src/uuv_4dof_controller.cpp src/pid_controller.cpp)
target_link_libraries(uuv_control_node ${catkin_LIBRARIES})
