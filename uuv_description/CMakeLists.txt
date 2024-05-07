cmake_minimum_required(VERSION 3.0.2)
project(uuv_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
      geometry_msgs
      #nav_msgs
      genmsg
      tf2
      tf2_ros
      tf2_geometry_msgs
      gazebo_msgs
      message_generation
      std_msgs
)
find_package(gazebo REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EtaPose.msg
  Gate.msg 
  MasterStatus.msg
  MissionStatus.msg
  ObjectDetected.msg
  ObjectDetectedList.msg
  Obstacle.msg
  SplineDefinition.msg
  SystemDynamics.msg
  ThrustControl.msg
  Trajectory.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   #nav_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(gazebo_interface src/gazebo_interface.cpp)
add_executable(gazebo_interface_uuv src/gazebo_interface_uuv.cpp)
target_link_libraries(gazebo_interface ${catkin_LIBRARIES})
target_link_libraries(gazebo_interface_uuv ${catkin_LIBRARIES})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/shooter_description.cpp
# )

#add_library(SixDOFMov
#            lib/mobility/include/SixDOFMov.hpp
#            lib/mobility/src/SixDOFMov.cpp)
#target_link_libraries(SixDOFMov ${GAZEBO_LIBRARIES})



#############
## Install ##
#############

#############
## Testing ##
#############