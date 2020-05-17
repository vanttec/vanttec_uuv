cmake_minimum_required(VERSION 2.8.3)
project(vehicle_user_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  message_generation
)

find_package(SDL REQUIRED)
set(LIBS ${SDL_LIBRARY})

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  KeyboardKey.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS ${LIBS}
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${SDL_INCLUDE_DIR} include)

add_executable(vehicle_user_control src/vehicle_user_control.cpp src/keyboard.cpp)

target_link_libraries(vehicle_user_control
  ${LIBS}
  ${catkin_LIBRARIES}
)

add_dependencies(vehicle_user_control ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS vehicle_user_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
