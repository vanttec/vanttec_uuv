cmake_minimum_required(VERSION 3.0.2)
project(vanttec_uuv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    gazebo_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    vehicle_user_control
    visualization_msgs
    pcl_ros
    roscpp
    sensor_msgs
    cv_bridge
)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
   FILES
   ThrustControl.msg
   GuidanceWaypoints.msg
   MasterStatus.msg
   Obstacle.msg
   DetectedObstacles.msg
   MissionStatus.msg
   obj_detected.msg
   obj_detected_list.msg
   obstacles_list.msg
)

add_service_files(
   FILES
   color_id.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs sensor_msgs vehicle_user_control pcl_ros
  DEPENDS ${LIBS}
)

include_directories(
    lib/uuv_common/include
    lib/uuv_odometry/include
    lib/uuv_control/include
    lib/uuv_guidance/include
    lib/uuv_master/include
    lib/uuv_motion_planning/include
    lib/uuv_simulation/include
    lib/uuv_missions/include
    include ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(uuv_odometry_node 
    src/uuv_odometry_node.cpp 
    lib/uuv_odometry/src/odometry_calculator.cpp)
add_dependencies(uuv_odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_odometry_node ${catkin_LIBRARIES})

add_executable(uuv_control_node 
    src/uuv_control_node.cpp 
    lib/uuv_control/src/uuv_4dof_controller.cpp 
    lib/uuv_control/src/pid_controller.cpp
)
add_dependencies(uuv_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_control_node ${catkin_LIBRARIES})

add_executable(asmc_guidance_unit_test 
    tests/asmc_guidance_unit_test.cpp
    lib/uuv_control/src/asmc.cpp
    lib/uuv_guidance/src/asmc_guidance.cpp
    lib/uuv_guidance/src/asmc_guidance_4dof.cpp)
add_dependencies(asmc_guidance_unit_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(asmc_guidance_unit_test ${catkin_LIBRARIES})

add_executable(uuv_simulation_node 
    src/uuv_simulation_node.cpp 
    lib/uuv_simulation/src/uuv_dynamic_4dof_model.cpp)
add_dependencies(uuv_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_simulation_node ${catkin_LIBRARIES})

add_executable(uuv_gate_simulation_node 
    src/uuv_gate_simulation_node.cpp) 
add_dependencies(uuv_gate_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_gate_simulation_node ${catkin_LIBRARIES})

add_executable(uuv_buoy_simulation_node 
    src/uuv_buoy_simulation_node.cpp) 
add_dependencies(uuv_buoy_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_buoy_simulation_node ${catkin_LIBRARIES})

add_executable(uuv_shootout_simulation_node 
    src/uuv_shootout_simulation_node.cpp) 
add_dependencies(uuv_shootout_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_shootout_simulation_node ${catkin_LIBRARIES})

add_executable(uuv_tf_broadcast_node 
    src/uuv_tf_broadcast_node.cpp 
    lib/uuv_simulation/src/tf_broadcaster.cpp)
add_dependencies(uuv_tf_broadcast_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_tf_broadcast_node ${catkin_LIBRARIES})

add_executable(uuv_master_node 
    src/uuv_master_node.cpp 
    lib/uuv_master/src/master_node.cpp)
add_dependencies(uuv_master_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_master_node ${catkin_LIBRARIES})

add_executable(uuv_waypoint_publisher_node 
    src/uuv_waypoint_publisher_node.cpp
    lib/uuv_motion_planning/src/waypoint_publisher.cpp
    lib/uuv_common/src/uuv_common.cpp
)
add_dependencies(uuv_waypoint_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_waypoint_publisher_node ${catkin_LIBRARIES})

add_executable(uuv_guidance_node 
    src/uuv_guidance_node.cpp 
    lib/uuv_guidance/src/uuv_guidance_controller.cpp
    lib/uuv_control/src/asmc.cpp
    lib/uuv_guidance/src/asmc_guidance.cpp
    lib/uuv_guidance/src/asmc_guidance_4dof.cpp)
add_dependencies(uuv_guidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_guidance_node ${catkin_LIBRARIES})

add_executable(uuv_mission_manager_node 
    src/uuv_mission_manager_node.cpp 
    lib/uuv_missions/src/uuv_mission_manager.cpp 
    lib/uuv_missions/src/gate_mission.cpp
    lib/uuv_missions/src/buoy_mission.cpp
    lib/uuv_missions/src/shootout_mission.cpp
    lib/uuv_common/src/uuv_common.cpp
)
add_dependencies(uuv_mission_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(uuv_mission_manager_node ${catkin_LIBRARIES})

add_executable(gazebo_interface src/gazebo_interface.cpp)
target_link_libraries(gazebo_interface ${catkin_LIBRARIES})

link_directories(${PCL_LIBRARY_DIRS})
add_executable(gate_detector_node src/gate_detector.cpp)
target_link_libraries(gate_detector_node ${catkin_LIBRARIES})