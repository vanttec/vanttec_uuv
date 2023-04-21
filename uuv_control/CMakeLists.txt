cmake_minimum_required(VERSION 3.0.2)
project(uuv_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    tf2_ros
    roscpp
    vanttec_msgs
    # nodelet
    # pluginlib
)
catkin_package()

include_directories(
    lib/vanttec_controllers/common_lib/include
    lib/vanttec_controllers/controllers/feedback_linearization/PID_based/include
    lib/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/include
    lib/vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/include
    lib/vanttec_controllers/dynamic_models/marine_vehicles/include
    lib/vanttec_controllers/simulation/include
    include/UUVs
    ${catkin_INCLUDE_DIRS}
)

add_executable(tf2_broadcaster_node
    src/UUVs/6dof_tf2_broadcast_node.cpp
    lib/vanttec_controllers/simulation/src/6dof_tf2_broadcaster.cpp)
target_link_libraries(tf2_broadcaster_node ${catkin_LIBRARIES})

add_executable(vtec_u4_simulation_node
    src/UUVs/model/vtec_u4_simulation_node.cpp
    lib/vanttec_controllers/dynamic_models/marine_vehicles/src/vtec_u4_6dof_dynamic_model.cpp
    lib/vanttec_controllers/dynamic_models/marine_vehicles/src/generic_6dof_uuv_dynamic_model.cpp)
target_link_libraries(vtec_u4_simulation_node ${catkin_LIBRARIES})

add_executable(inertial_vtec_u4_simulation_node
    src/UUVs/model/in_vtec_u4_simulation_node.cpp
    lib/vanttec_controllers/dynamic_models/marine_vehicles/src/vtec_u4_in_6dof_dynamic_model.cpp
    lib/vanttec_controllers/dynamic_models/marine_vehicles/src/generic_in_6dof_uuv_dynamic_model.cpp
    lib/vanttec_controllers/common_lib/src/common.cpp)
target_link_libraries(inertial_vtec_u4_simulation_node ${catkin_LIBRARIES})

add_executable(vtec_u4_pid_node
    src/UUVs/controllers/first_order/vtec_u4_pid_control_node.cpp
    lib/vanttec_controllers/controllers/feedback_linearization/PID_based/src/6dof_pid.cpp
    lib/vanttec_controllers/controllers/feedback_linearization/PID_based/src/pid.cpp
    lib/vanttec_controllers/common_lib/src/common.cpp)
target_link_libraries(vtec_u4_pid_node ${catkin_LIBRARIES})

add_executable(vtec_u4_asmc_node
    src/UUVs/controllers/second_order/vtec_u4_asmc_node.cpp
    lib/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/src/6dof_asmc.cpp
    lib/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/src/asmc.cpp
    lib/vanttec_controllers/common_lib/src/common.cpp)
target_link_libraries(vtec_u4_asmc_node ${catkin_LIBRARIES})

add_executable(vtec_u4_antsmc_node
    src/UUVs/controllers/second_order/vtec_u4_antsmc_node.cpp
    lib/vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/src/6dof_antsmc.cpp
    lib/vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/src/antsmc.cpp
    lib/vanttec_controllers/common_lib/src/common.cpp)
target_link_libraries(vtec_u4_antsmc_node ${catkin_LIBRARIES})
