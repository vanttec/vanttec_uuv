cmake_minimum_required(VERSION 3.0.2)
project(uuv_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    tf2_ros
    roscpp
    geometry_msgs
    vanttec_msgs
    # nodelet
    # pluginlib
)
catkin_package()

include_directories(
    libs/vanttec_controllers
    include/UUVs
    ${catkin_INCLUDE_DIRS}
)

add_executable(tf2_broadcaster_node
    src/UUVs/tf2_6dof_broadcast_node.cpp
    libs/vanttec_controllers/simulation/tf2_6dof_broadcaster.cpp)
target_link_libraries(tf2_broadcaster_node ${catkin_LIBRARIES})

add_executable(vtec_u4_simulation_node
    src/UUVs/models/vtec_u4_simulation_node.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vtec_u4_6dof_dynamic_model.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/base/marine_6dof_dynamic_model.cpp
    libs/vanttec_controllers/utils/utils.cpp)
target_link_libraries(vtec_u4_simulation_node ${catkin_LIBRARIES})

add_executable(inertial_vtec_u4_simulation_node
    src/UUVs/models/in_vtec_u4_simulation_node.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vtec_u4_in_6dof_dynamic_model.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/base/marine_6dof_in_dynamic_model.cpp
    libs/vanttec_controllers/utils/utils.cpp)
target_link_libraries(inertial_vtec_u4_simulation_node ${catkin_LIBRARIES})

# add_executable(vtec_u4_pid1_node
#     src/UUVs/controllers/first_order/vtec_u4_pid_control_node.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/PID_based/src/6dof_pid.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/PID_based/src/pid.cpp
#     libs/vanttec_controllers/utils/utils.cpp)
# target_link_libraries(vtec_u4_pid1_node ${catkin_LIBRARIES})

add_executable(vtec_u4_pid2_node
    src/UUVs/controllers/second_order/vtec_u4_pid_node.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/model_based_controllers/UUVs/vtec_u4_6dof_in_pid.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vtec_u4_in_6dof_dynamic_model.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/base/marine_6dof_in_dynamic_model.cpp

    libs/vanttec_controllers/controllers/feedback_linearization/PID_based/second_order/fblin_6dof_pid.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/base/fb_lin_6dof_control.cpp
    libs/vanttec_controllers/controllers/control_laws/PID/second_order/pid_6dof.cpp
    libs/vanttec_controllers/controllers/control_laws/PID/second_order/pid.cpp

    libs/vanttec_controllers/utils/utils.cpp)
target_link_libraries(vtec_u4_pid2_node ${catkin_LIBRARIES})
add_dependencies(vtec_u4_pid2_node vanttec_msgs_generate_messages_cpp)

add_executable(vtec_u4_pid2_traj_node
    src/UUVs/controllers/second_order/vtec_u4_pid_node_traj.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/model_based_controllers/UUVs/vtec_u4_6dof_in_pid.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vtec_u4_in_6dof_dynamic_model.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/base/marine_6dof_in_dynamic_model.cpp

    libs/vanttec_controllers/controllers/feedback_linearization/PID_based/second_order/fblin_6dof_pid.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/base/fb_lin_6dof_control.cpp
    libs/vanttec_controllers/controllers/control_laws/PID/second_order/pid_6dof.cpp
    libs/vanttec_controllers/controllers/control_laws/PID/second_order/pid.cpp

    libs/vanttec_controllers/utils/utils.cpp)
target_link_libraries(vtec_u4_pid2_traj_node ${catkin_LIBRARIES})
add_dependencies(vtec_u4_pid2_traj_node vanttec_msgs_generate_messages_cpp)

                # ${${PROJECT_NAME}_EXPORTED_TARGETS}
                # ${catkin_EXPORTED_TARGETS})

# add_executable(vtec_u4_asmc_node
#     src/UUVs/controllers/second_order/vtec_u4_asmc_node.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/src/6dof_asmc.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/src/asmc.cpp
#     libs/vanttec_controllers/utils/utils.cpp)
# target_link_libraries(vtec_u4_asmc_node ${catkin_LIBRARIES})

# add_executable(vtec_u4_antsmc_node
#     src/UUVs/controllers/second_order/vtec_u4_antsmc_node.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/src/6dof_antsmc.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/src/antsmc.cpp
#     libs/vanttec_controllers/utils/utils.cpp)
# target_link_libraries(vtec_u4_antsmc_node ${catkin_LIBRARIES})
