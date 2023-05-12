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
    libs/vanttec_controllers/
    # libs/vanttec_controllers/controllers/feedback_linearization/PID_based/include
    # libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/include
    # libs/vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/include
    # libs/vanttec_controllers/dynamic_models/base
    # libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/models
    # libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vehicles
    # libs/vanttec_controllers/simulation/include
    include/UUVs
    ${catkin_INCLUDE_DIRS}
)

add_executable(tf2_broadcaster_node
    src/UUVs/6dof_tf2_broadcast_node.cpp
    libs/vanttec_controllers/simulation/6dof_tf2_broadcaster.cpp)
target_link_libraries(tf2_broadcaster_node ${catkin_LIBRARIES})

# add_executable(vtec_u4_simulation_node
#     src/UUVs/model/vtec_u4_simulation_node.cpp
#     libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vehicles/vtec_u4_6dof_dynamic_model.cpp
#     libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/models/marine_6dof_dynamic_model.cpp
#     libs/vanttec_controllers/dynamic_models/base/base_fblin_6dof_model.cpp
#     libs/vanttec_controllers/utils/utils.cpp)
# target_link_libraries(vtec_u4_simulation_node ${catkin_LIBRARIES})

# add_executable(inertial_vtec_u4_simulation_node
#     src/UUVs/model/in_vtec_u4_simulation_node.cpp
#     libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vehicles/vtec_u4_in_6dof_dynamic_model.cpp
#     libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/models/marine_6dof_in_dynamic_model.cpp
#     libs/vanttec_controllers/dynamic_models/base/base_fblin_6dof_model.cpp
#     libs/vanttec_controllers/utils/utils.cpp)
# target_link_libraries(inertial_vtec_u4_simulation_node ${catkin_LIBRARIES})

# add_executable(vtec_u4_pid1_node
#     src/UUVs/controllers/first_order/vtec_u4_pid_control_node.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/PID_based/src/6dof_pid.cpp
#     libs/vanttec_controllers/controllers/feedback_linearization/PID_based/src/pid.cpp
#     libs/vanttec_controllers/utils/utils.cpp)
# target_link_libraries(vtec_u4_pid1_node ${catkin_LIBRARIES})

add_executable(vtec_u4_pid2_node
    src/UUVs/controllers/second_order/vtec_u4_pid_node.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/model_based_controllers/UUVs/vtec_u4_6dof_in_pid.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/underwater/vehicles/vtec_u4_in_6dof_dynamic_model.cpp
    libs/vanttec_controllers/dynamic_models/marine_vehicles/base/marine_6dof_in_dynamic_model.cpp
    libs/vanttec_controllers/dynamic_models/base/fblin_6dof_model.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/PID_based/second_order/6dof_pid.cpp
    libs/vanttec_controllers/controllers/feedback_linearization/base/fb_lin_6dof_control.cpp
    libs/vanttec_controllers/controllers/control_laws/PID/second_order/pid.cpp
    libs/vanttec_controllers/utils/utils.cpp)
target_link_libraries(vtec_u4_pid2_node ${catkin_LIBRARIES})

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
