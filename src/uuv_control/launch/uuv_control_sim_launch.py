import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():
    dynamic_sim_node = Node(
        package="uuv_control",
        executable="dynamic_model_uuv",
        parameters=[
            {"subname": "uuv"},
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_description'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
    )

    foxglove_bridge = Node(
        name="foxglove_bridge",
        package="foxglove_bridge",
        executable="foxglove_bridge")

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_control'),
                'launch',
                'teleop_launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        rviz,
        dynamic_sim_node,
        # foxglove_bridge,
        teleop_launch,
    ])
