import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    teleop_node = Node(
        package="uuv_control",
        executable="teleop_node_forces.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    can_node = Node(
    package="uuv_can",
    executable="uuv_can_node", 
    output='screen',
    emulate_tty=True,
    arguments=[('__log_level:=debug')],
    )


    return LaunchDescription([
        joy_node,
        teleop_node,
        can_node
    ])
