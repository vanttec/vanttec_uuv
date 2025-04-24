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
    objetivo_hundir = Node(
        package="uuv_control",
        executable="objetivo_hundir.cpp",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    pid = Node(
        package="uuv_control",
        executable="pid6dof_node.cpp",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    matriz_locacion = Node(
        package="uuv_control",
        executable="matriz_locacion.cpp",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    return LaunchDescription([
        objetivo_hundir,
        pid,
        matriz_locacion
    ])