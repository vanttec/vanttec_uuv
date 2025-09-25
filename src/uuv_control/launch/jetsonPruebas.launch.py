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

    matriz_locacion = Node(
        package="uuv_control",
        executable="matriz_locacion",
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'uuv_control:=debug']
    )

    to_arduino_from_locacion = Node(
        package="uuv_master",
        executable="to_arduino_from_locacion.py",
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'uuv_master:=debug']
    )

    

    return LaunchDescription([
        matriz_locacion,
        to_arduino_from_locacion
    ])
