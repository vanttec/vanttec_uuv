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

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("teleop_twist_joy"),
        'launch',
        'teleop-launch.py'
    ))
    )

    to_arduino = Node(
        package="uuv_master",
        executable="to_arduino.py",
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'uuv_master:=debug']
    )

    

    return LaunchDescription([
        joy_node,
        to_arduino
    ])
