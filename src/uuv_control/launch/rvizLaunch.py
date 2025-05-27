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

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('uuv_control'),
        'rviz',
        'my_uuv_vis.rviz'
    ])
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    visualizador = Node(
        package="uuv_control",
        executable="visualizador.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    return LaunchDescription([
        rviz2,
        visualizador
    ])
