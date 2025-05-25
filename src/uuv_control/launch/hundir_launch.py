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
    estados = Node(
        package="uuv_control",
        executable="estados",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    objetivo_hundir = Node(
        package="uuv_control",
        executable="objetivo_hundir",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )
    
    dynamic_model_uuv = Node(
        package="uuv_control",
        executable="dynamic_model_uuv",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )

    pid = Node(
        package="uuv_control",
        executable="pid.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )
    
    visualizador = Node(
        package="uuv_control",
        executable="visualizador.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    

    return LaunchDescription([
    	dynamic_model_uuv,
        estados,
        objetivo_hundir,
        pid,
        visualizador,
        # matriz_locacion,
        rviz2
    ])
