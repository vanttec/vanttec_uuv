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
        arguments=[('__log_level:=debug')]
    )

    teleop_forces_node = Node(
        package="uuv_control",
        executable="teleop_node_forces.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

    matriz_locacion = Node(
        package="uuv_control",
        executable="matriz_locacion.cpp",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )

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

    joystick = Node(
        package="uuv_control",
        executable="joystick.py",
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
    )



    teleop_launch = os.path.join(
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    )

    joy_type = DeclareLaunchArgument(
        'controller',
        default_value='xbox',
        description='Controller type (ps3/xbox/...)'
    )

    

    return LaunchDescription([
        # joy_node,
        teleop_forces_node,
        rviz2,
        joystick,
        visualizador,
        dynamic_model_uuv,
        pid,
        # matriz_locacion
        joy_type,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments={'joy_config': LaunchConfiguration('controller')}.items()
        )
    ])
