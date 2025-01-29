import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    this_dir = get_package_share_directory('vectornav')

    # Vectornav
    start_vectornav_cmd = Node(
        package='vectornav',
        executable='vectornav',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')])

    start_vectornav_sensor_msgs_cmd = Node(
        package='vectornav',
        executable='vn_sensor_msgs',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')])

    # kalman filter localization
    ekf_param_dir = LaunchConfiguration(
        'ekf_param_dir',
        default=os.path.join(
            get_package_share_directory('kalman_filter_localization'),
            'param',
            'ekf.yaml'))

    ekf = Node(
        package='kalman_filter_localization',
        executable='ekf_localization_node',
        parameters=[ekf_param_dir],
        remappings=[('/ekf_localization/gnss_pose', '/gnss_pose'),
                    ('/ekf_localization/imu', '/imu')],
        output='screen'
    )

    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_param_dir',
            default_value=ekf_param_dir,
            description='Full path to ekf parameter file to load'),
        ekf,
        tf,
        start_vectornav_cmd,
        start_vectornav_sensor_msgs_cmd
    ])

