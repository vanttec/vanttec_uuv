import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    this_dir = get_package_share_directory('uuv_localization')
    
    # Vectornav
    start_vectornav_cmd = Node(
        package='uuv_localization',
        executable='vectornav',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')])
    
    start_vectornav_sensor_msgs_cmd = Node(
        package='uuv_localization',
        executable='vn_sensor_msgs',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')])
    
    start_uuv_localization_cmd = Node(
        package='uuv_localization',
        executable='uuv_localization_node',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_vectornav_cmd)
    ld.add_action(start_vectornav_sensor_msgs_cmd)
    ld.add_action(start_uuv_localization_cmd)

    return ld
