# Original code from: https://github.com/DemianMArin/my_vanttec_uuv.git
# Modified by: Abraham de Jesus Maldonado Mata
# Date: 06/02/2025

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('uuv_can'),
        'config/',
        'motorPosition.yaml'
        )
    
    uuv_can = Node(
        package='uuv_can',
        executable='uuv_can_node',
        name='uuv_can_node',
        parameters= [config]
    )

    return LaunchDescription([uuv_can])
