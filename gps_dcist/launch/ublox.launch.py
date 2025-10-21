import os

import ament_index_python.packages

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('gps_dcist'),
        'config')
    param_config = os.path.join(config_directory, 'c94_m8p_rover.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['ublox_gps_node']['ros__parameters']
    container = ComposableNodeContainer(
            name='ublox_gps_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ublox_gps',
                    plugin='ublox_node::UbloxNode',
                    name='ublox_gps_node',
                    parameters=[params]),
            ],
            output='both',
    )

    return LaunchDescription([container])
