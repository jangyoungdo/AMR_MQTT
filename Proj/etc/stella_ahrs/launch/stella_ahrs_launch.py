#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    
    config_dir = get_package_share_directory('stella_ahrs')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(config_dir, 'rviz', 'imu_test.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    driver_node = LifecycleNode(package='stella_ahrs',
                                node_executable='stella_ahrs_node',
                                node_name='stella_ahrs_node',
                                output='screen',
                                emulate_tty=True,
                                node_namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '2','base_link','imu_link'],
                    )
    return LaunchDescription([
      driver_node,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        rviz_cmd,
        tf2_node,
    ])


