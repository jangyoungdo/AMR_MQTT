from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # 추가된 부분
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mqtt_client'),
        'config',
        'mqtt_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mqtt_client',
            executable='mqtt_client',
            name='mqtt_client_node',
            parameters=[config]
        )
    ])
