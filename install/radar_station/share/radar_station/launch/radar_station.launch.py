from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

config_file = os.path.join(
    get_package_share_directory('radar_station'),
    'config',
    'radar_station_conf.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_station',
            executable='radar_station',
            name='radar_station',
            parameters=[config_file],
            output='screen',
        )
    ])