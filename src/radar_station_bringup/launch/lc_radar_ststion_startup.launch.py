from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mindvision_camera'),
                'launch',
                'mv_launch.py'
            )
        )
    )

    livox_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros2_driver'),
                'launch',
                'livox_lidar_rviz_launch.py'
            )
        )
    )

    radar_ststion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('radar_station'),
                'launch',
                'radar_station.launch.py'
            )
        )
    )


    return LaunchDescription([
        camera_launch,
        livox_lidar_launch,
        radar_ststion
    ])