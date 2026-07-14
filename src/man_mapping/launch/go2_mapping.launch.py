import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('man_mapping'),
        'rviz',
        'go2_slam.rviz')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
                'transform_publish_period': 0.02,
                'map_update_interval': 0.5,
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.1,
                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.05,
                'transform_timeout': 0.3,
                'tf_buffer_duration': 10.0,
                'use_scan_matching': True,
                'correlation_search_space_dimension': 2.0,
                'correlation_search_space_resolution': 0.04,
                'correlation_search_space_smear_deviation': 0.2,
                'use_scan_matching': False,
                'use_scan_barycenter': False
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])