import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    man_mapping_share = get_package_share_directory('man_mapping')
    localization_launch = os.path.join(man_mapping_share, 'launch', 'localization.launch.py')
    
    localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(localization_launch))
    # rviz_config_path = os.path.join(man_mapping_share, 'rviz', 'go2_slam.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', rviz_config_path],
            output='screen'
    )
    return LaunchDescription([
        localization, 
        Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'utlidar/cloud_deskewed'),
                    ('scan', '/scan')
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'min_height': 0.1,
                    'max_height': 1.0,
                    'range_min': 0.1,
                    'range_max': 10.0,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'use_inf': True
                }]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'do_loop_closing': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
                'transform_publish_period': 0.5,
                'map_update_interval': 0.5,
                'max_laser_range': 10.0,
                'minimum_time_interval': 0.09,
                'minimum_travel_distance': 0.03,
                'minimum_travel_heading': 0.08,
                'transform_timeout': 0.3,
                'tf_buffer_duration': 10.0,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'loop_search_maximum_distance': 2.0,
                'loop_match_minimum_chain_size': 5,
                'loop_match_minimum_response_coarse': 0.45,
                'loop_match_minimum_response_fine': 0.55
            }]
        ),
        rviz_node
    ])