from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='man_mapping',
            executable='odom_tf_broadcaster',
            name='odom_footprint_tf_broadcaster',
            output='screen',
            parameters=[{
                'odom_topic': '/utlidar/robot_odom',
                'odom_frame': 'odom',
                'base_footprint': 'base_footprint',
                'base_frame': 'base_link',
            }]
        ),

        Node(
            package='man_mapping',
            executable='laser_static_tf_publisher',
            name='laser_static_tf_publisher',
            output='screen',
            parameters=[{
                'parent_frame': 'base_link',
                'child_frame': 'utlidar_lidar',
                'x': 0.0,
                'y': 0.0,
                'z': 0.12,
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0,
            }]
        ),

       Node(
            package='man_mapping',
            executable='cloud_to_scan',
            name='cloud_to_scan',
            output='screen',
            parameters=[{
                'cloud_topic': '/utlidar/cloud',
                'scan_topic': '/scan',
                'output_frame': 'utlidar_lidar',
                'min_height': -10.0,
                'max_height': 10.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.01745,
                'scan_time': 0.065,
                'range_min': 0.10,
                'range_max': 30.0,
                'use_inf': True,
            }]
        ),
    ])