import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    man_mapping_share = get_package_share_directory('go2_cam')

    go2_urdf_path = os.path.join(man_mapping_share, 'urdf', 'go2_description.urdf')
    robot_description = ParameterValue(Command(['cat ', go2_urdf_path]), value_type=str)
    rviz_config_path = os.path.join(man_mapping_share, 'rviz', 'config.rviz')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }]
        ),
        Node(
            package='go2_cam',
            executable='odom_covariance_fixer',
            name='odom_covariance_fixer',
            output='screen',
            parameters=[{
                'input_topic': '/utlidar/robot_odom',
                'output_topic' :'/robot_odom_fixed'
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_utlidar_imu',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.2',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'utlidar_imu',
            ],
            output='screen',
        ),
       Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_utlidar_lidar',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.2',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'utlidar_lidar',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_test',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_test',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
        ),
        Node(
            package='go2_cam',
            executable='go2_video_to_image',
            name='go2_video_to_image',
            output='screen',
            parameters=[{
                'input_topic': '/frontvideostream',
                'output_topic' :'/utlidar/camera',
                'resolution' : '720p',
                'frame_id' : 'front_camera_optical_frame',
                'network_interface': 'enp0s31f6'
                }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])