import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    mola_launch_dir = get_package_share_directory('mola_lidar_odometry')
    mola_launch_file = os.path.join(
        mola_launch_dir, 'ros2-launchs', 'ros2-lidar-odometry.launch.py'
    )

    smoother_dir = get_package_share_directory('mola_state_estimation_smoother')
    smoother_yaml = os.path.join(
        smoother_dir, 'params', 'state-estimation-smoother.yaml'
    )

    rviz_config_path = os.path.join(get_package_share_directory('man_mapping'), 'rviz', 'go2_slam.rviz')

    man_mapping_share = get_package_share_directory('man_mapping')

    rviz_config_path = os.path.join(man_mapping_share, 'rviz', 'go2_slam.rviz')

    go2_urdf_path = os.path.join(man_mapping_share, 'urdf', 'go2_description.urdf')

    robot_description = ParameterValue(Command(['cat ', go2_urdf_path]), value_type=str)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path], # Sets the config file
        output='screen'
    )
    

    mola_lio = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(mola_launch_file),
    launch_arguments={
        'use_rviz': "False",
        'lidar_topic_name': '/utlidar/cloud',
        'mola_tf_base_link': 'base_link',
        'lidar_qos_reliability': 'best_effort',
        'lidar_qos_depth': '100',
        'min_nearby_poses_occupied': '2',
        'odom_topic_name': '/utlidar/robot_odom',
        'odom_sensor_label': 'go2_wheel_odom',
        'publish_localization_following_rep105': 'False',
        'imu_topic_name': '/utlidar/imu_fixed',
        'use_imu_for_lio': 'True',
        'imu_gravity_correction': 'True',
        'imu_gravity_sigma_deg': '1.0',
        'initial_localization_method': 'InitLocalization::FixedPose',
        'mola_deskew_method': 'MotionCompensationMethod::IMU',
        'forward_ros_tf_odom_to_mola': 'False',
        'start_mapping_enabled': 'True',
    }.items()
)
    return LaunchDescription([
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }],
            remappings=[
                ('joint_states', '/joint_states'),
            ],
        ),
        rviz_node,
       Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_utlidar_lidar',
            arguments=[
                '--x', '0.0',        
                '--y', '0.0',
                '--z', '0.2',
                '--roll', '3.14',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'utlidar_lidar',
            ],
        ),

        Node(
            package='man_mapping',
            executable='laser_static_tf_publisher',
            name='imu_static_tf_publisher',
            output='screen',
            parameters=[{
                'parent_frame': 'base_link',
                'child_frame': 'utlidar_imu',
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_go2_urdf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base',
            ],
        ),
        Node(
            package='man_mapping',
            executable='imu_quaternion_normalizer',
            name='imu_quaternion_normalizer',
            output='screen',
            parameters=[{
                'input_topic': '/utlidar/imu',
                'output_topic': '/utlidar/imu_fixed',
                'drop_invalid': True,
            }],
        ),
        # Node(
        #     package='man_mapping',
        #     executable='odom_conditioner',
        #     name='odom_conditioner',
        #     output='screen',
        #     parameters=[{
        #         'input_topic': '/utlidar/robot_odom',
        #         'output_topic': '/utlidar/robot_odom_fixed',
        #         'frame_id': 'odom',
        #         'child_frame_id': 'base_link',
        #         'drop_non_monotonic_timestamps': True,
        #         'pose_cov_diag': [1.0, 1.0, 4.0, 1.0, 1.0, 0.5],
        #         'twist_cov_diag': [0.8, 0.8, 3.0, 4.0, 4.0, 2.0],
        #     }],
        # ),

        # Node(
        #     package='man_mapping',
        #     executable='virtual_odom_publisher',
        #     name='virtual_odom_publisher',
        #     output='screen',
        #     parameters=[{
        #         'odom_topic': '/utlidar/robot_odom',
        #         'odom_frame': 'odom',
        #         'base_frame': 'base_link',

        #         'enable_stationary_freeze': False,
        #         'linear_stationary_threshold': 0.30,
        #         'angular_stationary_threshold': 0.30,
        #         'stationary_hold_sec': 1.0,
        #     }],
        # ),
        mola_lio
    ])