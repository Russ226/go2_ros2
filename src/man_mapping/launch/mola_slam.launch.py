import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    mola_launch_dir = get_package_share_directory('mola_lidar_odometry')
    mola_launch_file = os.path.join(
        mola_launch_dir, 'ros2-launchs', 'ros2-lidar-odometry.launch.py'
    )

    smoother_dir = get_package_share_directory('mola_state_estimation_smoother')
    smoother_yaml = os.path.join(
        smoother_dir, 'params', 'state-estimation-smoother.yaml'
    )

    SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC', '0.1')
    SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC', '0.1')
    SetEnvironmentVariable('MOLA_IMU_GRAVITY_ALIGNMENT_SIGMA', '0.1')

    mola_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mola_launch_file),
        launch_arguments={
            'lidar_topic_name': '/utlidar/cloud_deskewed',
            'mola_tf_base_link': 'base_link',
            'lidar_qos_reliability': 'best_effort',
            'lidar_qos_depth': '200',
            'min_nearby_poses_occupied': '3',
            'odom_topic_name': '/utlidar/robot_odom',
            'odom_sensor_label': 'go2_wheel_odom',
            'publish_localization_following_rep105': 'False',
            'imu_topic_name': '/utlidar/imu',
            'use_imu_for_lio': 'True',
            'imu_gravity_correction': 'True',
            'imu_gravity_sigma_deg': '2.0',
            'use_state_estimator': 'True',
            'state_estimator_config_yaml': smoother_yaml,
            'navstate_sliding_window_sec': '2.0',
            'navstate_kinematic_model': 'KinematicModel::ConstantVelocity',
            'forward_ros_tf_odom_to_mola': 'False',
        }.items()
    )
    return LaunchDescription([
        mola_lio,

        Node(
            package='man_mapping',
            executable='laser_static_tf_publisher',
            name='laser_static_tf_publisher',
            output='screen',
            parameters=[{
                'parent_frame': 'base_link',
                'child_frame': 'utlidar_lidar',
                'x': 0.0, 'y': 0.0, 'z': 0.12,
                'roll': 0.0002, 'pitch': 0.3462, 'yaw': 0.0,
            }]
        ),

        Node(
            package='man_mapping',
            executable='laser_static_tf_publisher',
            name='imu_static_tf_publisher',
            output='screen',
            parameters=[{
                'parent_frame': 'base_link',
                'child_frame': 'utlidar_imu',
                'x': 0.0, 'y': 0.0, 'z': 0.12,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            }]
        ),

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
                'publish_tf': True, 
            }]
        ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[{
        #         'frequency': 50.0,
        #         'sensor_timeout': 0.2,
        #         'two_d_mode': False,
        #         'publish_tf': True,
        #         'map_frame': 'map',
        #         'odom_frame': 'odom',
        #         'base_link_frame': 'base_link',
        #         'world_frame': 'odom',

        #         'odom0': '/utlidar/robot_odom',
        #         'odom0_config': [False, False, False,
        #                           False, False, False,
        #                           True,  True,  False,
        #                           False, False, True,
        #                           False, False, False],
        #         'odom0_differential': False,
        #         'odom1': '/lidar_odometry/pose',
        #         'odom1_config': [True,  True,  True,
        #                           True,  True,  True,
        #                           False, False, False,
        #                           False, False, False,
        #                           False, False, False],
        #         'odom1_differential': False,
        #     }],
        # )
    ])