import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('man_mapping')
    urdf_path = os.path.join(share, 'urdf', 'go2_description.urdf')
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
    )

    odom_covariance_fixer = Node(
        package='man_mapping',
        executable='odom_covariance_fixer',
        name='odom_covariance_fixer',
        output='screen',
        parameters=[{
            'input_topic': '/utlidar/robot_odom',
            'output_topic': '/utlidar/robot_odom_fixed',
        }],
    )

    # imu_covariance_fixer = Node(
    #     package='man_mapping',
    #     executable='imu_covariance_fixer',
    #     name='imu_covariance_fixer',
    #     output='screen',
    #     parameters=[{
    #         'input_topic': '/utlidar/imu',
    #         'output_topic': '/utlidar/imu_fixed',

    #         'orientation_diagonal': [0.05, 0.05, 0.20],
    #         'angular_velocity_diagonal': [0.0025, 0.0025, 0.0025],
    #         'linear_acceleration_diagonal': [0.25, 0.25, 0.25],
    #     }],
    # )

    imu_gyro_bias = Node(
        package='man_mapping',
        executable='imu_bias_corrector',
        name='imu_bias_corrector',
        output='screen',
        parameters=[{
            'input_topic': '/utlidar/imu',
            'output_topic': '/utlidar/imu_bias_corrected',
            'startup_calibration_seconds' : 10.0,
            'gyro_stationary_threshold_rad_s': 0.5,
            'accel_magnitude_tolerance_m_s2': 1.25,
            'stationary_bias_alpha': 0.001,
            'orientation_diagonal': [0.05, 0.05, 0.20],
            'angular_velocity_diagonal': [0.025, 0.025, 0.025],
            'linear_acceleration_diagonal': [0.25, 0.25, 0.25] 
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'frequency': 50.0,
            'sensor_timeout': 0.1,
            'two_d_mode': False,
            'publish_tf': True,
            'print_diagnostics': True,

            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',

            'odom0': '/utlidar/robot_odom_fixed',
            'odom0_config': [
                True, True, False,
                False, False, False,
                True, True, False,
                False, False, True,
                False, False, False,
            ],
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_queue_size': 20,

            'imu0': '/utlidar/imu_bias_corrected',
            'imu0_config': [
                False, False, False,
                False, False, True,
                False, False, False,
                False, False, True,
                False, False, False,
            ],
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_remove_gravitational_acceleration': False,
            'imu0_queue_size': 100,
        }],
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_utlidar_imu',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
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
                '--z', '0.0',
                '--roll', '3.14159265359',
                '--pitch', '0.0',
                '--yaw', '1.57079632679',
                '--frame-id', 'base_link',
                '--child-frame-id', 'utlidar_lidar',
            ],
            output='screen',
        ),
        robot_state_publisher,
        odom_covariance_fixer,
        imu_gyro_bias,
        # imu_covariance_fixer,
        ekf,
    ])