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
    man_mapping_share = get_package_share_directory('man_mapping')

    go2_urdf_path = os.path.join(man_mapping_share, 'urdf', 'go2_description.urdf')
    robot_description = ParameterValue(Command(['cat ', go2_urdf_path]), value_type=str)
    ekf_yaml_path = os.path.join(man_mapping_share, 'params', 'go2_ekf.yaml')

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
            package='man_mapping',
            executable='odom_covariance_fixer',
            name='odom_covariance_fixer',
            output='screen',
            parameters=[{
                'input_topic': '/utlidar/robot_odom',
                'output_topic' :'/utlidar/robot_odom_fixed'
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
                '--roll', '3.14',
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
                '--roll', '3.14',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'utlidar_lidar',
            ],
            output='screen',
        ),
        Node(
            package='man_mapping',
            executable='go2_lowstate_bridge',
            name='go2_lowstate_bridge',
            output='screen',
            parameters=[{
                'lowstate_topic': '/lf/lowstate',
                'imu_topic': '/imu/data',
                'joint_state_topic': '/joint_states',
                'foot_contact_topic': '/foot_contacts',
                'raw_foot_force_topic': '/foot_force/raw',
                'base_frame': 'base_link',

                'num_active_joints': 12,
                'use_mode_filter': True,
                'publish_raw_foot_force': True,
                'foot_force_threshold': 20.0,

                'imu.orientation_covariance': [
                    0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.02
                ],
                'imu.angular_velocity_covariance': [
                    0.001, 0.0, 0.0,
                    0.0, 0.001, 0.0,
                    0.0, 0.0, 0.002
                ],
                'imu.linear_acceleration_covariance': [
                    0.1, 0.0, 0.0,
                    0.0, 0.1, 0.0,
                    0.0, 0.0, 0.2
                ],
            }],
        ),
        # Node(
        #     package='man_mapping',
        #     executable='go2_leg_odometry',
        #     name='go2_leg_odometry',
        #     output='screen',
        #     # prefix=['gdbserver localhost:3000'],
        #     parameters=[
        #         {'robot_description': robot_description},
        #         {
        #             'joint_states_topic': '/joint_states',
        #             'foot_contacts_topic': '/foot_contacts',
        #             'imu_topic': '/imu/data',
        #             'odom_topic': '/odometry/leg',
        #             'base_link': 'base_link',
        #             'odom_frame': 'odom',
        #             'foot_links': ['FR_foot', 'FL_foot', 'RR_foot', 'RL_foot'],
        #         }
        #     ],
        # ),
        
         Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('odometry/filtered', '/odom')
            ],
            parameters=[
                ekf_yaml_path,
                {
                    'odom0': '/utlidar/robot_odom_fixed',
                    # 'odom1': '/utlidar/robot_odom_fixed',
                    'imu0': 'utlidar/imu',
                }
            ],
        ),
    ])