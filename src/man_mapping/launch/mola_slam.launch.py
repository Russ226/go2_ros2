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

    man_mapping_share = get_package_share_directory('man_mapping')

    rviz_config_path = os.path.join(man_mapping_share, 'rviz', 'go2_slam.rviz')

    rviz_config_path = os.path.join(man_mapping_share, 'rviz', 'go2_slam.rviz')

    go2_urdf_path = os.path.join(man_mapping_share, 'urdf', 'go2_description.urdf')

    robot_description = ParameterValue(Command(['cat ', go2_urdf_path]), value_type=str)

    mola_pipe_path = os.path.join(get_package_share_directory('man_mapping'), 'params', 'go2_quadruped.yaml') 

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
        'lidar_qos_reliability': 'reliable',
        'lidar_qos_depth': '100',
        'min_nearby_poses_occupied': '1',
        # 'odom_topic_name': '/utlidar/robot_odom',
        # 'odom_sensor_label': 'go2_odom',
        'publish_localization_following_rep105': 'False',
        'imu_topic_name': '/utlidar/imu_fixed',
        'use_imu_for_lio': 'True',
        'imu_gravity_correction': 'True',
        'imu_gravity_sigma_deg': '5.0',
        'initial_localization_method': 'InitLocalization::FixedPose',
        'mola_deskew_method': 'MotionCompensationMethod::None',
        'forward_ros_tf_odom_to_mola': 'True',
        'start_mapping_enabled': 'True'
    }.items()
)
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
        rviz_node,
        Node(
            package='man_mapping',
            executable='go2_joint_state_bridge',
            name='go2_joint_state_bridge',
            output='screen',
            parameters=[{
                'lowstate_topic': '/lf/lowstate',
                'joint_states_topic': '/joint_states',

                'joint_names': [
                    # Unitree motor indices 0..2: front-right
                    'FR_hip_joint',
                    'FR_thigh_joint',
                    'FR_calf_joint',

                    # Unitree motor indices 3..5: front-left
                    'FL_hip_joint',
                    'FL_thigh_joint',
                    'FL_calf_joint',

                    # Unitree motor indices 6..8: rear-right
                    'RR_hip_joint',
                    'RR_thigh_joint',
                    'RR_calf_joint',

                    # Unitree motor indices 9..11: rear-left
                    'RL_hip_joint',
                    'RL_thigh_joint',
                    'RL_calf_joint',
                ],

                # Unitree Go2 standard: FR, FL, RR, RL; each hip, thigh, calf.
                'motor_indices': [
                    0, 1, 2,
                    3, 4, 5,
                    6, 7, 8,
                    9, 10, 11,
                ],

                # Begin with +1.0; calibrate against the URDF / physical pose.
                'signs': [
                    1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0,
                    1.0, 1.0, 1.0,
                ],

                'offsets': [
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                ],
            }],
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
        ),

        Node(
            package='man_mapping',
            executable='imu_bias_corrector',
            name='imu_bias_corrector',
            output='screen',
            parameters=[{
                'calibration_samples': 100,
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
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
            }]
        ),
        # Node(
        #     package='man_mapping',
        #     executable='virtual_odom_publisher',
        #     name='virtual_odom_publisher',
        #     output='screen'
        # ),
        mola_lio
    ])