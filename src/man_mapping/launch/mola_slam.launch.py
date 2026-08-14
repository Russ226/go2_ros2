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
    smoother_dir = get_package_share_directory('mola_state_estimation_smoother')
    smoother_yaml = os.path.join(
        smoother_dir, 'params', 'state-estimation-smoother.yaml'
    )
    mola_launch_dir = get_package_share_directory('mola_lidar_odometry')
    mola_launch_file = os.path.join(
        mola_launch_dir, 'ros2-launchs', 'ros2-lidar-odometry.launch.py'
    )

    man_mapping_share = get_package_share_directory('man_mapping')
    rviz_config_path = os.path.join(man_mapping_share, 'rviz', 'go2_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        # parameters=[{'use_sim_time': True}],
        output='screen'
    )
    localization_launch = os.path.join(
        man_mapping_share, 'launch', 'mola.localization.launch.py'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch)
    )
    
    mola_lio = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(mola_launch_file),
    launch_arguments={
        # 'use_sim_time': 'True',
        'start_active': "True",
        # 'use_mola_gui': 'False',
        'use_namespace': 'False',
        'use_rviz': 'False',
        'lidar_topic_name': '/utlidar/cloud_accumulated',
        'lidar_qos_depth': '10',
        'imu_qos_depth': '1000',
        'imu_topic_name': '/utlidar/imu_bias_corrected',
        'mola_tf_base_link': 'base_link',
        'odom_topic_name': '/odometry/filtered',
        'odom_sensor_label': 'ekf_odom',
        'publish_localization_following_rep105': 'True',
        'forward_ros_tf_odom_to_mola': 'False',
        'use_imu_for_lio': 'True',
        'imu_gravity_correction': 'False',
        'imu_gravity_sigma_deg': '5.0',
        'initial_localization_method': 'InitLocalization::FixedPose',
        'mola_deskew_method': 'MotionCompensationMethod::IMU',
        'start_mapping_enabled': 'True',
        'enforce_planar_motion':'False',
        'generate_simplemap': 'False'
        
    }.items())
    
       
    return LaunchDescription([
        # localization,
        rviz_node,
        mola_lio
    ])