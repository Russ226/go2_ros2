#!/bin/bash

my_array=(
  /base_to_utlidar_imu
  /base_to_utlidar_lidar
  /ekf_filter_node
  /go2_leg_odometry
  /go2_lowstate_bridge
  /mola_bridge_ros2
  /odom_covariance_fixer
  /robot_state_publisher
  /rviz2
  /transform_listener_impl_5729db5a0140
  /transform_listener_impl_598c4c75e780
  /transform_listener_impl_7134480a7230
)

for item in "${my_array[@]}"; do
  echo "=== $item ==="
  ros2 param get "$item" use_sim_time
done

