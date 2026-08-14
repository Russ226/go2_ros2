cmake_minimum_required(VERSION 3.8)
project(man_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)


add_executable(cloud_to_scan src/PointToCloud.cpp)
ament_target_dependencies(cloud_to_scan rclcpp sensor_msgs)

add_executable(odom_tf_broadcaster src/OdomTfBroadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(laser_static_tf_publisher src/LaserStaticTfPublisher.cpp)
ament_target_dependencies(laser_static_tf_publisher rclcpp geometry_msgs tf2 tf2_ros)

add_executable(map_pose_tf_broadcaster src/map_pose_tf_broadcaster.cpp)
ament_target_dependencies(map_pose_tf_broadcaster rclcpp geometry_msgs tf2_ros tf2 nav_msgs)

add_executable(gyro_bias_corrector src/gyro_bias_corrector.cpp)
ament_target_dependencies(gyro_bias_corrector rclcpp sensor_msgs nav_msgs)

add_executable(virtual_odom_publisher src/virtual_odom_publisher.cpp)
ament_target_dependencies(virtual_odom_publisher rclcpp nav_msgs tf2_ros geometry_msgs tf2 tf2_geometry_msgs sensor_msgs)

add_executable(odom_covariance_fixer src/odom_covariance_fixer.cpp)
ament_target_dependencies(odom_covariance_fixer rclcpp nav_msgs)

add_executable(imu_quaternion_normalizer src/imu_quaternion_normalizer.cpp)
ament_target_dependencies(imu_quaternion_normalizer rclcpp sensor_msgs)

add_executable(mola_pose_tf_broadcaster src/mola_pose_tf_broadcaster.cpp)
ament_target_dependencies(mola_pose_tf_broadcaster rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(odom_conditioner src/odom_conditioner.cpp)
ament_target_dependencies(odom_conditioner rclcpp nav_msgs)

add_executable(pointcloud_rotator src/pointcloud_rotator.cpp)
ament_target_dependencies(pointcloud_rotator rclcpp sensor_msgs)

add_executable(go2_joint_state_bridge src/go2_joint_state_bridge.cpp)
ament_target_dependencies( go2_joint_state_bridge rclcpp sensor_msgs unitree_go)

add_executable(imu_bias_corrector src/imu_bias_corrector.cpp)
ament_target_dependencies(imu_bias_corrector rclcpp sensor_msgs)

add_executable(odom_fixed_publisher src/odom_fixed_publisher.cpp)
ament_target_dependencies(odom_fixed_publisher rclcpp nav_msgs tf2_ros geometry_msgs tf2 tf2_geometry_msgs sensor_msgs)

add_executable(go2_lowstate_bridge src/go2_lowstate_bridge.cpp)
ament_target_dependencies(go2_lowstate_bridge nav_msgs rclcpp sensor_msgs unitree_go std_msgs)

add_executable(go2_leg_odometry src/go2_leg_odometry.cpp)
ament_target_dependencies(go2_leg_odometry rclcpp sensor_msgs std_msgs nav_msgs urdf)

add_executable(imu_covariance_fixer src/imu_covariance_fixer.cpp)
ament_target_dependencies(imu_covariance_fixer rclcpp sensor_msgs)

add_executable(cloud_frame_accumulator src/cloud_frame_accumulator.cpp)
ament_target_dependencies(cloud_frame_accumulator rclcpp sensor_msgs)

target_link_libraries(go2_leg_odometry pinocchio::pinocchio)

install(DIRECTORY
  launch
  meshes
  dae
  params
  rviz
  urdf
  # config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  imu_bias_corrector
  go2_joint_state_bridge
  cloud_to_scan
  odom_tf_broadcaster
  laser_static_tf_publisher
  map_pose_tf_broadcaster
  gyro_bias_corrector
  virtual_odom_publisher
  odom_covariance_fixer
  imu_quaternion_normalizer
  mola_pose_tf_broadcaster
  odom_conditioner
  pointcloud_rotator
  go2_lowstate_bridge
  go2_leg_odometry
  imu_covariance_fixer
  cloud_frame_accumulator
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
