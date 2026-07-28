#!/usr/bin/env bash

export MOLA_SIGMA_MIN_MOTION=0.50
export MOLA_SIGMA_MAX_MOTION=3.0
export MOLA_ICP_MAX_ITERATIONS=2
export MOLA_MIN_ICP_QUALITY=0.90
export MOLA_MIN_KEYFRAME_DISTANCE=0.5

colcon build
source install/source.sh
ros2 launch man_mapping mola_slam.launch.py


