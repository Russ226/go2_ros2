#!/usr/bin/env bash
# export MOLA_WRITE_DEBUG_ICP_LOG_IF_QUALITY_UNDER=0.6
export MOLA_MINIMUM_ICP_QUALITY=0.65
export MOLA_SIGMA_INITIAL=0.20
export MOLA_SIGMA_MIN_MOTION=0.07
export MOLA_SIGMA_MAX_MOTION=0.40

colcon build
source install/setup.sh
ros2 launch man_mapping mola_slam.launch.py


