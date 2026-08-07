#!/bin/bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select trajectory_predictor coord_transform --symlink-install
