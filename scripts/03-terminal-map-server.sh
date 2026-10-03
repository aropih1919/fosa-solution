#!/usr/bin/env bash
# 03-terminal-map-server — Map Server (fournit /map).
# Terminal 3 : lance le serveur, puis dans un 2e onglet activer le lifecycle.
#   ros2 lifecycle set /map_server configure
#   ros2 lifecycle set /map_server activate
#   ros2 lifecycle get /map_server
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$(ros2 pkg prefix caytu_nav_bringup)/share/caytu_nav_bringup/maps/stadium_map.yaml" \
  -p use_sim_time:=true
