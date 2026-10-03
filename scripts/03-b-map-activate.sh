#!/usr/bin/env bash
# 03-b — 2e onglet du terminal 3 : active le map server (one-shot, quitte apres).
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate && ros2 lifecycle get /map_server
