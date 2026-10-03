#!/usr/bin/env bash
# 04-b — 2e onglet du terminal 4 : active AMCL (one-shot, quitte apres).
# Puis RViz : Fixed Frame=map, 2D Pose Estimate en (-0.2,-7.4) vers le nord.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate && ros2 lifecycle get /amcl
