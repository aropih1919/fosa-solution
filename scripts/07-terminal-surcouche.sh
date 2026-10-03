#!/usr/bin/env bash
# 07-terminal-surcouche — Gouverneur + inflation adaptative (branche locale, non poussee).
# Terminal 7 : a lancer APRES le 05 (Nav2 publie /surcouche/cmd_vel_raw via le remap).
# Sans ce terminal, le robot ne recoit AUCUNE consigne (l'ancien chemin cmd_vel est coupe).
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch caytu_nav_surcouche surcouche.launch.py
