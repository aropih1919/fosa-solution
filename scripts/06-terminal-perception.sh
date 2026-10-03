#!/usr/bin/env bash
# 06-perception — Filtre hauteur + watchdog camera (branche locale, non poussee).
# Terminal 6 : flux /top_camera/obstacles_points + signal /camera_perception_ready.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix caytu_camera_perception >/dev/null 2>&1 || { echo "[ERREUR] package non builde. Lancer mission.sh -> build-cible"; exit 1; }
exec ros2 launch caytu_camera_perception perception.launch.py
