## Mise en place de workspace CLI
ros_src && source install/setup.sh

# Terminal 1 — Simulation
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch parc_robot_bringup task.launch.py  

# Terminal 2 — Filtre LiDAR (self-detection)
ros2 run laser_filters scan_to_scan_filter_chain \
  --ros-args \
  --params-file $(realpath ~/ros2_ws/src/fosa-solution/caytu_nav_bringup/config/laser_filter_params.yaml) \
  -r scan:=/scan -r scan_filtered:=/scan_filtered

# Terminal 3 — Carte statique
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=$(realpath ~/ros2_ws/src/fosa-solution/caytu_nav_bringup/maps/stadium_map.yaml) -p use_sim_time:=true

ros2 lifecycle set /map_server configure

ros2 lifecycle set /map_server activate

# Terminal 4 — AMCL
ros2 run nav2_amcl amcl \
  --ros-args --params-file $(realpath ~/ros2_ws/src/fosa-solution/caytu_nav_bringup/config/amcl_params.yaml)

ros2 lifecycle set /amcl configure

ros2 lifecycle set /amcl activate
# → donner la pose initiale via RViz2 (2D Pose Estimate)

# Terminal 5 — La pile navigation de Faneva
ros2 launch caytu_nav_bringup navigation.launch.py use_sim_time:=true

# Terminal 6 — Watchdog + point d'entrée
python3 ~/ros2_ws/src/fosa-solution/caytu_nav_solution/caytu_nav_solution/localization_watchdog.py &
ros2 run caytu_nav_solution task_solution.py --ros-args -p use_test_goal:=false

## Verification
ros2 lifecycle list
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

## ros2 action list