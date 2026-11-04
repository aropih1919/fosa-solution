# Paquets utilisés

Inventaire établi à partir des `package.xml` et des imports Python du dépôt.

## Paquets du projet

| Paquet | Type de build |
|---|---|
| `caytu_config` | ament_python |
| `caytu_interfaces` | ament_cmake (messages et services) |
| `caytu_nav_solution` | ament_python |
| `caytu_nav_bringup` | ament_cmake |
| `caytu_perception` | ament_python |
| `caytu_course` | ament_python |

## Dépendances ROS 2

- **Build** : `ament_cmake`, `ament_python`, `rosidl_default_generators`, `rosidl_default_runtime`
- **Base** : `rclpy`, `launch`, `launch_ros`, `ament_index_python`, `rosidl_runtime_py`
- **Messages** : `std_msgs`, `geometry_msgs`, `sensor_msgs`, `sensor_msgs_py`, `nav_msgs`, `nav2_msgs`, `action_msgs`, `builtin_interfaces`
- **Transformations et image** : `tf2_ros`, `cv_bridge`
- **SLAM et filtres** : `slam_toolbox`, `laser_filters`
- **Nav2** : `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `nav2_smac_planner`, `nav2_regulated_pure_pursuit_controller`, `nav2_costmap_2d`

## Bibliothèques Python externes

- `numpy` (`python3-numpy`)
- `opencv` / `cv2` (`python3-opencv`)
- `yaml` (`python3-yaml`)
- `pytest` (`python3-pytest`, tests)
- `setuptools`
- `vosk` (reconnaissance vocale) : importé dans le code mais non déclaré dans un `package.xml`

## Bibliothèque standard Python

`array`, `collections`, `copy`, `dataclasses`, `glob`, `itertools`, `json`, `math`, `numbers`, `os`, `pathlib`, `re`, `signal`, `struct`, `sys`, `tempfile`, `time`, `types`, `unicodedata`

## Dépendances par paquet

- **caytu_config** → yaml, ament_index_python
- **caytu_interfaces** → std_msgs, geometry_msgs
- **caytu_nav_solution** → rclpy, nav2_msgs, geometry_msgs, action_msgs, builtin_interfaces, yaml, caytu_config
- **caytu_perception** → rclpy, sensor_msgs(_py), tf2_ros, cv_bridge, opencv, numpy, caytu_config, caytu_interfaces
- **caytu_course** → tous les paquets ci-dessus, plus slam_toolbox, laser_filters et les paquets Nav2
- **caytu_nav_bringup** → aucune dépendance déclarée
