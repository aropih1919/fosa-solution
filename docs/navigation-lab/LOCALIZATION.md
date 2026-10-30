# Localisation

| Mode | Disponible | Source `map → odom` | Remarques |
|---|---|---|---|
| `odom_imu` | Oui | `odom_imu_localizer` | Mode soumis. `/odom` + `/imu`, spawn de `task_params.yaml`; dérive de position possible, sans correction globale par carte. |
| `odom_only` | Oui | Le même `odom_imu_localizer` | Réutilise le nœud avec `use_imu_yaw=false`; mode de diagnostic, pas une stratégie recommandée pour slalom. |
| `amcl` | Oui si `nav2_amcl` est installé et les prérequis de carte sont satisfaits | AMCL | Carte PGM valide, spawn/but dans une zone libre, scan initial cohérent, pose initiale depuis le YAML et `/amcl_pose` reçu avant la mission. |
| `amcl_ekf` | **Indisponible** | — | Le paquet et l'URDF `parc_robot_bringup` sont absents ici. L'historique indique que DiffDrive publie déjà `odom → base_footprint`; le désactiver et vérifier les covariances/frames de l'IMU n'a pas été possible. Le mode restera bloqué plutôt que de créer un doublon TF. |

AMCL consomme `/scan_clean`, pas `/scan_filtered` ni `/scan_clear`. Le premier garde le frame lidar d'origine, retire les retours du sol et sert au marquage. `/scan_clear` est un scan artificiellement prolongé pour libérer la costmap. Les coordonnées de `initial_pose` sont les coordonnées monde du spawn, et non des coordonnées locales à la carte.

Avant AMCL, le laboratoire contrôle que la carte est un PGM P5 8 bits, que son image existe, que son origine est sans rotation et que le spawn et le but disposent d'au moins 0,5 m de cellules libres autour d'eux. `task_solution.py` compare ensuite le scan lidar reçu au départ aux murs de la carte. Une carte manquante, vide, hors repère, un scan absent ou moins de 30 retours utiles font échouer AMCL avant son lancement. La sélection AMCL ne passe jamais par la carte vide de secours.

Les essais historiques mentionnent une carte SLAM exprimée depuis le spawn, tandis que les coordonnées de mission viennent du repère Gazebo, et signalent une convergence AMCL insuffisante. L'ancien fichier AMCL utilisait `/scan_filtered`, avant le retrait des retours du sol. Ce sont des pistes plausibles, pas la preuve d'une cause unique. Le mode actuel utilise la carte café du repère Gazebo, `/scan_clean`, `set_initial_pose=true` et attend un message `/amcl_pose` ainsi que la TF avant d'envoyer le but.

La configuration AMCL suit les paramètres ROS 2 Jazzy `set_initial_pose` et `initial_pose` de la [documentation Nav2](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/).
