# Architecture de départ

## Base examinée

Le laboratoire part de `pre-final`, commit `61435e43dd76a482b971668a52409a31fbba6a20`, la version V2 soumise. Le point d'entrée PARC reste `ros2 run caytu_nav_solution task_solution.py`.

## Flux des nœuds

| Composant | Entrées | Sorties et rôle |
|---|---|---|
| `task_solution.py` | `task_params.yaml`, paramètres ROS | Lit le spawn et le but, choisit la carte, lance le bringup, envoie `NavigateToPose`, puis arrête les processus. |
| `scan_to_scan_filter_chain` | `/scan` (`sensor_msgs/LaserScan`) | `/scan_filtered`, après retrait des secteurs où le lidar voit le robot. |
| `lidar_floor_filter` | `/scan_filtered`, `/imu` | `/scan_clean` pour le marquage, `/scan_clear` pour le raytracing, `/robot_tilt`, TF statique `base_footprint → base_footprint_level`. |
| `top_cam_to_scan` | `/top_camera_depth/points` (`PointCloud2`) | `/top_camera_scan` (`LaserScan`) dans `base_footprint_level`. |
| `bottom_cam_to_scan` | `/bottom_camera_depth/points` (`PointCloud2`) | `/bottom_camera_scan` (`LaserScan`) dans `base_footprint_level`. |
| `odom_imu_localizer` | `/odom`, `/imu`, spawn du YAML officiel | TF `map → odom` et `/localization_pose`. Il corrige le cap avec l'IMU, mais n'effectue pas de correction globale par carte : la position peut dériver. |
| Nav2 | `/map`, scans d'obstacles, TF | SmacPlanner2D, `controller_server`, behaviors, arbre borné, action `navigate_to_pose`; commandes remappées vers `/robot_base_controller/cmd_vel_unstamped`. |

## Repères et responsabilités TF

```text
map
 └─ map → odom : odom_imu_localizer (seul producteur dans le mode actuel)
     └─ odom → base_footprint : odométrie du robot / plugin de base
         ├─ lidar et caméras : TF du modèle officiel
         └─ base_footprint → base_footprint_level : lidar_floor_filter
```

Le localisateur personnalisé ne publie pas `odom → base_footprint`. Les anciens fichiers d'essai EKF indiquent que cette transformation est déjà publiée par le plugin DiffDrive, mais le paquet `parc_robot_bringup` et son URDF ne sont pas installés dans l'environnement inspecté. Ce point doit être confirmé sur l'installation officielle avant de rendre `amcl_ekf` disponible. Il ne faut jamais lancer deux producteurs pour la même arête TF.

Le scan de localisation retenu pour AMCL est `/scan_clean` : il conserve l'en-tête du scan d'origine, donc son frame lidar, et remplace les retours attribués au sol par `+inf`. `/scan_clear` est destiné au raytracing des costmaps et prolonge artificiellement la visibilité au sol ; il ne convient pas au modèle de mesure AMCL. La chaîne AMCL devra donc attendre la TF entre le frame réel indiqué dans cet en-tête et `base_footprint`.

## Cartes, contrôleurs et costmaps

- La carte `cafe_map.yaml` est en repère monde Gazebo, résolution 5 cm, origine `(-5.5, -11.8)`. Le dépôt vérifie déjà que le spawn et le but officiels sont dans sa zone libre.
- `task_solution.py` peut remplacer la carte par une carte vide quand son scan initial ne correspond pas à la carte du café. C'est un secours pour le mode actuel, mais c'est interdit avec AMCL : AMCL doit échouer clairement si une carte connue, valide et cohérente n'est pas disponible.
- Le planificateur est `nav2_smac_planner::SmacPlanner2D`.
- RPP est le contrôleur de référence ; DWB est une alternative déjà configurée. Les deux utilisent `FollowPath` et le même serveur de contrôleur.
- La costmap globale consomme le lidar et la caméra haute, plus la couche statique. La costmap locale consomme les trois scans. Chaque couche marque/efface selon les règles de son capteur et partage l'empreinte du robot.
- Les vitesses de référence sont RPP 0,5 m/s et 0,8 rad/s pour la rotation ; DWB a les mêmes limites maximales. Elles ne sont pas des limites validées pour le robot physique.

## Historique AMCL et limites connues

Le README de la version soumise rapporte qu'une carte SLAM antérieure avait son origine au spawn alors que `task_params.yaml` est exprimé dans le repère du monde Gazebo. Il rapporte aussi qu'AMCL ne convergait pas et que le watchdog signalait une localisation perdue après quelques mètres. Les essais historiques ont utilisé `/scan_filtered` et un ancien lancement AMCL séparé. Cela laisse trois causes plausibles à vérifier : carte et pose initiale exprimées dans des repères différents, scan encore affecté par les retours du sol, et seuil du watchdog/covariances inadaptés. Ces indices ne prouvent pas à eux seuls une cause unique.

Le mode AMCL du laboratoire doit donc utiliser la carte du café vérifiée, initialiser explicitement la pose depuis `task_params.yaml`, consommer `/scan_clean` et attendre une pose/TF disponible avant d'envoyer un but. Le mode `amcl_ekf` reste indisponible tant que le producteur `odom → base_footprint`, les covariances et les frames de l'IMU ne sont pas vérifiés.

## Scénarios présents dans le dépôt

Le dépôt contient la carte du café et le spawn/but officiels, pas de fixtures Gazebo distinctes pour virages, passages étroits, obstacles mobiles ou blocage temporaire. Le seul scénario activable initialement est donc `full_course`. Les autres apparaîtront comme indisponibles jusqu'à la création et à la validation de mondes ou fixtures correspondants.

## Références Jazzy

- [Configuration officielle AMCL Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)
- [Configuration officielle MPPI Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
- [Plugins de navigation Nav2](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)
