# Fosa — notes techniques de la solution de navigation

Document de travail de l'équipe. Le document de soumission est `README.md`.

## 1. Architecture

```
task_solution.py  (caytu_nav_solution)        seule commande à lancer
 ├─ lit task_params.yaml : spawn et but, repère Gazebo
 ├─ compare un scan lidar à la carte du café (carte vide si désaccord)
 ├─ lance caytu_nav_bringup/launch/solution_bringup.launch.py
 │    ├─ laser_filters            /scan -> /scan_filtered   (robot retiré)
 │    ├─ lidar_floor_filter       /scan_filtered -> /scan_clean, /scan_clear
 │    │                           TF base_footprint -> base_footprint_level
 │    ├─ pointcloud_to_laserscan  x2 : /top_camera_scan, /bottom_camera_scan
 │    ├─ odom_imu_localizer       TF map -> odom  (roues + cap IMU)
 │    └─ navigation.launch.py     map_server, planner, controller,
 │                                behaviors, bt_navigator, lifecycle_manager
 ├─ envoie le but, le renvoie après chaque échec jusqu'à la limite de temps
 └─ arrête le robot et ferme tout ce qu'il a lancé
```

| Fichier | Rôle |
|---|---|
| `caytu_nav_solution/task_solution.py` | point d'entrée officiel |
| `caytu_nav_solution/odom_imu_localizer.py` | localisation sans carte |
| `caytu_nav_solution/lidar_floor_filter.py` | inclinaison du robot, sol du lidar, repère horizontal |
| `caytu_nav_solution/nav_math.py` | calculs purs, testés sans ROS |
| `caytu_nav_solution/task_params.py` | lecture de `task_params.yaml` |
| `caytu_nav_bringup/config/nav2_params.yaml` | toute la configuration Nav2 |
| `caytu_nav_bringup/behavior_trees/navigate_replan_if_invalid.xml` | arbre par défaut |
| `caytu_nav_bringup/behavior_trees/navigate_replan_periodic.xml` | arbre de secours |
| `caytu_nav_bringup/maps/cafe_map.*` | murs du café, repère Gazebo |
| `caytu_nav_bringup/tools/generate_world_map.py` | génération de la carte (développement) |
| `tools/diag_nav.py` | diagnostic pendant un essai (développement) |

Le URDF et les paquets officiels PARC ne sont pas modifiés. La pose réelle
`/sitoe_robot/pose` n'est lue que par l'outil de diagnostic, jamais par la
solution.

## 2. Choix techniques

- **Repère `map` = repère Gazebo.** L'odométrie démarre à zéro au spawn ; la
  localisation publie donc `map -> odom` à partir du spawn de
  `task_params.yaml`. Le but du même fichier s'envoie tel quel.
- **Cap pris sur l'IMU.** L'entraxe déclaré au plugin d'odométrie (0,3409 m)
  est plus petit que l'entraxe réel (0,393 m).
- **Robot incliné de 2,5°.** La roulette arrière est 1 cm plus basse que les
  roues. Les nuages des caméras sont filtrés dans `base_footprint_level`, et
  les rayons du lidar qui touchent le sol sont retirés.
- **Caméra haute à `min_height` 0,60 m.** Le bruit de profondeur de Gazebo
  (0,10 m) s'applique aussi à la hauteur des points.
- **Une couche de costmap par capteur**, combinées en maximum.
  `obstacle_max_range` reste inférieur à la portée du scan caméra.
- **Replanification seulement si le chemin devient invalide.** Le robot garde
  son chemin tant qu'aucun obstacle ne le coupe.
- **Tolérance d'arrivée de 0,10 m** autour du centre du but, cap libre.

## 3. Paramètres utiles

Paramètres de `task_solution.py` :
`ros2 run caytu_nav_solution task_solution.py --ros-args -p nom:=valeur`

| Paramètre | Défaut | Effet |
|---|---|---|
| `behavior_tree` | `if_invalid` | `periodic` : replanification à 2 Hz (arbre de secours) |
| `static_map` | `auto` | `always` : carte imposée ; `never` : carte vide |
| `time_limit_sec` | `600.0` | limite de la tâche, en temps simulé |
| `task_params_file` | vide | autre fichier de spawn et de but (essais) |

Réglages dans `nav2_params.yaml` :

| Besoin | Paramètre |
|---|---|
| Vitesse | `FollowPath.desired_linear_vel` (0.5 ; monter par paliers de 0.1) |
| Pivots sur place | `FollowPath.rotate_to_heading_min_angle` (0.785) |
| Arrêts « collision ahead » trop fréquents | `FollowPath.max_allowed_time_to_collision_up_to_carrot` (1.0) |
| Marge autour des obstacles | `inflation_radius` et `cost_travel_multiplier` |

Réglages dans `solution_bringup.launch.py` :

| Besoin | Paramètre |
|---|---|
| Faux obstacles de la caméra haute sur sol vide | `min_height` (0.60, maximum 0.72) |
| Lidar qui marque encore le sol | argument `floor_line_distance` (distance lue droit devant dans `/scan`) |

## 4. Diagnostic pendant un essai

```
python3 src/fosa-solution/tools/diag_nav.py --ros-args -p use_sim_time:=true | tee diag.log
```

| Colonne | Valeur attendue |
|---|---|
| `ecart` | sous 0,15 m (affiche « n/d » si `/sitoe_robot/pose` ne publie pas) |
| `top` | 0 à 2 sur sol dégagé |
| `sol_lidar` | stable |
| `tangage` | 2 à 3,5°, suivi de « mesuré » |
| `v` | proche de 0,50 en ligne droite |
| `contacts` | 0 |

Messages attendus de `task_solution.py`, dans l'ordre :
`Carte du café validée`, `Inclinaison mesurée`, `Nav2 prêt et robot localisé`,
`Envoi du but`, `BUT ATTEINT`.

## 5. Tests

```
cd ~/ros2_ws
colcon test --packages-select caytu_nav_bringup caytu_nav_solution
colcon test-result --verbose
```

- `caytu_nav_solution/test` : calculs (poses, fusion roues + IMU, sol du lidar, carte).
- `caytu_nav_bringup/test` : garde-fous sur la configuration, les arbres et la carte.

## 6. Avant la soumission

- [ ] Remplacer l'adresse du mainteneur dans les deux `package.xml`.
- [ ] Compléter les noms des membres dans `README.md` (deux langues).
- [ ] 5 essais réussis d'affilée avec la configuration officielle.
- [ ] 5 essais avec un autre spawn et un autre but dans `task_params.yaml`.
- [ ] 5 essais avec des obstacles ajoutés ; `contacts` à 0 dans `diag.log`.
- [ ] Essai de la commande unique sur une machine propre, avec les seules
      dépendances du README.
- [ ] Vidéo de démonstration de moins de 200 Mo.
- [ ] Zip du dossier, sans `build/`, `install/`, `log/` ni `.git/`.
- [ ] Question posée aux organisateurs : limite de 10 minutes en temps simulé
      ou réel ; usage d'outils d'aide à l'écriture du code.
