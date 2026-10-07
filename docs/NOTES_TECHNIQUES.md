# Fosa — notes techniques de la solution de navigation

Document de travail de l'equipe. Le document de soumission est `README.md`.

## 1. Architecture

```
task_solution.py  (caytu_nav_solution)        seule commande a lancer
 ├─ lit task_params.yaml : spawn et but, repere Gazebo
 ├─ compare un scan lidar a la carte du cafe (carte vide si desaccord)
 ├─ lance caytu_nav_bringup/launch/solution_bringup.launch.py
 │    ├─ laser_filters            /scan -> /scan_filtered   (robot retire)
 │    ├─ lidar_floor_filter       /scan_filtered -> /scan_clean, /scan_clear
 │    │                           TF base_footprint -> base_footprint_level
 │    ├─ pointcloud_to_laserscan  x2 : /top_camera_scan, /bottom_camera_scan
 │    ├─ odom_imu_localizer       TF map -> odom  (roues + cap IMU + recalage murs)
 │    └─ navigation.launch.py     map_server, planner, controller,
 │                                behaviors, bt_navigator, collision_monitor (option),
 │                                lifecycle_manager
 ├─ envoie le but, le renvoie apres chaque echec jusqu'a la limite de temps
 ├─ ecrit run_*.json + runs.csv dans report_dir
 └─ arrete le robot et ferme tout ce qu'il a lance

nav_monitor (outil) : lit /plan, action NavigateToPose, cmd_vel, poses,
 scans, contacts, tilt, correction ; affiche [ETIQUETTE] texte + ligne toutes les 2 s.
run_benchmark.py (outil) : serie d'essais avec scenarios YAML, boites SDF,
 mesure reelle Gazebo, agregats CSV + MD.
```

| Fichier | Role |
|---|---|
| `caytu_nav_solution/task_solution.py` | point d'entree officiel + rapport |
| `caytu_nav_solution/odom_imu_localizer.py` | localisation roues + IMU + recalage |
| `caytu_nav_solution/lidar_floor_filter.py` | inclinaison, sol lidar, repere horizontal |
| `caytu_nav_solution/nav_math.py` | calculs purs (poses, fusion, sol, carte, champ, recalage) |
| `caytu_nav_solution/nav_events.py` | detecteur pur d'evenements (moniteur) |
| `caytu_nav_solution/nav_monitor.py` | noeud moniteur terminal |
| `caytu_nav_solution/run_report.py` | metriques O(1) et rapport JSON/CSV |
| `caytu_nav_solution/task_params.py` | lecture de `task_params.yaml` |
| `caytu_nav_bringup/config/nav2_params.yaml` | toute la configuration Nav2 + collision_monitor |
| `caytu_nav_bringup/behavior_trees/navigate_replan_if_invalid.xml` | arbre par defaut |
| `caytu_nav_bringup/behavior_trees/navigate_replan_periodic.xml` | arbre de secours |
| `caytu_nav_bringup/maps/cafe_map.*` | murs du cafe, repere Gazebo |
| `tools/run_benchmark.py` + `benchmark_scenarios.yaml` | essais en serie |

Le URDF et les paquets officiels PARC ne sont pas modifies. La pose reelle
`/sitoe_robot/pose` n'est lue que par `nav_monitor` et `run_benchmark.py`,
jamais par la solution.

## 2. Parametres de task_solution.py

`ros2 run caytu_nav_solution task_solution.py --ros-args -p nom:=valeur`

| Parametre | Defaut | Effet |
|---|---|---|
| `time_limit_sec` | `600.0` | limite de la tache, horloge du noeud |
| `stop_margin_sec` | `1.0` | marge avant arret propre |
| `success_radius` | `0.20` | rayon d'arrivee au centre du but |
| `arrival_recheck_radius` | `0.35` | reenvoi si Nav2 annonce trop loin |
| `retry_pause_sec` | `1.0` | pause entre tentatives |
| `nav2_startup_timeout_sec` | `180.0` | attente Nav2 (temps reel) |
| `launch_bringup` | `True` | lance le bringup, sinon suppose tourne |
| `bringup_package` | `caytu_nav_bringup` | paquet du launch |
| `bringup_launch` | `solution_bringup.launch.py` | launch complet |
| `behavior_tree` | `if_invalid` | `periodic` : secours a 2 Hz |
| `static_map` | `auto` | `always` imposee ; `never` vide |
| `map_yaml` | vide | carte (vide = cafe du bringup) |
| `map_check_timeout_sec` | `4.0` | attente scan de controle |
| `map_check_min_agreement` | `0.35` | seuil validation cafe |
| `task_params_file` | vide | autre spawn/but (essais) |
| `global_frame` | `map` | repere monde Gazebo |
| `base_frame` | `base_footprint` | base du robot |
| `scan_topic` | `/scan` | scan brut pour controle carte |
| `cmd_vel_topic` | `/robot_base_controller/cmd_vel_unstamped` | commande robot |
| `write_report` | `True` | ecriture du rapport |
| `report_dir` | `~/.ros/fosa_runs` | dossier JSON + CSV |
| `map_correction` | `auto` | `never` : jamais de recalage |
| `collision_monitor` | `False` | `True` : couche approche |

## 3. Outils

Moniteur (terminal separe, pendant la solution) :
```
ros2 run caytu_nav_solution nav_monitor --ros-args -p use_sim_time:=true
ros2 run caytu_nav_solution nav_monitor --ros-args -p log_file:=/tmp/nav.log
```
Ligne toutes les 2 s : pose, but a D m, ecart (ou n/d), v w, lidar top bottom,
sol_lidar, tangage (mesure ou defaut), replans, contacts.

Rapport : `report_dir/run_AAAAMMJJ_HHMMSS.json` + `runs.csv` (6 lignes loguees).

Benchmark :
```
python3 tools/run_benchmark.py --runs 2 --scenarios officiel,deux_boites --out /tmp/bench
```
Sorties `benchmark_results.csv` + `benchmark_results.md` (moyennes en une passe).

## 4. Avant la soumission

- [ ] Adresse mainteneur a remplacer dans les deux `package.xml`.
- [ ] Noms des membres dans `README.md` (deux langues).
- [ ] `colcon build` + `colcon test` : 0 echec.
- [ ] Commande unique sur machine propre avec seules dependances du README.
- [ ] `grep -rniE "gmail|trinome|/home/"` : seul le test d'absence repond.
- [ ] Aucune coordonnee en dur hors `task_params.yaml`.
- [ ] Video de moins de 200 Mo, zip sans `build/`, `install/`, `log/`, `.git/`.
