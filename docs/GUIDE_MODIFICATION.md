# Guide de modification de la solution

Ce guide s'adresse à celui qui doit **modifier** la solution pendant ou après la
compétition, sans connaître tout ROS 2. Il indique où se trouve chaque réglage et
ce qu'il faut vérifier après une modification.

La règle de nommage du code : les **actions propres à notre solution peuvent
être nommées en français** (`charger_carte`, `recevoir_scan`), tandis que les
**termes techniques gardent leur nom anglais** (`Pose2D`, `TaskParams`, `scan`,
`yaw`, `Nav2`). Les noms des topics, repères, paramètres ROS et fichiers
officiels ne changent jamais.

---

## 1. Carte des fichiers

| Fichier | Rôle | On y touche pour... |
|---|---|---|
| `caytu_nav_solution/caytu_nav_solution/task_solution.py` | point d'entrée officiel : lit le départ/but, choisit la carte, lance le bringup, envoie le but, gère les échecs et l'arrêt | limites de temps, rayons d'arrivée, pauses entre tentatives |
| `caytu_nav_solution/caytu_nav_solution/nav_math.py` | calculs purs, sans ROS : poses, fusion roues + IMU, sol du lidar, carte PGM | formules mathématiques (rarement) |
| `caytu_nav_solution/caytu_nav_solution/task_params.py` | lit `task_params.yaml` (départ et but) | rien : le fichier vient du paquet officiel |
| `caytu_nav_solution/caytu_nav_solution/lidar_floor_filter.py` | retire du lidar les rayons qui touchent le sol et publie le repère horizontal | seuils du sol, géométrie du lidar |
| `caytu_nav_solution/caytu_nav_solution/odom_imu_localizer.py` | remplace AMCL : publie `map -> odom` avec les roues et l'IMU | choix IMU / odométrie |
| `caytu_nav_bringup/launch/solution_bringup.launch.py` | démarre les capteurs, la localisation et Nav2 | seuils des caméras, topics, repère horizontal |
| `caytu_nav_bringup/launch/navigation.launch.py` | démarre les nœuds Nav2 | rien au quotidien |
| `caytu_nav_bringup/config/nav2_params.yaml` | réglages Nav2 : costmaps, planificateur, comportements | marges de sécurité, vitesse du planificateur |
| `caytu_nav_bringup/config/controller_rpp.yaml` | contrôleur par défaut (Regulated Pure Pursuit) | **vitesse**, lookahead, ralentissements |
| `caytu_nav_bringup/config/controller_dwb.yaml` | contrôleur de secours (DWB) | seulement si on repasse à DWB |
| `caytu_nav_bringup/maps/cafe_map.*` | carte des murs du café officiel | régénération après un nouveau monde |
| `caytu_nav_bringup/tools/generate_world_map.py` | régénère la carte depuis le monde Gazebo | outil de développement |

Les tests se lancent sans Gazebo :

```
cd ~/ros2_ws
colcon build --symlink-install --packages-select caytu_nav_bringup caytu_nav_solution
source install/setup.bash
colcon test --packages-select caytu_nav_bringup caytu_nav_solution
colcon test-result --verbose
```

---

## 2. Changer le départ ou le but

**Ne modifiez rien dans le code.** Le départ et le but viennent du fichier
officiel `task_params.yaml` du paquet `parc_robot_bringup` :

```
$(ros2 pkg prefix parc_robot_bringup)/share/parc_robot_bringup/config/task_params.yaml
```

```yaml
/**:
  ros__parameters:
    x: -0.200546      # départ, repère Gazebo
    y: -7.485170
    z: 0.220794
    yaw: 1.571        # cap de départ (rad)
    goal_x: -2.293350 # centre du cercle vert
    goal_y: 2.232090
    goal_z: 0.220042
```

La solution relit ce fichier à chaque lancement : si le jury change le départ ou
le but, elle suit automatiquement. Pour tester un autre fichier sans toucher à
l'officiel :

```
ros2 run caytu_nav_solution task_solution.py \
  --ros-args -p task_params_file:=/chemin/vers/mon_task_params.yaml
```

---

## 3. Changer la vitesse du robot

Fichier : `caytu_nav_bringup/config/controller_rpp.yaml`

```yaml
desired_linear_vel: 0.5            # vitesse maximale en ligne droite (m/s)
rotate_to_heading_angular_vel: 0.8 # vitesse de rotation sur place (rad/s)
max_angular_accel: 1.6
```

Le plugin DiffDrive officiel accepte jusqu'à **1,5 m/s** et **1 rad/s**. Montez
par paliers de 0,1 m/s et refaites un essai : au-delà de 0,7 m/s, le robot
commence à osciller dans les virages serrés du café.

Pour changer de contrôleur sans éditer de fichier :

```
ros2 run caytu_nav_solution task_solution.py --ros-args -p controller:=dwb
```

---

## 4. Changer les tolérances d'arrivée et les tentatives

Fichier : `caytu_nav_solution/caytu_nav_solution/task_solution.py`, méthode
`_declarer_parametres` (les valeurs par défaut sont aussi modifiables en ligne de
commande).

| Paramètre | Défaut | Effet |
|---|---|---|
| `time_limit_sec` | `600.0` | durée officielle de la tâche (temps simulé) |
| `stop_margin_sec` | `1.0` | marge gardée pour s'arrêter proprement |
| `success_radius` | `0.20` | distance au centre du but considérée comme arrivée (m) |
| `arrival_recheck_radius` | `0.35` | si Nav2 annonce l'arrivée au-delà, le but est renvoyé |
| `retry_pause_sec` | `1.0` | pause entre deux tentatives |
| `nav2_startup_timeout_sec` | `180.0` | attente maximale de Nav2 (temps réel) |

Exemple : être plus indulgent sur l'arrivée et réessayer plus vite.

```
ros2 run caytu_nav_solution task_solution.py \
  --ros-args -p success_radius:=0.30 -p retry_pause_sec:=0.5
```

Le cercle vert officiel a un rayon de 0,6 m : ne dépassez pas
`success_radius` = 0,35 m pour rester crédible.

---

## 5. Changer la carte des murs

Trois modes, réglables par le paramètre `static_map` :

| Valeur | Comportement |
|---|---|
| `auto` (défaut) | la carte du café est utilisée seulement si le lidar confirme que le monde y ressemble |
| `always` | la carte est imposée sans vérification |
| `never` | navigation sans carte statique (une carte vide est fabriquée) |

```
ros2 run caytu_nav_solution task_solution.py --ros-args -p static_map:=never
```

Le seuil de vérification est `map_check_min_agreement` (0,35 par défaut) : en
dessous, la solution considère que le monde n'est pas le café et navigue sans
carte. Pour régénérer la carte après une mise à jour du monde officiel :

```
python3 caytu_nav_bringup/tools/generate_world_map.py \
  $(ros2 pkg prefix parc_robot_bringup)/share/parc_robot_bringup \
  caytu_nav_bringup/maps
```

---

## 6. Changer les seuils des capteurs

### Lidar et sol

Fichier : `caytu_nav_solution/caytu_nav_solution/lidar_floor_filter.py`, méthode
`_declarer_parametres`.

| Paramètre | Défaut | Effet |
|---|---|---|
| `floor_ratio` | `0.80` | un retour à plus de 80 % de la distance théorique du sol est du sol |
| `clear_ratio` | `0.70` | distance « libre » publiée pour effacer la costmap |
| `tilt_samples` | `150` | mesures IMU immobiles avant de figer l'inclinaison |
| `default_robot_pitch` | `0.0429` | inclinaison par défaut (rad) avant la mesure IMU |
| `lidar_mount_pitch` | `-0.0436` | inclinaison du lidar dans le robot (rad) |
| `lidar_mount_z` | `0.0481` | hauteur du lidar (m) |
| `lidar_lever_x` | `0.116` | distance axe des roues → lidar (m) |

Si le lidar marque encore le sol, mesurez dans `/scan` la distance lue droit
devant sur sol dégagé, puis imposez-la au lancement :

```
ros2 launch caytu_nav_bringup solution_bringup.launch.py floor_line_distance:=1.20
```

### Caméras

Fichier : `caytu_nav_bringup/launch/solution_bringup.launch.py`, fonctions
`_creer_camera_haute` et `_creer_camera_basse`.

| Réglage | Caméra haute | Caméra basse |
|---|---|---|
| `min_height` | `0.60` | `0.25` |
| `max_height` | `1.40` | `1.40` |
| `range_max` | `3.0` | `1.2` |
| `angle_min` / `angle_max` | `-0.785` / `0.785` | idem |

Ne descendez pas `min_height` de la caméra haute sous 0,60 m : le bruit de
profondeur de Gazebo est de 0,10 m et ferait apparaître le sol comme un obstacle.
Ne montez pas non plus au-dessus de 0,72 m : les plateaux de table sont à 0,74 m.

---

## 7. Changer les marges de sécurité Nav2

Fichier : `caytu_nav_bringup/config/nav2_params.yaml`

```yaml
global_costmap:
  inflation_radius: 0.90    # marge autour des murs (planification)
  cost_scaling_factor: 3.0

local_costmap:
  inflation_radius: 0.55    # marge autour des obstacles (évitement)
  cost_scaling_factor: 4.0
```

- Le robot touche un obstacle → augmentez `inflation_radius` de la costmap
  locale par petits pas (0,05 m).
- Le robot ne passe plus entre les tables → diminuez-la, ou vérifiez que
  l'obstacle est bien vu par un capteur (hauteur sous 0,60 m : lidar et caméra
  basse seulement).
- `cost_scaling_factor` doit rester égal à `inflation_cost_scaling_factor` du
  contrôleur (`controller_rpp.yaml`).

---

## 8. Ce qu'il ne faut pas changer

- **Les noms des topics et des repères** : `/scan`, `/scan_clean`, `/scan_clear`,
  `/robot_base_controller/cmd_vel_unstamped`, `map`, `odom`, `base_footprint`,
  `base_footprint_level`. Les paquets officiels et Nav2 les attendent tels quels.
- **Les noms des paramètres ROS** (`time_limit_sec`, `static_map`,
  `floor_line_distance`...) : le jury et les fichiers de configuration les
  utilisent.
- **Le point d'entrée** `ros2 run caytu_nav_solution task_solution.py` : c'est la
  commande d'évaluation.
- **L'ordre d'arrêt** dans `sequence_arret` : annuler le but, vitesse nulle,
  arrêter le bringup, vitesse nulle. Le supprimer laisserait le robot avancer
  après la fin du programme.

---

## 9. Après chaque modification

1. Compiler : `colcon build --symlink-install --packages-select caytu_nav_bringup caytu_nav_solution`
2. Lancer les tests : `colcon test --packages-select caytu_nav_bringup caytu_nav_solution`
3. Vérifier les arguments des launch :
   `ros2 launch caytu_nav_bringup solution_bringup.launch.py --show-args`
4. Faire un essai complet dans Gazebo :

```
ros2 launch parc_robot_bringup task.launch.py
ros2 run caytu_nav_solution task_solution.py
```

5. En cas de doute, suivre le diagnostic :

```
python3 tools/diag_nav.py --ros-args -p use_sim_time:=true | tee diag.log
```

Les colonnes `ecart`, `top`, `sol_lidar`, `v` et `contacts` sont expliquées dans
`docs/CHANGEMENTS_V2.md`, section 4.
