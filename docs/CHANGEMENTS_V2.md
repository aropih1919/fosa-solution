# Fosa — version 2 de la solution de navigation

Ce document accompagne la refonte du 5 octobre 2026. Il dit comment installer la
version 2, ce qui a changé, comment valider le premier essai dans Gazebo et quoi
régler si un point ne se passe pas comme prévu.

## 1. Installation

La version 2 supprime des fichiers. Ne la copiez pas par-dessus l'ancienne :
remplacez le dossier.

```
cd ~/ros2_ws/src
mv fosa-solution fosa-solution-v1          # sauvegarde, hors de src de préférence
unzip fosa-solution-v2.zip
cd ~/ros2_ws
rm -rf build/caytu_nav_bringup build/caytu_nav_solution \
       install/caytu_nav_bringup install/caytu_nav_solution
colcon build --symlink-install
source install/setup.bash
```

Si `fosa-solution-v1` reste dans `src`, colcon verra deux fois les mêmes paquets :
déplacez-le ailleurs ou ajoutez-y un fichier vide `COLCON_IGNORE`.

Lancement (deux terminaux) :

```
ros2 launch parc_robot_bringup task.launch.py
ros2 run caytu_nav_solution task_solution.py
```

## 2. Ce qui a changé

| Sujet | Version 1 | Version 2 |
|---|---|---|
| Lancement | launch du bringup, puis `task_solution` | une seule commande, `task_solution.py`, qui lance tout |
| Localisation | carte SLAM + AMCL + watchdog de covariance | `odom_imu_localizer` : spawn de `task_params.yaml`, distance des roues, cap de l'IMU |
| Repère `map` | repère de la carte SLAM | repère du monde Gazebo |
| Carte | `stadium_map` (SLAM, murs dédoublés) | `cafe_map`, générée depuis le monde officiel, vérifiée par le lidar au démarrage |
| Lidar | `/scan_filtered` | `/scan_clean` (marquage) et `/scan_clear` (effacement), sans les retours du sol |
| Caméra haute | `min_height` 0.30 dans `base_footprint` | `min_height` 0.60 dans `base_footprint_level` (repère horizontal) |
| Costmaps | `obstacle_max_range` = portée du scan caméra | strictement inférieur (plus de point fantôme à 3 m) |
| Inflation | 0.30 m (global), 0.25 m (local) | 0.90 m (global), 0.55 m (local) |
| Contrôleur | DWB | Regulated Pure Pursuit par défaut, DWB en option |
| Tolérance d'arrivée | 0.20 m, cap imposé | 0.10 m du centre du but, cap libre |
| Échec de Nav2 | arrêt du programme | but renvoyé jusqu'à la limite de temps |
| Arrêt | robot laissé avec sa dernière vitesse sur Ctrl-C | vitesse nulle envoyée, tous les processus fermés |

Fichiers supprimés : `amcl_params.yaml`, `slam_toolbox_params.yaml`,
`costmap_common_params.yaml`, `global_costmap_params.yaml`,
`local_costmap_params.yaml`, `controller_server_params.yaml`,
`planner_server_params.yaml`, `maps/stadium_map*`, `localization_watchdog.py`,
`goal_locator.py`, `nav2_client.py`.

Fichiers ajoutés : `nav_math.py`, `task_params.py`, `odom_imu_localizer.py`,
`lidar_floor_filter.py`, `controller_rpp.yaml`, `controller_dwb.yaml`,
`maps/cafe_map.*`, `tools/generate_world_map.py`, `tools/diag_nav.py`, tests.

Le URDF et les paquets officiels PARC ne sont pas modifiés.

## 3. Ce qui a été vérifié, et ce qui ne l'a pas été

Vérifié hors Gazebo :

- 13 tests unitaires des calculs et 13 tests de configuration (`pytest`).
- Compilation `colcon` des deux paquets sous ROS 2 Jazzy.
- Exécution réelle de `task_solution.py`, des fichiers de lancement et des deux
  nœuds face à un simulateur simplifié qui reproduit les défauts du robot (cap
  d'odométrie faussé de 15 %, robot incliné, lidar touchant le sol) : but
  atteint, écart de localisation final de 1 à 2 mm, reprise après refus et
  échecs de Nav2, limite de temps, autre spawn et autre but, monde différent du
  café, Ctrl-C.
- Noms des paramètres Nav2 contrôlés dans les sources Nav2 1.3.13 (Jazzy).
- Carte : 99,7 % des retours d'un lidar simulé dans le monde officiel tombent
  sur ses murs.
- Chaîne capteurs → costmap → planification rejouée dans le monde officiel avec
  le bruit des caméras : but atteint dans 18 essais sur 18 avec le réglage
  retenu, marge réelle minimale aux obstacles de 0,26 à 0,45 m.

Non vérifié : le comportement dans Gazebo avec le vrai Nav2. Nav2 et Gazebo
n'étaient pas disponibles dans l'environnement de développement de cette
version. Le premier essai ci-dessous est donc une étape de validation à part
entière.

## 4. Premier essai : liste de contrôle

Dans un troisième terminal :

```
python3 src/fosa-solution/tools/diag_nav.py --ros-args -p use_sim_time:=true | tee diag.log
```

Messages attendus dans le terminal de `task_solution.py`, dans l'ordre :

1. `Départ (...) -> but (...)` : les valeurs de `task_params.yaml`.
2. `Carte du café validée : NN % ...` : attendu entre 50 et 95 %.
3. `Inclinaison mesurée : tangage 2.x°` : attendu entre 2 et 3,5°. La même ligne
   donne la distance de la ligne de sol du lidar, ou indique qu'aucun rayon ne
   touche le sol.
4. `Nav2 prêt et robot localisé`.
5. `Envoi du but, tentative 1`, puis `En route : X m restants` toutes les 5 s.
6. `BUT ATTEINT en X s ... centre du robot à 0.0X m du centre du but`.

Dans `diag.log`, sur le trajet :

| Colonne | Valeur attendue | Si ce n'est pas le cas |
|---|---|---|
| `ecart` | sous 0,15 m | voir 5.1 |
| `top` | 0 à 2 sur sol dégagé | voir 5.2 |
| `sol_lidar` | stable (0 ou quelques dizaines) | voir 5.3 |
| `v` | proche de 0,50 en ligne droite | voir 5.4 |
| `contacts` | 0 | voir 5.5 |

## 5. Réglages

Tous les paramètres de `task_solution.py` se passent ainsi :
`ros2 run caytu_nav_solution task_solution.py --ros-args -p nom:=valeur`.

### 5.1 L'écart de localisation grandit

- En virage surtout : l'IMU n'est pas utilisée. Cherchez `Cap fourni par l'IMU`
  dans les messages de `odom_imu_localizer`.
- En ligne droite : le rayon de roue de l'odométrie est en cause, ce qui est peu
  probable en simulation. Relevez l'écart en fin de trajet et signalez-le.
- Pour comparer avec l'odométrie seule : dans `solution_bringup.launch.py`,
  ajoutez `"use_imu_yaw": False` aux paramètres de `odom_imu_localizer`.

### 5.2 La caméra haute voit encore des obstacles sur sol vide

Dans `solution_bringup.launch.py`, montez `min_height` de la caméra haute de
0.60 à 0.70. Ne dépassez pas 0.72 : les plateaux de table sont à 0.74 m.

### 5.3 Le lidar marque encore le sol

Mesurez dans `/scan` la distance lue droit devant sur sol dégagé, puis imposez-la :
ajoutez `floor_line_distance:=<distance>` à la commande `ros2 launch` de mise au
point, ou changez la valeur par défaut de cet argument dans
`solution_bringup.launch.py`.

### 5.4 Le robot est lent ou hésite

- Vitesse : `desired_linear_vel` dans `controller_rpp.yaml` (0.5 par défaut ; le
  plugin officiel accepte davantage, à monter par paliers de 0.1).
- Arrêts fréquents avec `collision ahead` dans les journaux : baissez
  `max_allowed_time_to_collision_up_to_carrot` à 0.7.
- Pour revenir à DWB : `-p controller:=dwb`.

### 5.5 Le robot touche un obstacle

- Augmentez `inflation_radius` de la costmap locale (0.55) et
  `cost_travel_multiplier` du planificateur (3.0).
- Vérifiez dans RViz que l'obstacle touché apparaît dans la costmap avant le
  contact. S'il n'apparaît pas, notez sa hauteur : sous 0,60 m, seuls le lidar
  et la caméra basse peuvent le voir.

### 5.6 Carte des murs

- `-p static_map:=never` : navigation sans carte (carte vide).
- `-p static_map:=always` : carte imposée sans vérification.
- Pour régénérer la carte après une mise à jour du monde officiel :
  `python3 caytu_nav_bringup/tools/generate_world_map.py <chemin>/parc_robot_bringup caytu_nav_bringup/maps`
  (outil de développement : `pip install trimesh pycollada pillow numpy`).

## 6. Points à confirmer auprès des organisateurs

- La limite de 10 minutes est-elle comptée en temps simulé ou en temps réel ?
  La solution compte en temps simulé (`time_limit_sec`, 600 par défaut).
- La pose réelle `/sitoe_robot/pose` n'est utilisée que par l'outil de
  diagnostic, jamais par la solution.
