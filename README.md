# ÉQUIPE FOSA : PARC Engineers League 2026

## Introduction

La tâche de la phase 1 demande de faire naviguer de façon autonome le robot de service CAYTU Sito-É dans le restaurant d'un stade intelligent, de son point de départ jusqu'au cercle vert, en moins de 10 minutes et sans toucher d'obstacle. Un robot assistant de stade doit avant tout savoir se déplacer seul parmi les tables, les chaises et les visiteurs : c'est la base de tous les services qu'il rendra ensuite (guidage, information, sécurité). Notre solution s'appuie sur Nav2 et sur les trois capteurs de distance du robot : le lidar, bas et précis, et les deux caméras de profondeur, qui voient ce que le lidar ne voit pas (plateaux de table, dossiers de chaise, personnes). Nous avons privilégié une solution entièrement automatique et robuste : une seule commande, aucune intervention dans RViz, et un comportement qui reste correct si le point de départ, le but ou les obstacles changent.

**Pays de l'équipe :** Madagascar

**Membres de l'équipe :**

* À compléter — Nom et prénom (chef d'équipe)
* À compléter — Nom et prénom
* À compléter — Nom et prénom

## Dépendances

**Paquets nécessaires** (ROS 2 Jazzy, Gazebo Harmonic) :

* `navigation2` : pile Nav2 (map_server, planner Smac 2D, controller Regulated Pure Pursuit et DWB, behaviors, bt_navigator, costmap_2d, lifecycle_manager, nav2_msgs).

    * `$ sudo apt-get install ros-jazzy-navigation2`

* `laser_filters` : suppression des secteurs du lidar où le robot se voit lui-même.

    * `$ sudo apt-get install ros-jazzy-laser-filters`

* `pointcloud_to_laserscan` : conversion des nuages de points des deux caméras en scans 2D.

    * `$ sudo apt-get install ros-jazzy-pointcloud-to-laserscan`

* `PyYAML` : lecture de `task_params.yaml` et du fichier de carte.

    * `$ sudo apt-get install python3-yaml`

Les autres dépendances (`rclpy`, `tf2_ros`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `action_msgs`, `launch`, `launch_ros`) font partie de l'installation ROS 2 Jazzy de bureau. Toutes sont déclarées dans les `package.xml` ; on peut donc aussi les installer avec :

```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Compilation :

```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Tâche

Le nœud `task_solution.py` exécute toute la solution. Il lit le point de départ et le but dans `task_params.yaml` du paquet officiel, puis démarre lui-même la perception, la localisation et Nav2. La localisation n'utilise ni carte ni filtre à particules : le repère `map` est calé sur le repère de Gazebo à partir du point de départ, puis la position est suivie avec la distance parcourue par les roues et le cap de l'IMU. Le lidar est débarrassé des secteurs où il voit le robot et des rayons qui touchent le sol ; les nuages des deux caméras sont filtrés en hauteur dans un repère remis à l'horizontale, puis convertis en scans. Chaque capteur alimente sa propre couche de costmap, et une carte des murs du café, générée depuis le monde officiel, aide le planificateur (elle est écartée automatiquement si le lidar montre que le monde est différent). Le planificateur Smac 2D calcule le chemin, le contrôleur Regulated Pure Pursuit le suit avec replanification à 2 Hz, et le but est renvoyé après tout échec jusqu'à la limite de temps. À l'arrivée, le robot est arrêté au centre du cercle et tout ce qui a été lancé est fermé.

Commandes, dans deux terminaux :

```
ros2 launch parc_robot_bringup task.launch.py
```

```
ros2 run caytu_nav_solution task_solution.py
```

## Défis rencontrés

* **Repères différents.** Notre première carte, construite au SLAM, avait pour origine le point de départ du robot, alors que `task_params.yaml` est exprimé dans le repère de Gazebo. Le robot était donc initialisé hors de la carte et visait un mauvais endroit.
* **Localisation jamais convergée.** AMCL restait à sa covariance initiale puis dérivait à l'estime, ce qui déclenchait notre alarme « localisation perdue » après deux à trois mètres.
* **Robot incliné.** La roulette arrière descend 1 cm plus bas que les roues : le robot penche d'environ 2,5° vers l'avant, ce que la TF ne dit pas. Des rayons du lidar touchaient le sol et, pour les caméras, le sol semblait monter de 4 cm par mètre.
* **Bruit des caméras de profondeur.** Le simulateur ajoute 10 cm d'écart-type directement sur la hauteur des points. Avec un seuil trop bas, la caméra haute voyait un obstacle dans presque toutes les directions sur un sol vide.
* **Rayons vides marqués comme obstacles.** Avec `inf_is_valid`, un rayon sans obstacle devenait un point à portée maximale, lui-même marqué comme obstacle car `obstacle_max_range` avait la même valeur.
* **Odométrie en rotation.** L'entraxe déclaré au plugin d'odométrie (0,3409 m) est plus petit que l'entraxe réel des roues (0,393 m) ; nous prenons donc le cap sur l'IMU.
* **Simulation lente.** Sur nos machines, Gazebo tourne à environ 20 % du temps réel, ce qui rend chaque essai long et impose des délais d'attente généreux au démarrage de Nav2.
* **Contrôleur bloqué.** DWB oscillait sur place pendant près d'une minute dès qu'aucune trajectoire n'était valide ; nous sommes passés à Regulated Pure Pursuit, plus prévisible, en gardant DWB en option.
