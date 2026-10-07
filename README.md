# TEAM FOSA: PARC Engineers League 2026

*Version française plus bas.*

## Introduction

Phase 1 asks teams to drive the CAYTU Sito-É service robot autonomously through the restaurant of a smart stadium, from its start position to the green circle, in under 10 minutes and without touching any obstacle. A stadium assistant robot must first be able to move on its own between tables, chairs and visitors: every other service it will offer (guidance, information, safety) depends on it. Our solution is built on Nav2 and on the robot's three range sensors: the lidar, low and precise, and the two depth cameras, which see what the lidar cannot (table tops, chair backs, people). We aimed for a fully automatic and robust solution: one command, no action in RViz, and correct behaviour when the start pose, the goal or the obstacles change.

**Team Country:** Madagascar

**Team Member Names:**

* To be completed — First and Last Name (Team Leader)
* To be completed — First and Last Name
* To be completed — First and Last Name

## Dependencies

**Packages needed** (ROS 2 Jazzy, Gazebo Harmonic):

* `navigation2`: Nav2 stack (map server, Smac 2D planner, Regulated Pure Pursuit controller, behaviors, BT navigator, costmaps, lifecycle manager, messages).

    * `$ sudo apt-get install ros-jazzy-navigation2`

* `laser_filters`: removes the lidar sectors where the robot sees itself.

    * `$ sudo apt-get install ros-jazzy-laser-filters`

* `pointcloud_to_laserscan`: converts the two depth-camera point clouds into 2D scans.

    * `$ sudo apt-get install ros-jazzy-pointcloud-to-laserscan`

* `PyYAML`: reads `task_params.yaml` and the map file.

    * `$ sudo apt-get install python3-yaml`

All dependencies are declared in the `package.xml` files, so they can also be installed with `rosdep install --from-paths src --ignore-src -r -y`, then built with `colcon build`.

## Task

The `task_solution.py` node runs the whole solution. It reads the start pose and the goal from `task_params.yaml` in the official package, then starts perception, localization and Nav2 by itself. Localization uses neither a SLAM map nor a particle filter: the `map` frame is aligned with the Gazebo world frame from the start pose, then the position is tracked with wheel distance and IMU heading. The lidar scan is cleaned of the sectors where it sees the robot and of the beams that hit the floor; the two camera point clouds are height-filtered in a levelled frame and converted to scans. Each sensor feeds its own costmap layer, and a wall map of the café, generated from the official world, helps the planner (it is dropped automatically if the lidar shows a different world). The Smac 2D planner computes the path, the Regulated Pure Pursuit controller follows it, and the path is replanned only when an obstacle makes it invalid. The goal is sent again after any failure until the time limit; on arrival the robot is stopped at the centre of the circle and everything that was started is shut down.

Commands, in two terminals:

```
ros2 launch parc_robot_bringup task.launch.py
```

```
ros2 run caytu_nav_solution task_solution.py
```

## Challenges Faced

* **Different frames.** Our first map, built with SLAM, had its origin at the robot start pose, while `task_params.yaml` is expressed in the Gazebo frame. The robot was initialised outside the map and aimed at the wrong place.
* **Localization that never converged.** AMCL stayed at its initial covariance and drifted on dead reckoning, which triggered our "localization lost" alarm after two to three metres.
* **Tilted robot.** The rear caster sits 1 cm lower than the wheels: the robot leans about 2.5° forward, which the TF tree does not show. Lidar beams hit the floor and, for the cameras, the floor seemed to rise by 4 cm per metre.
* **Depth-camera noise.** The simulator adds 10 cm of standard deviation directly to point heights. With a threshold set too low, the top camera saw an obstacle in almost every direction on an empty floor.
* **Empty rays marked as obstacles.** With `inf_is_valid`, a ray with no obstacle became a point at maximum range, itself marked as an obstacle because `obstacle_max_range` had the same value.
* **Rotation odometry.** The wheel separation declared to the odometry plugin (0.3409 m) is smaller than the real one (0.393 m), so we take the heading from the IMU.
* **Slow simulation.** On our machines Gazebo runs at about 20 % of real time, which makes every test long and requires generous start-up timeouts.
* **Hesitation between two paths.** Facing an obstacle that can be passed on either side, periodic replanning made the path flip from one side to the other. We now keep the current path as long as it stays valid.

--------------------------------------------------

# ÉQUIPE FOSA : PARC Ligue des Ingénieurs 2026

## Introduction

La phase 1 demande de faire naviguer de façon autonome le robot de service CAYTU Sito-É dans le restaurant d'un stade intelligent, de son point de départ jusqu'au cercle vert, en moins de 10 minutes et sans toucher d'obstacle. Un robot assistant de stade doit avant tout savoir se déplacer seul parmi les tables, les chaises et les visiteurs : tous les services qu'il rendra ensuite (guidage, information, sécurité) en dépendent. Notre solution s'appuie sur Nav2 et sur les trois capteurs de distance du robot : le lidar, bas et précis, et les deux caméras de profondeur, qui voient ce que le lidar ne voit pas (plateaux de table, dossiers de chaise, personnes). Nous avons visé une solution entièrement automatique et robuste : une seule commande, aucune action dans RViz, et un comportement correct si le point de départ, le but ou les obstacles changent.

**Pays de l'équipe :** Madagascar

**Noms des membres de l'équipe :**

* À compléter — Nom et prénom (chef d'équipe)
* À compléter — Nom et prénom
* À compléter — Nom et prénom

## Dépendances

**Paquets nécessaires** (ROS 2 Jazzy, Gazebo Harmonic) :

* `navigation2` : pile Nav2 (serveur de carte, planificateur Smac 2D, contrôleur Regulated Pure Pursuit, behaviors, BT navigator, costmaps, lifecycle manager, messages).

    * `$ sudo apt-get install ros-jazzy-navigation2`

* `laser_filters` : supprime les secteurs du lidar où le robot se voit lui-même.

    * `$ sudo apt-get install ros-jazzy-laser-filters`

* `pointcloud_to_laserscan` : convertit les nuages de points des deux caméras en scans 2D.

    * `$ sudo apt-get install ros-jazzy-pointcloud-to-laserscan`

* `PyYAML` : lit `task_params.yaml` et le fichier de carte.

    * `$ sudo apt-get install python3-yaml`

Toutes les dépendances sont déclarées dans les `package.xml` ; on peut donc aussi les installer avec `rosdep install --from-paths src --ignore-src -r -y`, puis compiler avec `colcon build`.

## Tâche

Le nœud `task_solution.py` exécute toute la solution. Il lit le point de départ et le but dans `task_params.yaml` du paquet officiel, puis démarre lui-même la perception, la localisation et Nav2. La localisation n'utilise ni carte SLAM ni filtre à particules : le repère `map` est calé sur le repère de Gazebo à partir du point de départ, puis la position est suivie avec la distance des roues et le cap de l'IMU. Le scan du lidar est débarrassé des secteurs où il voit le robot et des rayons qui touchent le sol ; les nuages des deux caméras sont filtrés en hauteur dans un repère remis à l'horizontale, puis convertis en scans. Chaque capteur alimente sa propre couche de costmap, et une carte des murs du café, générée depuis le monde officiel, aide le planificateur (elle est écartée automatiquement si le lidar montre un autre monde). Le planificateur Smac 2D calcule le chemin, le contrôleur Regulated Pure Pursuit le suit, et le chemin n'est recalculé que si un obstacle le rend invalide. Le but est renvoyé après tout échec jusqu'à la limite de temps ; à l'arrivée, le robot est arrêté au centre du cercle et tout ce qui a été lancé est fermé.

Commandes, dans deux terminaux :

```
ros2 launch parc_robot_bringup task.launch.py
```

```
ros2 run caytu_nav_solution task_solution.py
```

## Défis rencontrés

* **Repères différents.** Notre première carte, construite au SLAM, avait pour origine le point de départ du robot, alors que `task_params.yaml` est exprimé dans le repère de Gazebo. Le robot était initialisé hors de la carte et visait un mauvais endroit.
* **Localisation jamais convergée.** AMCL restait à sa covariance initiale puis dérivait à l'estime, ce qui déclenchait notre alarme « localisation perdue » après deux à trois mètres.
* **Robot incliné.** La roulette arrière descend 1 cm plus bas que les roues : le robot penche d'environ 2,5° vers l'avant, ce que la TF ne montre pas. Des rayons du lidar touchaient le sol et, pour les caméras, le sol semblait monter de 4 cm par mètre.
* **Bruit des caméras de profondeur.** Le simulateur ajoute 10 cm d'écart-type directement sur la hauteur des points. Avec un seuil trop bas, la caméra haute voyait un obstacle dans presque toutes les directions sur un sol vide.
* **Rayons vides marqués comme obstacles.** Avec `inf_is_valid`, un rayon sans obstacle devenait un point à portée maximale, lui-même marqué comme obstacle car `obstacle_max_range` avait la même valeur.
* **Odométrie en rotation.** L'entraxe déclaré au plugin d'odométrie (0,3409 m) est plus petit que l'entraxe réel (0,393 m) ; nous prenons donc le cap sur l'IMU.
* **Simulation lente.** Sur nos machines, Gazebo tourne à environ 20 % du temps réel, ce qui rend chaque essai long et impose des délais de démarrage généreux.
* **Hésitation entre deux chemins.** Face à un obstacle contournable des deux côtés, la replanification périodique faisait basculer le chemin d'un côté à l'autre. Nous gardons maintenant le chemin en cours tant qu'il reste valide.
