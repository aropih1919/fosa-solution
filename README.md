# TEAM FOSA: PARC Engineers League 2026

*Version française plus bas.*

## Introduction

Phase 1 asks teams to drive the CAYTU Sito-É service robot autonomously through the restaurant of a smart stadium, from its start position to the green circle, in under 10 minutes and without touching any obstacle. A stadium assistant robot must first be able to move on its own between tables, chairs and visitors: every other service it will offer (guidance, information, safety) depends on it. Our solution is built on Nav2 and on the robot's three range sensors: the lidar, low and precise, and the two depth cameras, which see what the lidar cannot (table tops, chair backs, people). We aimed for a solution that is fully automatic, decisive and robust: one command, no action in RViz, a robot that commits to a route instead of hesitating, and correct behaviour when the start pose, the goal or the obstacles change.

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

* `NumPy`: floor filtering of the lidar scan, map check at start-up, comparison of paths.

    * `$ sudo apt-get install python3-numpy`

All dependencies are declared in the `package.xml` files, so they can also be installed with `rosdep install --from-paths src --ignore-src -r -y`, then built with `colcon build`.

## Task

The `task_solution.py` node runs the whole solution. It reads the start pose and the goal from `task_params.yaml` in the official package, then starts perception, localization and Nav2 by itself.

**Localization.** No SLAM map and no particle filter: the `map` frame is aligned with the Gazebo world frame from the start pose, then the position is tracked with wheel distance and IMU heading. The odometry of the official drive plugin describes the middle of the wheel axle, which is 9.5 cm in front of `base_footprint`: when the robot turns on the spot, the axle stays still but `base_footprint` moves on an arc. The localization takes this into account, so the position stays right after every turn.

**Perception.** The lidar scan is cleaned of the sectors where it sees the robot and of the beams that hit the floor; the two camera point clouds are height-filtered in a levelled frame and converted to scans. Each sensor feeds its own costmap layer, and a wall map of the café, generated from the official world, helps the planner (it is dropped automatically if the lidar shows a different world).

**Planning, control, and decisions.** The Smac 2D planner recomputes the path twice per second from what the sensors have discovered, and the Regulated Pure Pursuit controller follows it at up to 0.7 m/s, slowing down in tight turns and near obstacles. Between the planner and the controller, our `path_keeper` node makes the robot commit to the route it has chosen. A new path on the same route (the two never more than 1 m apart) is always accepted, so the path keeps adjusting smoothly. A path on another route, for instance around the other side of a table, is accepted only if the current route is blocked, runs too close to an obstacle twice in a row, or if the new route saves more than 1.5 m plus the cost of turning around; such a change for a gain happens at most once every 5 s. Paths are scored with the planner's own cost formula, in metres.

**When something goes wrong.** Every interruption is logged with the cause given by Nav2, and the next step depends on that cause and on the progress made: send the goal again at once when nothing needs cleaning (for example, a position not yet available right after start-up), clear the global costmap only beyond 2.5 m from the robot (a table top is seen by the cameras alone, towards the front, and must not be forgotten while it is beside the robot), clear around the robot, and clear everything only as a last resort. If the goal itself cannot be reached, for example because an obstacle stands on it, the robot goes to the closest free point around it. If Nav2 is still not ready 60 s after start-up (its lifecycle manager can wait forever for a lost service answer), everything is restarted, with a longer delay each time. The perception, `path_keeper` and Nav2 server nodes are restarted if they stop. On arrival the robot is stopped, a short report of the run is written, and everything that was started is shut down.

Commands, in two terminals:

```
ros2 launch parc_robot_bringup task.launch.py
```

```
ros2 run caytu_nav_solution task_solution.py
```

At the end of each run, the log shows where the time went (start-up, driving, turning on the spot, stops), the final distance to the goal and the cause of each interruption. The same report is saved in `~/.ros/fosa_runs/`. For comparisons, the driving settings of our previous version can be restored in one parameter: `ros2 run caytu_nav_solution task_solution.py --ros-args -p drive_profile:=reference`.

## Challenges Faced

* **Different frames.** Our first map, built with SLAM, had its origin at the robot start pose, while `task_params.yaml` is expressed in the Gazebo frame. The robot was initialised outside the map and aimed at the wrong place.
* **Localization that never converged.** AMCL stayed at its initial covariance and drifted on dead reckoning, which triggered our "localization lost" alarm after two to three metres.
* **Tilted robot.** The rear caster sits 1 cm lower than the wheels: the robot leans about 2.5° forward, which the TF tree does not show. Lidar beams hit the floor and, for the cameras, the floor seemed to rise by 4 cm per metre.
* **Depth-camera noise.** The simulator adds 10 cm of standard deviation directly to point heights. With a threshold set too low, the top camera saw an obstacle in almost every direction on an empty floor.
* **Empty rays marked as obstacles.** With `inf_is_valid`, a ray with no obstacle became a point at maximum range, itself marked as an obstacle because `obstacle_max_range` had the same value.
* **Rotation odometry.** The wheel separation declared to the odometry plugin (0.3409 m) is smaller than the real one (0.393 m), so we take the heading from the IMU.
* **The odometry follows the axle, not the robot centre.** Reading the plugin source, we found that its odometry integrates the wheels, so it tracks the middle of the axle, 9.5 cm in front of `base_footprint`. Without correction, the estimated position is off by up to 19 cm after a half turn, which also shifts every obstacle seen after the turn.
* **Hesitation in front of a table.** Facing a table that can be passed on either side, the robot sometimes turned left, then right, then left, several seconds on the spot. On a test bench that runs the real Nav2 and our solution with a simplified 2D simulator, we traced the cause: the cameras only see ±45° ahead and their depth is noisy, so the side the robot looks at always seems a little more cluttered than the other one, and the next path chooses the other side. A first attempt, replanning only when the path became invalid, with the Smac Hybrid-A* planner, was slower (3 min 18 s instead of 2 min 33 s for the official run in our simulator). The `path_keeper` node now keeps periodic replanning but makes the robot commit to a route, as described above.
* **Seeing further did not help.** We tried marking obstacles up to 3.5 m instead of 2.5 m with the top camera, to see tables earlier. On the test bench, the noisy far points closed narrow passages on and off, and the robot hesitated more. We kept 2.5 m.
* **Navigation interrupted for no visible reason.** Reading the Nav2 source, we found that the behaviour tree gives each server 20 ms to answer by default. A single late answer makes a tree node fail, and the navigation can be abandoned without any recovery. We raised this delay to 500 ms, and every interruption is now logged with the error code given by Nav2.
* **A start-up that never ends.** Nav2's lifecycle manager waits without any time limit for each node to answer. If one answer is lost, Nav2 never becomes active and the robot never moves; we saw it happen on the test bench. The solution now restarts everything if Nav2 is not ready in time.
* **Tables forgotten after a costmap clear.** The lidar is 5 cm above the floor: of a 91 cm table it sees only the 4 cm central column. The top is seen by the cameras alone, in a 90° cone towards the front. Clearing the whole global costmap during a recovery erased the tables standing beside the robot, so we now clear only what is far from it, except as a last resort.

--------------------------------------------------

# ÉQUIPE FOSA : PARC Ligue des Ingénieurs 2026

## Introduction

La phase 1 demande de faire naviguer de façon autonome le robot de service CAYTU Sito-É dans le restaurant d'un stade intelligent, de son point de départ jusqu'au cercle vert, en moins de 10 minutes et sans toucher d'obstacle. Un robot assistant de stade doit avant tout savoir se déplacer seul parmi les tables, les chaises et les visiteurs : tous les services qu'il rendra ensuite (guidage, information, sécurité) en dépendent. Notre solution s'appuie sur Nav2 et sur les trois capteurs de distance du robot : le lidar, bas et précis, et les deux caméras de profondeur, qui voient ce que le lidar ne voit pas (plateaux de table, dossiers de chaise, personnes). Nous avons visé une solution entièrement automatique, décidée et robuste : une seule commande, aucune action dans RViz, un robot qui s'engage sur une route au lieu d'hésiter, et un comportement correct si le point de départ, le but ou les obstacles changent.

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

* `NumPy` : filtrage du sol dans le scan du lidar, vérification de la carte au démarrage, comparaison des chemins.

    * `$ sudo apt-get install python3-numpy`

Toutes les dépendances sont déclarées dans les `package.xml` ; on peut donc aussi les installer avec `rosdep install --from-paths src --ignore-src -r -y`, puis compiler avec `colcon build`.

## Tâche

Le nœud `task_solution.py` exécute toute la solution. Il lit le point de départ et le but dans `task_params.yaml` du paquet officiel, puis démarre lui-même la perception, la localisation et Nav2.

**Localisation.** Ni carte SLAM ni filtre à particules : le repère `map` est calé sur le repère de Gazebo à partir du point de départ, puis la position est suivie avec la distance des roues et le cap de l'IMU. L'odométrie du plugin de propulsion officiel décrit le milieu de l'essieu, 9,5 cm devant `base_footprint` : quand le robot pivote sur place, l'essieu ne bouge pas mais `base_footprint` décrit un arc. La localisation en tient compte, si bien que la position reste juste après chaque virage.

**Perception.** Le scan du lidar est débarrassé des secteurs où il voit le robot et des rayons qui touchent le sol ; les nuages des deux caméras sont filtrés en hauteur dans un repère remis à l'horizontale, puis convertis en scans. Chaque capteur alimente sa propre couche de costmap, et une carte des murs du café, générée depuis le monde officiel, aide le planificateur (elle est écartée automatiquement si le lidar montre un autre monde).

**Planification, commande et décisions.** Le planificateur Smac 2D recalcule le chemin deux fois par seconde à partir de ce que les capteurs ont découvert, et le contrôleur Regulated Pure Pursuit le suit jusqu'à 0,7 m/s, en ralentissant dans les virages serrés et près des obstacles. Entre le planificateur et le contrôleur, notre nœud `path_keeper` fait s'engager le robot sur la route qu'il a choisie. Un nouveau chemin qui suit la même route (les deux ne s'écartent nulle part de plus d'un mètre) est toujours accepté : le chemin continue de s'ajuster en douceur. Un chemin qui passe par une autre route, par exemple de l'autre côté d'une table, n'est accepté que si la route suivie est coupée, si elle frôle un obstacle deux calculs de suite, ou si la nouvelle route fait gagner plus de 1,5 m en plus du prix du demi-tour ; un tel changement pour un gain n'a lieu qu'une fois toutes les 5 s au plus. Les chemins sont notés avec la formule de coût du planificateur lui-même, en mètres.

**Quand quelque chose se passe mal.** Chaque interruption est écrite dans le journal avec la cause donnée par Nav2, et la suite dépend de cette cause et du progrès accompli : renvoyer le but tout de suite quand il n'y a rien à nettoyer (par exemple une position pas encore disponible juste après le démarrage), effacer la costmap globale seulement à plus de 2,5 m du robot (un plateau de table n'est vu que par les caméras, vers l'avant, et ne doit pas être oublié quand il est sur le côté du robot), effacer autour du robot, et tout effacer seulement en dernier recours. Si le but lui-même est inaccessible, par exemple parce qu'un obstacle est posé dessus, le robot rejoint le point libre le plus proche autour du but. Si Nav2 n'est toujours pas prêt 60 s après le démarrage (son gestionnaire de cycle de vie peut attendre sans fin une réponse perdue), tout est relancé, avec un délai plus long à chaque fois. Les nœuds de perception, `path_keeper` et les serveurs Nav2 sont relancés s'ils s'arrêtent. À l'arrivée, le robot est arrêté, un court rapport du trajet est écrit, et tout ce qui a été lancé est fermé.

Commandes, dans deux terminaux :

```
ros2 launch parc_robot_bringup task.launch.py
```

```
ros2 run caytu_nav_solution task_solution.py
```

À la fin de chaque trajet, le journal montre où est passé le temps (démarrage, roulage, pivots sur place, arrêts), la distance finale au but et la cause de chaque interruption. Le même rapport est enregistré dans `~/.ros/fosa_runs/`. Pour comparer, les réglages de conduite de notre version précédente se rétablissent en un paramètre : `ros2 run caytu_nav_solution task_solution.py --ros-args -p drive_profile:=reference`.

## Défis rencontrés

* **Repères différents.** Notre première carte, construite au SLAM, avait pour origine le point de départ du robot, alors que `task_params.yaml` est exprimé dans le repère de Gazebo. Le robot était initialisé hors de la carte et visait un mauvais endroit.
* **Localisation jamais convergée.** AMCL restait à sa covariance initiale puis dérivait à l'estime, ce qui déclenchait notre alarme « localisation perdue » après deux à trois mètres.
* **Robot incliné.** La roulette arrière descend 1 cm plus bas que les roues : le robot penche d'environ 2,5° vers l'avant, ce que la TF ne montre pas. Des rayons du lidar touchaient le sol et, pour les caméras, le sol semblait monter de 4 cm par mètre.
* **Bruit des caméras de profondeur.** Le simulateur ajoute 10 cm d'écart-type directement sur la hauteur des points. Avec un seuil trop bas, la caméra haute voyait un obstacle dans presque toutes les directions sur un sol vide.
* **Rayons vides marqués comme obstacles.** Avec `inf_is_valid`, un rayon sans obstacle devenait un point à portée maximale, lui-même marqué comme obstacle car `obstacle_max_range` avait la même valeur.
* **Odométrie en rotation.** L'entraxe déclaré au plugin d'odométrie (0,3409 m) est plus petit que l'entraxe réel (0,393 m) ; nous prenons donc le cap sur l'IMU.
* **L'odométrie suit l'essieu, pas le centre du robot.** En lisant le code source du plugin, nous avons vu que son odométrie intègre les roues : elle suit donc le milieu de l'essieu, 9,5 cm devant `base_footprint`. Sans correction, la position estimée est fausse de 19 cm au plus après un demi-tour, et tout obstacle vu après le virage est décalé d'autant.
* **Hésitation devant une table.** Face à une table contournable des deux côtés, le robot tournait parfois à gauche, puis à droite, puis à gauche, plusieurs secondes sur place. Sur un banc d'essai qui fait tourner le vrai Nav2 et notre solution avec un simulateur 2D simplifié, nous en avons trouvé la cause : les caméras ne voient que ±45° vers l'avant et leur profondeur est bruitée, si bien que le côté que le robot regarde paraît toujours un peu plus encombré que l'autre, et le chemin suivant choisit l'autre côté. Un premier essai, qui ne recalculait le chemin que s'il devenait invalide, avec le planificateur Smac Hybrid-A*, était plus lent (3 min 18 s au lieu de 2 min 33 s pour le trajet officiel dans notre simulateur). Le nœud `path_keeper` garde la replanification périodique mais fait s'engager le robot sur une route, comme décrit plus haut.
* **Voir plus loin n'aidait pas.** Nous avons essayé de marquer les obstacles jusqu'à 3,5 m au lieu de 2,5 m avec la caméra haute, pour voir les tables plus tôt. Sur le banc d'essai, les points lointains, bruités, refermaient des passages étroits par intermittence, et le robot hésitait davantage. Nous avons gardé 2,5 m.
* **Navigation interrompue sans raison visible.** En lisant le code source de Nav2, nous avons vu que l'arbre de comportement laisse par défaut 20 ms à chaque serveur pour répondre. Une seule réponse en retard fait échouer un nœud de l'arbre, et la navigation peut être abandonnée sans aucune récupération. Nous avons porté ce délai à 500 ms, et chaque interruption est maintenant écrite dans le journal avec le code d'erreur donné par Nav2.
* **Un démarrage qui ne finit jamais.** Le gestionnaire de cycle de vie de Nav2 attend sans limite de temps la réponse de chaque nœud. Si une réponse se perd, Nav2 ne devient jamais actif et le robot ne bouge pas ; nous l'avons vu arriver sur le banc d'essai. La solution relance maintenant tout si Nav2 n'est pas prêt à temps.
* **Des tables oubliées après un effacement de costmap.** Le lidar est à 5 cm du sol : d'une table de 91 cm, il ne voit que la colonne centrale de 4 cm. Le plateau n'est vu que par les caméras, dans un cône de 90° vers l'avant. Effacer toute la costmap globale pendant une récupération supprimait les tables situées sur le côté du robot ; nous n'effaçons donc plus que ce qui est loin de lui, sauf en dernier recours.
