# Guide de lancement — Navigation PARC 2026

Ce guide démarre la simulation officielle, puis toute la pile de navigation
FOSA. Les chemins des cartes et du Behavior Tree sont résolus depuis les packages
ROS installés : aucune commande ne dépend d'un répertoire personnel.

## 0. Préparation — à faire une fois après une modification

Ouvrir un terminal dans la racine du workspace ROS 2, puis construire les deux
packages de la solution :

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
colcon build --symlink-install --packages-select caytu_nav_bringup caytu_nav_solution
source install/setup.bash
```

Remplacer `/chemin/vers/ros2_ws` par le chemin réel du workspace. Les terminaux des
étapes suivantes doivent tous exécuter les deux commandes `source` ci-dessus avant
d'appeler `ros2`.


## 1. Terminal 1 — Démarrer Gazebo, le robot et RViz

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash
ros2 launch parc_robot_bringup task.launch.py use_sim_time:=true
```

Attendre que :

- Gazebo affiche le stade, le robot Sito-E et le cercle vert du but ;
- Gazebo soit en lecture (bouton **Play** si la simulation est en pause) ;
- RViz2 s'ouvre ;
- le robot apparaisse près de la pose définie par `task_params.yaml`.

Ne pas déplacer le robot, le but ou les obstacles à la souris dans Gazebo : cela
fausserait l'odométrie et la localisation.

## 2. Terminal 2 — Démarrer localisation et navigation FOSA

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash
ros2 launch caytu_nav_bringup solution_bringup.launch.py use_sim_time:=true
```

Cette unique commande démarre automatiquement :

- `scan_to_scan_filter_chain` : transforme `/scan` en `/scan_filtered` pour retirer
  l'auto-détection du châssis ;
- `map_server` : publie la carte statique ;
- `amcl` : publie la localisation et la TF `map -> odom` ;
- `localization_watchdog` : publie `/localization_ready` ;
- planner, controller DWB, behavior server, BT Navigator et leurs lifecycle managers.

Pour utiliser une autre carte, sans modifier de fichier, ajouter l'argument suivant :

```bash
ros2 launch caytu_nav_bringup solution_bringup.launch.py use_sim_time:=true \
  map:="$(ros2 pkg prefix caytu_nav_bringup)/share/caytu_nav_bringup/maps/stadium_map01.yaml"
```

## 3. RViz — Contrôler la localisation initialisée automatiquement

`solution_bringup.launch.py` lit dynamiquement la pose de spawn (`x`, `y`, `z`,
`yaw`) de `parc_robot_bringup/config/task_params.yaml` et l'envoie à AMCL. Il ne faut
donc pas cliquer une pose approximative au démarrage : c'était la cause d'un plan
calculé depuis une mauvaise position et dirigé vers un mur.

1. Dans RViz, régler **Fixed Frame** sur `map`.
2. Vérifier que le display **Map** affiche la carte et que **RobotModel** affiche le
   robot. Si l'un manque, cliquer **Add** puis ajouter `Map` (topic `/map`) et
   `RobotModel`.
3. Ajouter, si nécessaire, les displays suivants pour le diagnostic :

   - `LaserScan` : topic `/scan_filtered` ;
   - `Map` : topics `/global_costmap/costmap` et `/local_costmap/costmap` ;
   - `Path` : topic `/plan` ;
   - `Path` : topic `/local_plan`.

4. Attendre que le robot RViz se superpose au robot Gazebo et que le nuage laser soit
   cohérent avec les murs. Pour la configuration actuellement fournie, la pose attendue
   est approximativement `x=-0.200546`, `y=-7.485170`, `yaw=1.571`.
5. Utiliser **2D Pose Estimate** uniquement comme récupération si les deux robots ne
   se superposent pas : cliquer au centre de la position réelle dans Gazebo et tirer
   la flèche dans sa direction réelle. Ne pas utiliser une position approximative.

Ne pas utiliser **2D Goal Pose** pour la tâche officielle : `task_solution` lit le
but officiel `goal_x/goal_y` depuis `task_params.yaml` et l'envoie à Nav2.

## 4. Terminaux 3A et 3B — Vérifier les prérequis avant le goal

`ros2 topic hz` et `ros2 topic echo` sont des commandes bloquantes : la seconde ligne
ne s'exécute jamais tant que la première n'est pas arrêtée. Utiliser donc deux
terminaux (ou arrêter la première commande avec `Ctrl+C` avant de taper la suivante).

### Terminal 3A — Débit du scan en temps simulé

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash

ros2 topic hz --use-sim-time /scan_filtered
```

Résultat attendu : `/scan_filtered` est proche de **10 Hz**. Sans
`--use-sim-time`, la commande mesure le temps mur et peut afficher ~1.4 Hz si Gazebo
tourne plus lentement que le temps réel ; ce n'est pas une perte de scan puisque Nav2
utilise lui-même le temps simulé.

### Terminal 3B — État du watchdog

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash

ros2 topic echo /localization_ready
```

Résultat attendu : `data: true` après cinq mesures AMCL stables. Si aucun message
arrive, vérifier d'abord que le launch est à jour : arrêter les launches, exécuter
l'étape 0 (`colcon build`), re-sourcer `install/setup.bash`, puis relancer les étapes
1 et 2.

Arrêter les commandes de surveillance avec `Ctrl+C`, puis exécuter dans un terminal
les contrôles ponctuels suivants :

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_footprint
```

Les lifecycle nodes doivent être `active` et la dernière commande doit afficher une
transformée continue `map -> base_footprint`. Si `/localization_ready` reste à false,
ne pas lancer le client : refaire l'étape RViz et consulter les logs AMCL.

## 5. Terminal 3 — Envoyer le goal officiel

Quand les prérequis sont validés, lancer le client dans le même terminal :

```bash
ros2 run caytu_nav_solution task_solution --ros-args \
  -p use_sim_time:=true \
  -p use_test_goal:=false \
  -p test_mode:=false
```

Le client attend `/localization_ready`, envoie une unique action `NavigateToPose`,
puis annule le goal au bout de 600 s. Il annule également le goal si le watchdog
signale une perte de localisation. Observer simultanément :

- Gazebo : le déplacement réel et les contacts éventuels avec les murs/obstacles ;
- RViz : le plan global, le plan local, les costmaps et le footprint ;
- Terminal 2 : les logs AMCL, planner, controller et BT Navigator.

## 6. Diagnostic du cas « le robot tourne sans avancer »

Dans un quatrième terminal, lancer une seule commande à la fois pendant le problème :

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash

# Commande réellement envoyée au robot : comparer linear.x et angular.z.
ros2 topic echo /robot_base_controller/cmd_vel_unstamped

# Vérifier qu'il n'existe pas de publisher inattendu.
ros2 topic info -v /robot_base_controller/cmd_vel_unstamped

# Rechercher le message de scores DWB, puis l'afficher si présent.
ros2 topic list -t | grep -E 'LocalPlanEvaluation|trajectory'

# Vérifier qu'un plan global est produit.
ros2 topic echo /plan --once
```

Interprétation rapide :

- `linear.x = 0` et `angular.z != 0` : Nav2 demande une rotation ; consulter les
  logs BT pour distinguer `Spin` recovery de DWB ;
- `linear.x > 0` mais le robot ne bouge pas : contrôler le bridge et les contacts
  dans Gazebo ;
- aucun plan ou erreurs TF : revenir aux étapes 2–4 ;
- costmap occupée dans le footprint : vérifier `/scan_filtered` et l'estimation RViz.

## 7. Arrêt propre

Arrêter dans cet ordre :

1. Terminal du client `task_solution` : `Ctrl+C` ;
2. Terminal `solution_bringup.launch.py` : `Ctrl+C` ;
3. Terminal `task.launch.py` (Gazebo/RViz) : `Ctrl+C`.

Relancer ensuite à partir de l'étape 1 pour une nouvelle tentative propre.
