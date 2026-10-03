# Guide de lancement — Navigation PARC 2026 (LiDAR + caméra)

Ce guide démarre la simulation officielle, puis toute la pile de navigation FOSA, **y compris la
perception caméra** (obstacles en hauteur invisibles au LiDAR). Les chemins des cartes et du
Behavior Tree sont résolus depuis les packages ROS installés : aucune commande ne dépend d'un
répertoire personnel.

Les passages ajoutés pour la caméra sont repérés par **[Caméra]**.

## Vue d'ensemble

```
Gazebo ──► /scan ──► scan_to_scan_filter_chain ──► /scan_filtered ──┐
   │                                                                 ├─► costmaps ──► Nav2
   └──► /top_camera_depth/points ──► camera_obstacle_node ──► /top_camera/obstacles_scan ─┘
                                            (couche camera_layer, bande 0,30–1,40 m)

localization_watchdog ──► /localization_ready ────────┐
camera_perception_watchdog ──► /camera_perception_ready ┴─► task_solution ──► NavigateToPose
```

## 0. Préparation — à faire une fois après une modification

Ouvrir un terminal dans la racine du workspace ROS 2, puis construire les **trois** packages de
la solution (**[Caméra]** `caytu_camera_perception` est nouveau) :

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
colcon build --symlink-install --packages-select caytu_camera_perception caytu_nav_bringup caytu_nav_solution
source install/setup.bash
```

Remplacer `/chemin/vers/ros2_ws` par le chemin réel du workspace (chez vous : `~/ros2_ws`). Les terminaux des étapes
suivantes doivent tous exécuter les deux commandes `source` ci-dessus avant d'appeler `ros2`.

Avec `--symlink-install`, modifier un fichier `.py` ou `.yaml` **existant** ne demande pas de
recompiler ; **ajouter** un fichier, si.

**[Caméra] Tests rapides, sans Gazebo :**

```bash
python3 -m pytest -q -s src/solutions/caytu_camera_perception/test
python3 -m pytest -q src/solutions/caytu_nav_bringup/test
python3 src/solutions/check_camera_integration.py src/solutions
```

Attendu : 5 tests passés, 8 tests passés, puis `RESULTAT : TOUT EST COHERENT`.

> Copier uniquement les commandes, pas les commentaires `#` ni les lignes de résultat. Une ligne
> contenant `->` crée un fichier parasite (le `>` est lu comme une redirection par bash).

## 1. Terminal 1 — Démarrer Gazebo, le robot et RViz

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash
export GZ_IP=127.0.0.1
ros2 launch parc_robot_bringup task.launch.py use_sim_time:=true
```

`export GZ_IP=127.0.0.1` **[Caméra]** évite l'erreur `Network is unreachable` de Gazebo, qui
empêche le bridge de voir les topics (dont le nuage de points de la caméra). À exécuter dans
**chaque** terminal qui lance Gazebo ou la solution.

Attendre que :

- Gazebo affiche le stade, le robot Sito-E et le cercle vert du but ;
- Gazebo soit en lecture (bouton **Play** si la simulation est en pause) ;
- RViz2 s'ouvre ;
- le robot apparaisse près de la pose définie par `task_params.yaml`.

Ne pas déplacer le robot, le but ou les obstacles à la souris dans Gazebo : cela fausserait
l'odométrie et la localisation.

**[Caméra] Contrôle avant de continuer**, dans un terminal libre :

```bash
ros2 topic echo /top_camera_depth/points --once --field header.frame_id
```

Attendu : `top_camera_depth`. Si le topic est absent, le bridge ne voit pas Gazebo : revoir
`GZ_IP`. Le bridge de l'image couleur (`top_camera_color/image_raw`) est déjà lancé par
`task.launch.py` : aucun bridge supplémentaire n'est nécessaire.

## 2. Terminal 2 — Démarrer localisation, navigation et perception caméra

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GZ_IP=127.0.0.1
ros2 launch caytu_nav_bringup solution_bringup.launch.py use_sim_time:=true
```

Cette unique commande démarre automatiquement :

- `scan_to_scan_filter_chain` : transforme `/scan` en `/scan_filtered` pour retirer
  l'auto-détection du châssis ;
- `map_server` : publie la carte statique ;
- `amcl` : publie la localisation et la TF `map -> odom` ;
- `localization_watchdog` : publie `/localization_ready` ;
- planner, controller DWB, behavior server, BT Navigator et leurs lifecycle managers ;
- **[Caméra]** `camera_obstacle_node` : nuage de points `/top_camera_depth/points` →
  `/top_camera/obstacles_scan` (démarre environ 3 s après le reste) ;
- **[Caméra]** `camera_perception_watchdog` : publie `/camera_perception_ready`.

**[Caméra] Dans les logs, vous devez voir :**

- `Bande de hauteur [0.30, 1.40] m, portée [0.30, 4.00] m, 106 secteurs -> /top_camera/obstacles_scan (frame top_camera_link)`
- `TF caméra résolue (cloud frame = "top_camera_depth")`
- `Using plugin "camera_layer"` pour la costmap locale (`controller_server`) **et** la costmap
  globale (`planner_server`)
- puis `Perception caméra PRÊTE` et `Localisation CONFIRMÉE`

Pour utiliser une autre carte, sans modifier de fichier, ajouter l'argument suivant :

```bash
ros2 launch caytu_nav_bringup solution_bringup.launch.py use_sim_time:=true \
  map:="$(ros2 pkg prefix caytu_nav_bringup)/share/caytu_nav_bringup/maps/stadium_map01.yaml"
```

## 3. RViz — Contrôler la localisation initialisée automatiquement

`solution_bringup.launch.py` lit dynamiquement la pose de spawn (`x`, `y`, `z`, `yaw`) de
`parc_robot_bringup/config/task_params.yaml` et l'envoie à AMCL. Il ne faut donc pas cliquer une
pose approximative au démarrage : c'était la cause d'un plan calculé depuis une mauvaise position
et dirigé vers un mur.

1. Dans RViz, régler **Fixed Frame** sur `map`.
2. Vérifier que le display **Map** affiche la carte et que **RobotModel** affiche le robot. Si
   l'un manque, cliquer **Add** puis ajouter `Map` (topic `/map`) et `RobotModel`.
3. Ajouter, si nécessaire, les displays suivants pour le diagnostic :

   - `LaserScan` : topic `/scan_filtered` ;
   - **[Caméra]** `LaserScan` : topic `/top_camera/obstacles_scan` (choisir une couleur
     différente du LiDAR) ;
   - `Map` : topics `/global_costmap/costmap` et `/local_costmap/costmap` ;
   - **[Caméra]** `Polygon` : topic `/local_costmap/published_footprint` (pour contrôler que le
     footprint épouse le robot) ;
   - `Path` : topic `/plan` ;
   - `Path` : topic `/local_plan`.

4. Attendre que le robot RViz se superpose au robot Gazebo et que le nuage laser soit cohérent
   avec les murs. Pour la configuration actuellement fournie, la pose attendue est
   approximativement `x=-0.200546`, `y=-7.485170`, `yaw=1.571`.
5. Utiliser **2D Pose Estimate** uniquement comme récupération si les deux robots ne se
   superposent pas : cliquer au centre de la position réelle dans Gazebo et tirer la flèche dans
   sa direction réelle. Ne pas utiliser une position approximative.

**[Caméra]** La couche caméra n'a pas de topic propre : elle est fusionnée dans
`/local_costmap/costmap` et `/global_costmap/costmap`. Pour la voir, regardez le scan caméra
(arc d'obstacles) et les cellules marquées correspondantes dans la costmap locale.

Ne pas utiliser **2D Goal Pose** pour la tâche officielle : `task_solution` lit le but officiel
`goal_x/goal_y` depuis `task_params.yaml` et l'envoie à Nav2.

## 4. Terminaux 3A, 3B et 3C — Vérifier les prérequis avant le goal

`ros2 topic hz` et `ros2 topic echo` sont des commandes bloquantes : la seconde ligne ne
s'exécute jamais tant que la première n'est pas arrêtée. Utiliser donc plusieurs terminaux (ou
arrêter la première commande avec `Ctrl+C` avant de taper la suivante).

### Terminal 3A — Débit du scan LiDAR en temps simulé

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash

ros2 topic hz --use-sim-time /scan_filtered
```

Résultat attendu : `/scan_filtered` est proche de **10 Hz**. Sans `--use-sim-time`, la commande
mesure le temps mur et peut afficher ~1.4 Hz si Gazebo tourne plus lentement que le temps réel ;
ce n'est pas une perte de scan puisque Nav2 utilise lui-même le temps simulé.

### Terminal 3B — État du watchdog de localisation

```bash
source /opt/ros/jazzy/setup.bash
cd /chemin/vers/ros2_ws
source install/setup.bash

ros2 topic echo /localization_ready
```

Résultat attendu : `data: true` après cinq mesures AMCL stables. Si aucun message arrive,
vérifier d'abord que le launch est à jour : arrêter les launches, exécuter l'étape 0
(`colcon build`), re-sourcer `install/setup.bash`, puis relancer les étapes 1 et 2.

### Terminal 3C — **[Caméra]** Perception caméra

Une commande à la fois (`Ctrl+C` entre chaque) :

```bash
ros2 topic hz --use-sim-time /top_camera_depth/points
ros2 topic hz --use-sim-time /top_camera/obstacles_scan
ros2 topic echo /camera_perception_ready --once
ros2 run tf2_ros tf2_echo base_footprint top_camera_link
```

| Commande | Résultat attendu |
|---|---|
| `hz` du nuage de points | nominal **15 Hz** ; mesuré **2 à 3 Hz** sur le Dell (voir la sous-section sur la simulation lente) |
| `hz` du scan d'obstacles | suit celui du nuage (3 à 4 Hz mesurés sur le Dell) |
| `/camera_perception_ready` | `data: true` (seuils `min_rate_hz` et `window_sec` dans `camera_obstacles.yaml`, en temps simulé) |
| `tf2_echo` | translation ≈ (−0,052 ; 0 ; 1,013), rotation identité (la première ligne « frame does not exist » est un retard de démarrage) |

Le terminal 2 affiche toutes les 5 s une ligne de statistiques du node caméra :

```
x Hz, y ms/img | secteurs : N obstacle, M libres, K inconnus | z(base) p5/p50/p95 = a/b/c m
```

- `x Hz` est mesuré en **temps réel** (pas en temps simulé) : il est bas si Gazebo est lent.
- `y ms/img` : temps de calcul, de l'ordre de 8 à 14 ms sur le vrai nuage.
- **`z(base) p5` doit être proche de 0** (c'est le sol, valeurs observées : −0,03 à −0,04). Si
  elle est loin de 0, les axes du nuage sont mal interprétés : ne pas continuer, prévenir le
  trinôme caméra avec la ligne complète.

Arrêter les commandes de surveillance avec `Ctrl+C`, puis exécuter dans un terminal les contrôles
ponctuels suivants :

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_footprint
```

Les lifecycle nodes doivent être `active` et la dernière commande doit afficher une transformée
continue `map -> base_footprint`. Si `/localization_ready` reste à false, ne pas lancer le
client : refaire l'étape RViz et consulter les logs AMCL.

### **[Caméra]** Si la simulation est lente

Mesurer le facteur temps réel (Real Time Factor, RTF) de deux façons :

- en bas de la fenêtre Gazebo (pourcentage affiché à côté des boutons de lecture) ;
- `ros2 topic hz /scan_filtered` **sans** `--use-sim-time` : le résultat divisé par 10 donne le RTF.

Mesures sur le Dell de l'équipe : 0,54 à 0,67 Hz, soit un RTF de 5 à 7 %, et 7,65 % affichés par
Gazebo. Il faut alors 13 à 19 s réelles pour 1 s simulée. Dans ce cas :

- les erreurs `Lookup would require extrapolation into the past` au démarrage sont normales : elles
  se résorbent quand l'horloge simulée rattrape la transformation `map -> odom` datée dans le
  futur par AMCL ;
- `/camera_perception_ready` reste à `false` si le débit vu en temps simulé est sous le seuil
  `min_rate_hz` de `camera_obstacles.yaml` ;
- le délai de repli de la caméra (étape 5) se compte en **temps simulé** : à 7 %, les 20 s par
  défaut durent 4 à 6 minutes réelles, ce qui explique que le robot reste longtemps immobile au
  départ. Pour les tests, le réduire (`-p camera_wait_timeout_sec:=2.0`, soit environ 30 s réelles) ;
- les 600 s du règlement durent alors des heures réelles : réserver les tests complets à une
  machine rapide.

Pour développer le filtre caméra sans Gazebo, enregistrer une fois des données puis les rejouer à
pleine vitesse :

```bash
ros2 bag record -o cam_bag /top_camera_depth/points /tf /tf_static /clock /scan /odom
# Ctrl+C pour arrêter l'enregistrement, puis : ros2 bag info cam_bag
ros2 bag play cam_bag --clock
```

Le chemin du bag doit être placé **avant** `--clock` : sinon `ros2 bag play` lit `cam_bag` comme la
fréquence de l'horloge et échoue (`invalid positive_float value`).

## 5. Terminal 3 — Envoyer le goal officiel

Quand les prérequis sont validés (localisation **et** caméra), lancer le client dans le même
terminal :

```bash
ros2 run caytu_nav_solution task_solution --ros-args \
  -p use_sim_time:=true \
  -p use_test_goal:=false \
  -p test_mode:=false
```

Le client attend **`/localization_ready` et `/camera_perception_ready`**, envoie une unique
action `NavigateToPose`, puis annule le goal au bout de 600 s. Il annule également le goal si le
watchdog signale une perte de localisation.

**[Caméra] Comportement attendu dans les logs du client :**

1. `attente de /localization_ready ... et de /camera_perception_ready (require_camera=True)`
2. `Signal localization_ready reçu (réel)`
3. `Localisation prête, attente de la caméra (max 20s avant repli LiDAR seul)`
4. envoi du but dès que la caméra est prête.

Si la caméra n'est pas prête au bout de `camera_wait_timeout_sec` (20 s en temps simulé par
défaut), le client **part avec le LiDAR seul** et l'écrit dans le log. Paramètres :

| Paramètre | Effet |
|---|---|
| `-p require_camera:=false` | ne plus attendre la caméra (la couche `camera_layer` reste active dans les costmaps) |
| `-p camera_wait_timeout_sec:=N` | délai en temps simulé avant le repli LiDAR seul |

Un message `[ERROR] localization_ready absent ... bloqué` affiché une fraction de seconde avant
`Signal localization_ready reçu` est sans gravité (course entre le minuteur de repli et l'arrivée
du signal).

Observer simultanément :

- Gazebo : le déplacement réel et les contacts éventuels avec les murs/obstacles ;
- RViz : le plan global, le plan local, les costmaps, le footprint et **[Caméra]** le scan
  d'obstacles caméra ;
- Terminal 2 : les logs AMCL, planner, controller et BT Navigator ;
- les collisions :

```bash
ros2 topic echo /base_collisions --once
ros2 topic echo /top_chassis_collisions --once
```

> La commande de lancement imposée par le règlement est `ros2 run <package> task_solution.py`.
> Vérifier qu'elle fonctionne telle quelle : `ros2 run caytu_nav_solution task_solution.py` ;
> sinon, voir le point 3.2 de `LANCEMENT_ET_APPORT_CAMERA.md`.

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

- `linear.x = 0` et `angular.z != 0` : Nav2 demande une rotation ; consulter les logs BT pour
  distinguer `Spin` recovery de DWB ;
- `linear.x > 0` mais le robot ne bouge pas : contrôler le bridge et les contacts dans Gazebo ;
- aucun plan ou erreurs TF : revenir aux étapes 2–4 ;
- costmap occupée dans le footprint : vérifier `/scan_filtered`, l'estimation RViz **et
  [Caméra] le footprint (voir section 8)**.

### **[Caméra]** Cas « le robot est bloqué alors que rien ne gêne » (obstacle fantôme)

Départager caméra et LiDAR :

```bash
# Le scan caméra annonce-t-il des obstacles inattendus ? (valeur finie = obstacle, inf = libre, nan = inconnu)
ros2 topic echo /top_camera/obstacles_scan --once --field ranges

# Vider la costmap locale
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

- Le robot repart après le clear **et** le scan caméra montre des valeurs finies où il n'y a
  rien : la cause est la caméra (seuils de `camera_obstacles.yaml`, ou effacement de la couche).
- Le robot repart mais le scan caméra est propre : la marque venait du LiDAR.
- Rien ne change : chercher ailleurs (footprint, localisation, Behavior Tree).

## 7. **[Caméra]** Tests de la perception en simulation

Ces tests valident que la caméra voit bien ce que le LiDAR ne voit pas. Ils déplacent le robot :
**relancer ensuite la simulation** pour repartir de la pose de spawn officielle.

**Plateau de table.** Le robot apparaît en (−0,20 ; −7,49), orienté vers +y, avec une
`cafe_table` droit devant (environ 4,5 m). Avancer d'environ 1,5 m :

```bash
timeout 5 ros2 topic pub -r 10 /robot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic pub --once /robot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{}"
```

Attendu dans RViz : un arc du scan caméra à la distance du plateau, et des cellules marquées dans
la costmap locale là où le LiDAR seul ne voit que les pieds de la table.

**Mémoire.** Faire pivoter le robot de 90° puis revenir : la marque du plateau doit **persister**
pendant que la caméra ne regarde pas (couche `camera_layer` séparée, aucun effacement en dessous
de 1 m).

**Comparaison avec et sans caméra.** Pour un vrai « LiDAR seul », retirer `camera_layer` des
listes `plugins` de `global_costmap_params.yaml` et `local_costmap_params.yaml`, puis relancer.
`-p require_camera:=false` ne fait que ne plus attendre le signal caméra. Faire au moins 3 essais
par configuration :

| Essai | Temps | Distance au but | Collisions | Recoveries |
|---|---|---|---|---|
| LiDAR seul | | | | |
| LiDAR + caméra | | | | |

## 8. Points connus à valider

| Point | État | Vérification |
|---|---|---|
| Footprint Nav2 (1,3 × 0,5 m) | Probablement trop long : les 1,248 m du châssis semblent être sa hauteur | Gazebo : clic droit sur le robot → View → Collisions ; RViz : `Polygon` sur `/local_costmap/published_footprint`. Détails dans `LANCEMENT_ET_APPORT_CAMERA.md`, §3.5 |
| Commande `task_solution.py` | À tester telle que le règlement la donne | `ros2 run caytu_nav_solution task_solution.py` |
| Simulation lente (RTF ≈ 0,06) | Cause non confirmée (GPU intégré Intel détecté, pas de rendu logiciel) | Mesurer le RTF de base, `top`, `powerprofilesctl get` |
| Run complet avec caméra « prête » | Pas encore obtenu | Étapes 1 à 5 sur une machine à RTF proche de 1 |
| `enable_image_bridge` | Doit rester `false` : l'image couleur est déjà bridgée par `task.launch.py` | `ros2 topic list \| grep image_raw` |

## 9. Arrêt propre

Arrêter dans cet ordre :

1. Terminal du client `task_solution` : `Ctrl+C` ;
2. Terminal `solution_bringup.launch.py` : `Ctrl+C` ;
3. Terminal `task.launch.py` (Gazebo/RViz) : `Ctrl+C`.

Les messages `lifecycle_manager ... context is not valid` ou `exit code -2` affichés à l'arrêt
sont normaux.

Relancer ensuite à partir de l'étape 1 pour une nouvelle tentative propre.