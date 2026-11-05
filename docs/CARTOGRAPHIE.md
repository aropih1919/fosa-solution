# Cartographie manuelle — mode d'emploi (phase réelle, Dakar)

Le robot est piloté au clavier, slam_toolbox construit la carte, et on
l'enregistre. La solution sait ensuite naviguer sur cette carte en se
localisant avec slam_toolbox. On fait le même parcours dans Gazebo pour tout
valider avant de passer au vrai robot.

## 1. Ce qui a été ajouté

| Fichier | Rôle |
|---|---|
| `caytu_nav_bringup/launch/mapping.launch.py` | Lance la cartographie : filtres lidar, slam_toolbox, session de cartographie, et en option l'enregistrement, RViz ou le rejeu |
| `caytu_nav_bringup/launch/robot_interfaces.launch.py` | Adaptation au robot (repère au sol, filtres du lidar), partagée entre la navigation et la cartographie |
| `caytu_nav_bringup/config/robots/sim.yaml`, `real.yaml` | Topics et réglages de Gazebo (phase 1) et du robot réel (`docs/ROBOT_REEL.md`) |
| `caytu_nav_bringup/config/slam_mapping.yaml` | Réglages de slam_toolbox pour cartographier |
| `caytu_nav_bringup/config/slam_localization.yaml` | Réglages de slam_toolbox pour se localiser sur une carte enregistrée |
| `caytu_nav_bringup/rviz/mapping.rviz` | Vue RViz : carte, lidar filtré, robot, outils Mesurer et Publish Point |
| `caytu_nav_solution/mapping_session.py` | Note la pose de départ, sauvegarde automatiquement toutes les 30 s et fournit le service `/fosa/save_map` |
| `caytu_nav_solution/save_map.py` | `ros2 run caytu_nav_solution save_map` |
| `caytu_nav_solution/edit_map.py` | Efface un meuble mobile, ajoute un mur virtuel |
| `caytu_nav_solution/tf_bag_relay.py` | Rejeu d'un enregistrement (retire l'ancienne TF map -> odom) |
| `caytu_nav_solution/wait_for_tf.py` | Fait attendre Nav2 jusqu'à ce que slam_toolbox ait localisé le robot |
| `caytu_nav_solution/map_io.py` | Fichiers de carte, métadonnées et repères (testé sans ROS) |
| `tools/compare_maps.py` | Gazebo : note la carte générée face aux murs réels du café |
| `tools/diag_slam.py` | Gazebo : écart en direct entre slam_toolbox et la vraie pose du robot, pour trouver le moment où une carte décroche |
| `caytu_nav_bringup/launch/slam_odometry.launch.py` | Option `imu_odom:=true` : slam_toolbox sur l'odométrie corrigée par l'IMU (essai) |

Dans `solution_bringup.launch.py`, un nouvel argument `localization` existe :
`odom_imu` (défaut, comportement de la phase 1) ou `slam`. Dans
`task_solution.py`, un nouveau paramètre existe : `-p slam_map:=<dossier>`.

Changement qui touche aussi la phase 1 : Nav2 démarre maintenant dès que la
position du robot existe (`wait_for_tf`), au lieu de démarrer en même temps que
la localisation. En temps simulé, la costmap globale calcule son délai
d'attente avec une horloge qui vaut 0 avant le premier `/clock`. Si la position
n'était pas encore publiée à ce moment-là, l'activation échouait et tout Nav2
restait arrêté. Nous l'avons constaté dans nos essais avec les deux modes de
localisation. L'attente est déclenchée par un événement, sans délai fixe.

## 2. Installation

```
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-depth-image-proc
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

`rosbag2` fait partie de ROS 2 Jazzy desktop. RViz ne sert que sur un poste
avec écran (`rviz:=true`). `depth_image_proc` ne sert que sur le robot réel,
pour ses caméras de profondeur.

Les cartes vont dans `$FOSA_MAPS_DIR`, sinon dans `~/fosa_maps`. **Sur le robot,
le compte est partagé entre les équipes** : on y passe par les scripts de
`deploy/`. Ils font tout tourner en mémoire vive (`FOSA_MAPS_DIR=/dev/shm/fosa/maps`)
et effacent tout en fin de créneau (`docs/ROBOT_REEL.md`, partie 4).

## 3. Test dans Gazebo (café officiel)

### 3.1 Cartographier

Terminal 1 :

```
ros2 launch parc_robot_bringup task.launch.py
```

Terminal 2 :

```
ros2 launch caytu_nav_bringup mapping.launch.py gazebo:=true map_name:=cafe_test rviz:=true
```

Terminal 3, pour piloter (garder ce terminal au premier plan) :

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.6
```

1. **Ne bougez pas** avant le message `Départ noté : (0.00, 0.00, cap 0.00)`.
   Cette pose est l'origine de la carte, et le robot devra repartir d'ici pour
   s'y localiser.
2. Roulez lentement (0,2 m/s) : faites d'abord le tour de la salle, puis
   passez entre les tables.
3. Revenez au point de départ, pour que slam_toolbox puisse refermer la boucle.
4. Arrêtez le robot, puis, dans un terminal 4 :
   ```
   ros2 run caytu_nav_solution save_map
   ```
   Réponse attendue : `Carte enregistrée dans ~/fosa_maps/cafe_test : zone explorée ...`
5. Ctrl-C dans le terminal 2.

Toutes les 30 s, une sauvegarde complète est aussi écrite dans
`~/fosa_maps/cafe_test/auto` ; ce dossier peut servir tel quel de carte.
Relancer avec le même `map_name` n'écrase rien : l'ancienne carte est
déplacée dans `cafe_test_ancienne_<date>`.

### 3.2 Vérifier la carte

Le dossier doit contenir `map.pgm`, `map.yaml`, `map.posegraph`, `map.data` et
`fosa_map.yaml`. Ensuite :

```
python3 src/fosa-solution/tools/compare_maps.py ~/fosa_maps/cafe_test
```

L'outil donne la part des cellules occupées qui tombent à moins de 10 cm d'un
vrai mur. Les tables et les chaises ne figurent pas dans la carte de référence,
donc 100 % n'est pas atteignable. Dans notre simulateur 2D, nous obtenons
92,6 %. Un score nettement plus bas indique une carte décalée ou dédoublée.

### 3.2 bis Si la carte est dédoublée : trouver le moment où elle décroche

Dans Gazebo, nos premières cartes du café avaient des murs dédoublés, tournés
de 6° à 15°. Notre simulateur 2D ne reproduit pas ce défaut, même avec une
odométrie bien plus fausse que celle de Gazebo. La cause est donc à chercher
dans Gazebo lui-même. Pour la trouver, pendant la cartographie, dans un
terminal de plus, **avant de bouger le robot** :

```
python3 src/fosa-solution/tools/diag_slam.py --ros-args -p use_sim_time:=true \
    -p csv:=diag_slam.csv | tee diag_slam.log
```

Chaque seconde, l'outil affiche l'écart entre la position de slam_toolbox et
la vraie pose du robot dans Gazebo (`/sitoe_robot/pose`), la dérive du cap de
l'odométrie des roues et la commande en cours. Une ligne `SAUT` marque une
hausse brusque :
- dans un virage, l'odométrie ou le scan est en cause ;
- au retour dans une zone déjà vue, c'est une fermeture de boucle ratée ;
- dès le départ, c'est la configuration (TF, filtres).

Lancez aussi la cartographie avec `record:=true` : l'enregistrement contient
maintenant la vraie pose Gazebo. Envoyez `diag_slam.csv`, `diag_slam.log`,
l'enregistrement et le dossier de la carte.

Essai possible, à comparer avec `compare_maps.py` : ajouter `imu_odom:=true`
au lancement. slam_toolbox prend alors l'odométrie corrigée par l'IMU
(distance des roues, cap de l'IMU), au lieu de l'odométrie des roues dont le
cap est faux d'environ 15 % dans Gazebo.

### 3.3 Naviguer sur la carte générée

Relancez Gazebo pour que le robot revienne au spawn, puis :

```
ros2 run caytu_nav_solution task_solution.py --ros-args -p slam_map:=$HOME/fosa_maps/cafe_test
```

Messages attendus, dans l'ordre :

1. `Carte « cafe_test » : but Gazebo (-2.293, 2.232) = (9.718, 2.091) dans map`
2. `Carte enregistrée utilisée : .../map.yaml`
3. `TF map -> base_footprint disponible après X s : démarrage de Nav2.`
4. `BUT ATTEINT en X s ...`

## 4. Sur le vrai robot (créneau de 15 minutes)

Avant le créneau, vérifiez que slam_toolbox et teleop_twist_keyboard sont
installés sur le robot ; les organisateurs doivent l'accepter. Préparez
`deploy/robot.env` (adresse du robot, espace de travail de base). Au début du
créneau, lancez `deploy/deploy.sh` puis `deploy/connect.sh` : les commandes
ci-dessous se tapent dans ce terminal.

Pendant le créneau :

1. Vérifications (environ 5 min) : la liste de `docs/ROBOT_REEL.md`, partie 3
   (point d'entrée `/cmd_vel`, caméra haute, hauteur de `base_link`).
2. Placez le robot **sur la position de départ officielle**, puis lancez :
   ```
   ros2 launch caytu_nav_bringup mapping.launch.py map_name:=arene_dakar record:=true
   ```
   Sans `gazebo:=true`, le profil `real` est pris et tout tourne en temps réel.
   `record:=true` enregistre le lidar, l'odométrie, les IMU des caméras et la TF
   dans `$FOSA_MAPS_DIR/enregistrements/`. Pilotage :
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
   ```
   Sur le robot réel, le pilotage publie sur `/cmd_vel`, son nom par défaut :
   pas de redirection.
   Ajoutez `record_camera:=true` pour garder aussi l'image de la caméra haute
   (lourd, mais utile pour mettre au point la détection des tags).
3. Pilotez comme dans Gazebo. La cartographie note aussi la position des
   AprilTags (`tags:=true` par défaut) dans `<carte>/tags.yaml` : passez à
   1–2 m de chaque tag, face à lui, et arrêtez-vous 2 secondes (les vues
   prises en tournant vite ne sont pas gardées). Les journaux annoncent
   « Tag 7 repéré ». Revenez au départ, puis lancez `save_map`.
   Ces positions servent d'indices pour la course (docs/COURSE.md) : la
   course revoit toujours chaque tag avant de le déclarer visité.
4. Fin du créneau : `deploy/clean.sh` sur le portable rapatrie la carte et
   l'enregistrement dans `robot_data/maps/`, puis efface tout du robot.

Retouches (dans RViz, l'outil Publish Point donne les coordonnées :
`ros2 topic echo /clicked_point`) :

```
ros2 run caytu_nav_solution edit_map $FOSA_MAPS_DIR/arene_dakar \
    --erase 1.2 0.4 1.9 1.1       # table qui sera déplacée vendredi
    --wall -0.3 -0.3 4.8 -0.25    # bord ouvert de l'arène
```

Ces retouches ne modifient que `nav_map.*`, que la navigation utilise ensuite.
La carte brute et le graphe restent intacts ; `--reset` repart de la carte brute.

## 5. Mettre au point sans le robot (rejeu)

```
ros2 launch caytu_nav_bringup mapping.launch.py replay:=true robot:=real map_name:=rejeu_1
ros2 bag play $FOSA_MAPS_DIR/enregistrements/arene_dakar_<date> --clock --remap /tf:=/tf_bag
# à la fin du rejeu :
ros2 run caytu_nav_solution save_map
```

Le rejeu recalcule toute la chaîne (filtres lidar, puis slam_toolbox) avec les
réglages du moment. On peut donc régler autant de fois qu'on veut sur les
vraies données.

| Réglage | Où | Quand le toucher |
|---|---|---|
| `max_laser_range` | `slam_mapping.yaml`, ou `max_laser_range:=` au lancement | Le lidar voit des choses hors de l'arène (murs de la salle, public) : le baisser |
| `minimum_travel_distance` / `_heading` | `slam_mapping.yaml` | Laisser à 0,5. À 0,1, la carte a sauté de 1 m dans nos essais |
| Secteurs du robot | `laser_filter_params_real.yaml` | Le vrai robot apparaît dans son propre scan (points qui le suivent) |
| `floor_line_distance` | lancement | Une ligne de faux obstacles apparaît devant ou derrière le robot (sol) |
| `min_laser_range` | `slam_mapping.yaml` | Des morceaux du robot restent dans la carte |

## 6. Ce que nous avons vérifié

Dans un simulateur 2D du café (murs de `cafe_map`, cinq tables, odométrie au
cap faussé de 15 % comme dans Gazebo, robot penché de 2,5°) :

- tests unitaires (`pytest`) et compilation `colcon` sous ROS 2 Jazzy,
  slam_toolbox 2.8.5.
- Cartographie d'un parcours de 39 m : écart moyen de 2 cm entre la position
  estimée et la vraie (12 cm au plus). 98 % des cellules occupées sont à
  moins de 7,5 cm d'un vrai obstacle.
- Navigation sur la carte générée : but atteint en 30 s, à 0,14 m du centre,
  sans contact. Écart de localisation de 2 cm au plus.
- Phase 1, sans carte enregistrée : but atteint en 30 s, à 0,12 m du centre.
  Sans l'attente de la TF, Nav2 ne démarrait pas dans ce même simulateur.
- Enregistrement pendant la cartographie (lidar, odométrie, IMU, TF), puis
  rejeu : la carte est recalculée et l'ancienne TF map -> odom est écartée.
- Ctrl-C ferme tous les nœuds proprement.

**Pas encore vérifié** : Gazebo lui-même (caméras, vrai URDF, temps simulé lent)
et le vrai robot. La partie 3 de ce document est cette vérification.

## 7. Limites connues

- Pour naviguer sur une carte, le robot doit partir de la pose notée au début
  de la cartographie (`start_pose` dans `fosa_map.yaml`). Cartographiez depuis
  la position de départ officielle.
- `task_solution.py` vise encore un seul but lu dans `task_params.yaml`. Les
  tags, leur ordre et le statut des organisateurs sont la prochaine étape.
  Sur le vrai robot, ajoutez `-p use_sim_time:=false`.
- Si les cartes préparées sont interdites vendredi, la même chaîne servira à
  cartographier en direct (slam_toolbox en mode cartographie pendant la course,
  avec exploration). Ce mode reste à écrire.
