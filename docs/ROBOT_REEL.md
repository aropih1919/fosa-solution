# Robot réel (Dakar) — topics officiels et adaptation de la solution

Source : relevé officiel PARC du 2 novembre 2026 (ROS 2 Jazzy,
`ROS_DOMAIN_ID=0`, Jetson Orin). Les organisateurs le présentent comme un
instantané, sans garantie. On le recontrôle donc au premier créneau (partie 3).

Tout passe par un **profil de robot**, dans `caytu_nav_bringup/config/robots/` :
- `sim.yaml` : Gazebo, phase 1, profil par défaut ;
- `real.yaml` : robot de Dakar, solution lancée sur le robot ;
- `real_remote.yaml` : robot de Dakar, solution lancée sur le portable, sans
  les caméras de profondeur (partie 4).

Le profil est choisi avec `robot:=real` ou `robot:=real_remote`
(solution_bringup, task_solution, mapping). Sans cet argument,
`mapping.launch.py` prend `sim` avec `gazebo:=true`, `real` sinon.

## 1. Ce qui change entre Gazebo et le robot réel

| Besoin | Gazebo (phase 1) | Robot réel | Dans la solution (profil `real`) |
|---|---|---|---|
| Commande | `/robot_base_controller/cmd_vel_unstamped` (Twist) | `/cmd_vel` (Twist), lu par le nœud de sécurité `obstacle_avoidance`, qui commande la base sur `/botwheel_explorer/cmd_vel` | Nav2, `task_solution` et le pilotage au clavier publient sur `/cmd_vel` |
| Odométrie | `/odom` | `/botwheel_explorer/odom` | slam_toolbox et Nav2 lisent la TF ; `localisateur_odom_imu` prend le topic du profil |
| Arbre TF | `odom → base_footprint → base_link` | `odom → base_link`, **sans `base_footprint`** | on publie `base_link → base_footprint` (au sol) |
| Lidar | `/scan` (`lidar_link`) | `/scan` (`rplidar_node`, repère `laser`) | inchangé |
| Lidar filtré | notre `/scan_filtered` | `/scan_filtered` **appartient aux organisateurs** (alimente leur détecteur d'obstacles) | le nôtre devient `/fosa/scan_filtered`, avec un filtre propre au vrai robot |
| IMU | `/imu`, axes du robot | seulement `/camera1/camera1/imu` et `/camera2/camera2/imu`, dans le repère optique des caméras | pas d'IMU : sol supposé plat, cap donné par l'odométrie, que slam_toolbox corrige |
| Caméras de profondeur | nuages `/top_camera_depth/points`, `/bottom_camera_depth/points` | images `/cameraN/cameraN/depth/image_rect_raw`, **pas de nuages** | `depth_image_proc` → `/fosa/top_camera/points`, `/fosa/bottom_camera/points`, puis même filtrage en hauteur |
| Carte | — | aucun repère `map` | fourni par slam_toolbox |

Sans `depth_image_proc`, la solution démarre quand même, avec le lidar seul,
et le signale dans les journaux.

## 2. Ce qu'il ne faut jamais faire sur le robot

- **Publier sur un topic des organisateurs** : `/scan_filtered`, `/scan_merged`,
  `/camera1/scan`, `/camera2/scan`, `/goal`, `/nav_goal`,
  `/botwheel_explorer/cmd_vel`, `/stamped_vel`, `/safe_vel`, `/auto_vel`,
  `/status_led`. La liste est dans `real.yaml` (`reserved_topics`), et un test
  vérifie qu'aucun topic de la solution n'y figure.
  - `/goal` est lu par `obstacle_avoidance`.
  - Dans RViz, l'outil « 2D Goal Pose » publie sur `/goal_pose` (Nav2) : il
    n'y a pas de conflit.
- **Faire tourner Gazebo sur un portable en `ROS_DOMAIN_ID=0` sur le même
  Wi-Fi que le robot.** Le robot est sur le domaine 0 : le `/scan`, la `/tf` et
  le `/cmd_vel` du simulateur arriveraient sur le robot. Sur les portables qui
  simulent : `export ROS_DOMAIN_ID=42` (ou une autre valeur). Ne gardez 0 que
  pour parler au robot.
- **Modifier les paramètres des nœuds des organisateurs**, même si leurs
  services `set_parameters` sont visibles.

## 3. À vérifier au premier créneau (environ 5 minutes)

1. **Point d'entrée de la commande.** Demandez confirmation aux organisateurs
   que les équipes publient sur `/cmd_vel`. Ensuite :
   ```
   ros2 topic info /cmd_vel -v          # abonné attendu : obstacle_avoidance
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
   ```
   Le robot doit bouger. Observez aussi s'il ralentit ou s'arrête seul près
   d'un meuble : c'est `obstacle_avoidance`. Si les organisateurs donnent un
   autre topic, changez `cmd_vel_topic` dans `real.yaml`.
2. **Quelle caméra est la haute ?**
   ```
   ros2 run tf2_ros tf2_echo base_link camera1_link
   ros2 run tf2_ros tf2_echo base_link camera2_link
   ```
   Le profil suit le README officiel PARC du 5 novembre : camera1 est la
   haute, camera2 la basse. Si les hauteurs (z) disent l'inverse, échangez
   `top` et `bottom` dans `real.yaml` et les topics `top_*` / `bottom_*` dans
   `robot_reel.yaml`.
3. **Hauteur de `base_link` au-dessus du sol.**
   ```
   ros2 run tf2_ros tf2_echo base_link laser
   ```
   Mesurez au mètre la hauteur du centre du lidar au-dessus du sol. Ensuite :
   `base_link_height = hauteur mesurée − z de laser`. Reportez la valeur dans
   `real.yaml`. Une erreur de quelques centimètres décale d'autant la bande de
   hauteur des caméras, sans plus.
4. **Le lidar voit-il le robot ?** Dans RViz ou dans l'enregistrement, des
   points de `/fosa/scan_filtered` qui suivent le robot au-delà de 0,20 m
   doivent être retirés par des secteurs dans `laser_filter_params_real.yaml`.
5. **Le sol.** Une ligne de faux obstacles à distance constante dans
   `/scan_clean` signifie que le lidar touche le sol. Forcez alors
   `floor_line_distance:=<distance>`.
6. **Paquets.** `ros2 pkg prefix slam_toolbox depth_image_proc teleop_twist_keyboard`.

## 4. Travailler sur le robot par SSH

Le compte du robot est partagé entre toutes les équipes. Sous un même
utilisateur, chacun peut lire tous les fichiers du compte : les droits Unix ne
protègent rien. Consigne des organisateurs : ne pas téléverser le code source.
Deux façons de travailler en découlent.

| | **Mode A — à distance (recommandé)** | **Mode B — en mémoire sur le robot** |
|---|---|---|
| Où tourne la solution | Sur le portable | Sur le robot, dans `/dev/shm` |
| Ce qui va sur le robot | Rien de l'équipe | Les sept paquets ROS de la solution (sans tests, outils ni modèles d'IA), le temps du créneau |
| Respecte « pas de code source » | Oui | Non : à n'utiliser que si les organisateurs l'autorisent |
| Rôle de SSH | Surveiller, enregistrer, arrêter | Lancer et surveiller |
| Point faible | Le Wi-Fi : latence, coupures, horloges à caler | Le code est lisible pendant le créneau |
| Caméras de profondeur | Non (trop lourdes pour le Wi-Fi) : lidar seul | Oui |

Le guide officiel demande de faire tourner le code sur le robot, et précise
que piloter depuis un ordinateur par le réseau est « autorisé mais
déconseillé ». Le mode A respecte la consigne de ne rien téléverser, au prix
de ce que le guide déconseille. **Faites confirmer par les organisateurs :**
1. le mode A (nœuds sur le portable par le Wi-Fi) est-il accepté pendant les
   essais ?
2. le mode B (code en mémoire pendant le créneau, effacé ensuite) est-il
   accepté ?
3. pour vendredi, comment le code soumis est-il chargé et protégé ?
4. la base s'arrête-t-elle seule si `/cmd_vel` n'arrive plus (par exemple au
   bout d'une demi-seconde) ? En mode A, une coupure Wi-Fi ne doit pas laisser
   le robot rouler sur sa dernière commande.

### Une fois, sur le portable (les deux modes)

```
cp deploy/robot.env.example deploy/robot.env
#   ROBOT=<compte>@<adresse du robot>, ROBOT_BASE_SETUP=<setup.bash de base>
```

`deploy/robot.env` et `robot_data/` ne sont pas versionnés. Le mot de passe
n'est écrit nulle part : SSH le demande une fois par créneau, la connexion
étant réutilisée par tous les scripts. Jamais de `ssh -A`. Jamais de git, de
jeton ni de clé privée sur le robot.

### Mode A — la solution tourne sur le portable

Le robot garde seulement ses propres programmes (pilotes, lidar, odométrie).
Le portable, sur le même Wi-Fi, rejoint son réseau ROS 2 et fait tout le reste :
filtres du lidar, slam_toolbox, Nav2, task_solution. Le profil
`robot:=real_remote` est le robot réel sans les caméras de profondeur. Leurs
images (environ 25 Mo/s chacune) ne passent pas par le Wi-Fi : la navigation
se fait au lidar seul.

Portable : ROS 2 Jazzy avec Nav2, slam_toolbox, laser_filters et
teleop_twist_keyboard. Rien à installer sur le robot.

**Début de créneau :**

```
export ROS_DOMAIN_ID=0                 # domaine du robot (relevé officiel)
# caytu_config du robot réel : temps réel, /cmd_vel, caméras du robot.
export FOSA_CONFIG_OVERRIDE=$PWD/caytu_config/config/robot_reel.yaml
deploy/check_link.sh
```

`check_link.sh` n'envoie rien sur le robot. Il mesure :
- le Wi-Fi : pertes et latence ;
- l'écart entre l'horloge du portable et celle du robot, qui doit rester sous
  50 ms, sinon Nav2 et la TF rejettent les messages. Le script donne la
  commande pour caler le portable ;
- le domaine ROS et le middleware des deux côtés ;
- la réception de `/scan` et de l'odométrie sur le portable.

Si le portable ne voit pas les topics du robot :
`export ROS_STATIC_PEERS=<adresse du robot>`.

**Pendant le créneau, sur le portable :**

```
# Cartographie
ros2 launch caytu_nav_bringup mapping.launch.py robot:=real_remote map_name:=arene_dakar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
ros2 run caytu_nav_solution save_map            # la carte reste sur le portable

# Navigation sur la carte
ros2 run caytu_nav_solution task_solution.py --ros-args -p robot:=real_remote \
    -p slam_map:=$HOME/fosa_maps/arene_dakar -p task_params_file:=<fichier>
```

**Enregistrer sur le robot**, pour avoir des données complètes même si le
Wi-Fi coupe. Seule la commande publique `ros2 bag` est utilisée, en mémoire :

```
deploy/robot_record.sh start arene_dakar    # --camera pour l'image de la caméra haute
deploy/robot_record.sh stop                 # arrête proprement et rapatrie
```

**Fin de créneau :** `deploy/clean.sh` efface ce qui reste sur le robot
(enregistrements, journaux).

**Surveiller ou arrêter par SSH :** `ssh <robot>` puis les commandes ROS
publiques : `ros2 topic hz /scan`, `ros2 run tf2_ros tf2_echo odom base_link`.
Pour arrêter le robot si le portable plante :
`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"` pendant
quelques secondes.

**Précautions propres au mode A :**
- portable près de la borne Wi-Fi ;
- vitesses basses : 0,3 m/s au plus tant que la question 4 n'est pas réglée ;
- aucun autre nœud ROS sur le portable, et pas de RViz avec affichage des
  caméras ;
- testé dans notre simulateur sur une seule machine, donc **sans** la latence
  ni les coupures d'un vrai Wi-Fi.

### Mode B — la solution tourne sur le robot, en mémoire

À n'utiliser que si les organisateurs acceptent le code en mémoire pendant le
créneau. Tout vit dans `/dev/shm/fosa`, que le redémarrage du robot entre deux
créneaux efface : **on redéploie à chaque créneau**. Les journaux ROS, les
fichiers temporaires, les cartes et, s'il faut l'installer, faster-whisper y
vont aussi ; l'historique bash est coupé ; tout est effacé en fin de créneau.
Limite : pendant le créneau, ce dossier reste lisible par le même compte.

Deux endroits, deux sortes de commandes :

| Où | Quoi |
|---|---|
| **Portable**, dossier du dépôt | les scripts `deploy/*.sh` uniquement : ils parlent au robot par SSH |
| **Terminal du robot**, ouvert par `deploy/connect.sh` | les commandes `ros2 launch` / `ros2 run` des guides, avec `robot:=real` |

`run.sh` et `install.sh` ne servent que sur un PC (ils supposent le dépôt
complet) : ils ne partent pas sur le robot. Dans le terminal du robot,
`FOSA_CONFIG_OVERRIDE` (robot_reel.yaml), `FOSA_MAPS_DIR` et l'espace de
travail compilé sont déjà chargés par `env.sh`.

**1. Début de créneau, sur le portable :**

```bash
deploy/deploy.sh --map arene_dakar    # ou sans --map au tout premier créneau
deploy/connect.sh                     # terminal tmux sur le robot
```

`deploy.sh` envoie caytu_config, caytu_interfaces, caytu_semantic_map,
caytu_perception, caytu_assistant, caytu_nav_bringup et caytu_nav_solution
(sans tests, outils ni `config/models/`), les compile (environ 20 s dans
notre essai, plus sur le Jetson), puis liste ce qui manque au robot :
« INDISPENSABLE absent » bloque la course, à régler avec les organisateurs
(dont `apriltag_ros`, le détecteur des tags) ; « Optionnel absent » ne
concerne que `voix:=true` ou le secours `detecteur:=opencv`.

**2. Dans le terminal du robot** (`Ctrl-b c` pour un deuxième terminal) :

```bash
# Cartographie + tags (premier créneau)
ros2 launch caytu_nav_bringup mapping.launch.py robot:=real map_name:=arene_dakar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
ros2 run caytu_nav_solution save_map

# Repérage des tags sur la carte enregistrée (si le temps le permet)
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar

# Répétition de la course
ros2 launch caytu_nav_bringup course.launch.py robot:=real \
    carte:=$FOSA_MAPS_DIR/arene_dakar voix:=false ordre:=7,1,9,4
ros2 run caytu_nav_solution depart            # deuxième terminal
```

Mot de départ, si faster-whisper manque au robot et que le robot a Internet :
`pip install --target "$FOSA_ROOT/py" faster-whisper`, puis
`python3 -c "from faster_whisper import WhisperModel; WhisperModel('base', device='cpu', compute_type='int8')"`
(modèle téléchargé dans `$FOSA_ROOT/hf`). Tout est en mémoire : à refaire à
chaque créneau, et effacé par `clean.sh`.

**3. Fin de créneau, sur le portable :**

```bash
deploy/clean.sh       # rapatrie les cartes dans robot_data/maps/, arrête tout, efface /dev/shm/fosa
```

Ne jamais finir un créneau sans `clean.sh` : la carte et les `tags.yaml`
faits pendant le créneau n'existent que sur le robot jusqu'à ce qu'il les
rapatrie (`deploy/fetch.sh` le fait aussi en cours de route).

**Vérifié** (simulateur, robot simulé par un compte SSH local) :
`deploy.sh --map` envoie et compile les sept paquets en 20 s sur un ROS sans
notre code ; la course lancée par SSH depuis l'espace déployé, ordre
`9,4,7,1`, visite les 4 tags (distances vraies 0,49 à 0,52 m, alignement
à 3,2° au plus, fin à 262 s, aucun contact) ; `clean.sh` rapatrie la carte,
arrête tout et efface le dossier. Pas vérifié : le vrai robot (Jetson, son
`~/.bashrc`, ses paquets installés, le temps de compilation).

### Ce qui peut fuir malgré tout

Pendant votre créneau, quiconque est sur le même Wi-Fi avec
`ROS_DOMAIN_ID=0` voit vos nœuds, vos topics (la carte, `/map`) et peut lire
leurs paramètres (`ros2 param dump`). C'est vrai dans les deux modes. Demandez
aux organisateurs si les autres équipes sont sur ce réseau pendant les
créneaux.

## 5. Interfaces disponibles, pas encore utilisées

- **Démarrage vocal** : `/audio`, `/audio/channel0` à `5`, `/speech_audio`,
  `/is_speeching` (`respeaker_node`, `audio_common_msgs/AudioData`).
  L'encodage et la fréquence d'échantillonnage ne figurent pas dans le relevé.
- **AprilTag** : `/camera1/camera1/color/image_raw` et `color/camera_info`
  (caméra haute selon le README officiel du 5 novembre ; à confirmer au point 2
  de la partie 3).
- **Arbitre officiel** (`parc_referee`, paquet `parc_msgs` déjà installé dans
  `~/ros2_ws`) : `/parc/tag_order`, `/parc/run_status`, `/parc/start`,
  `/parc/tag_visit`, `/parc/finish`. Voir `deploy/README.md`, partie
  « Arbitre ».

## 6. Ce que nous avons vérifié

Dans un simulateur 2D du café réglé sur les interfaces du relevé :
`/botwheel_explorer/odom`, TF `odom → base_link` sans `base_footprint`, lidar
dans `laser`, aucune IMU de base, commande sur `/cmd_vel`, temps réel, et une
image de profondeur par caméra (camera1 en haut depuis le 5 novembre).

- Cartographie avec `record:=true` (profil `real` choisi seul) :
  - écart moyen de 1 cm entre position estimée et vraie (8 cm au plus) ;
  - rien n'est publié sur `/scan_filtered`, le nôtre sort sur `/fosa/scan_filtered` ;
  - `base_footprint` est bien publié sous `base_link` ;
  - l'enregistrement contient les topics du profil réel.
- Navigation sur cette carte avec `task_solution -p robot:=real` : but atteint
  en 37 s, sans contact, 1 cm d'écart de localisation.
- Chaîne caméra : `depth_image_proc` → `/fosa/top_camera/points` (10 Hz) →
  `/top_camera_scan` (10 Hz).
- tests `pytest`, dont la vérification qu'aucun topic de la solution
  n'est réservé aux organisateurs.
- Mode A : `task_solution -p robot:=real_remote` sur une carte enregistrée,
  but atteint en 40 s, sans contact ni nœud caméra (simulateur et solution sur
  la même machine, donc sans latence Wi-Fi). `check_link.sh` et
  `robot_record.sh` testés contre un faux robot : enregistrement fait sur le
  robot, arrêté proprement et rapatrié.
- Scripts `deploy/` contre un faux robot (serveur SSH local, ROS 2 Jazzy) :
  - déploiement et compilation en 3 s ;
  - cartographie lancée par SSH, carte enregistrée dans `/dev/shm/fosa` ;
  - aucun journal ni fichier temporaire hors de ce dossier ;
  - environnement correct dans la session tmux ;
  - `clean.sh` rapatrie la carte, arrête les nœuds et efface le dossier.

Messages sans conséquence vus pendant ces essais :
- slam_toolbox peut s'arrêter avec `process has died ... context cannot be
  slept` au Ctrl-C, une fois la carte enregistrée ;
- le planificateur Smac a affiché une fois « Inflation layer either not found
  ... », avec une configuration Nav2 identique aux autres essais. Le robot a
  atteint son but.

**Pas vérifié** : le vrai robot. Les points de la partie 3 restent à
contrôler sur place.
