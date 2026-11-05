# Simulation Fosa : tout essayer sans le robot

Ce dossier rejoue toute la solution de la phase finale sans le robot :
- cartographie et position des tags ;
- ordre des tags publié par les « organisateurs » ;
- départ, course, visites à 0,5 m avec image ;
- personne qui traverse, tag absent ;
- interfaces du vrai robot ;
- mot de départ.

Un **arbitre** juge chaque course sur la **vraie** position du robot, comme
les juges, et le banc écrit un rapport « RÉUSSI / ÉCHEC » par scénario.

Deux simulateurs :

| | Simulateur 2D (`sim2d/`) | Gazebo PARC |
|---|---|---|
| Lancement | rien à installer en plus, tout automatique | votre monde Gazebo habituel |
| Monde | café PARC en 2D, 4 panneaux à tags | votre monde, tags posés par `tools/tags_fictifs.py` |
| Caméra | image synthétique (tags en perspective) | rendu Gazebo |
| Interfaces du vrai robot (`robot:=real`) | **oui** (`--reel`) | non |
| Durée d'une course | ~5 min | ~5 min (plus le démarrage de Gazebo) |
| Usage | essais complets, répétables, le soir, en série | confirmer dans le vrai simulateur PARC |

## 1. Prérequis

Dans chaque terminal :
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash        # votre workspace, compilé (colcon build)
cd ~/ros2_ws/src/fosa-solution
```

| Pour | Il faut |
|---|---|
| Tout | ROS 2 Jazzy, Nav2, slam_toolbox, nav2_collision_monitor, laser_filters, pointcloud_to_laserscan, `apriltag_ros` (détecteur des tags), python3 : numpy, yaml, OpenCV avec aruco (le simulateur 2D dessine les tags avec) |
| `2d reel` | `depth_image_proc` (`sudo apt install ros-jazzy-depth-image-proc`) |
| `2d arbitre` | `depth_image_proc` ; `parc_msgs` et `parc_referee` compilés (dépôt PARC2026-Engineers-League à jour, `./install.sh` le fait) ; port 8080 libre (`FOSA_ARBITRE_PORT` pour changer) |
| Gazebo | Gazebo Harmonic et la commande `gz`, le monde PARC (`parc_robot_bringup`) |
| Voix | `ros-jazzy-audio-common-msgs`, `pip install faster-whisper` + modèle téléchargé, `espeak-ng` (facultatif) |

## 2. Démarrage rapide

```bash
simulation/banc.sh 2d tout
```

Durée : environ 45 minutes, sans rien toucher. Le banc enchaîne :
1. la cartographie ;
2. le repérage des tags ;
3. une course ;
4. une course avec une personne ;
5. une course sans le tag 9 ;
6. une course avec les interfaces du vrai robot.

À la fin, il affiche le rapport. Rapport et journaux :
`simulation/resultats/<date>_2d_tout/` (un sous-dossier par scénario).

Exemple de rapport :
```
[carte]      RÉUSSI  carte .../cartes/sim2d, 4 tags notés
               tag 1 : noté à 0.12 m de sa vraie place
[course    ] RÉUSSI  tags valides [7, 1, 9, 4] (ordre [7, 1, 9, 4]) ; fin 330 s ; immobile avant départ oui ; contacts 0
               VISITE tag 7 à 114 s : distance vraie 0.509 m, alignement +4.7° -> VALIDE
```

Le banc rend 0 si tout est réussi, 1 sinon.

## 3. Les scénarios

### 3.1 Simulateur 2D : `simulation/banc.sh 2d <scénario>`

| Scénario | Ce qui tourne | Ce qui est jugé |
|---|---|---|
| `carte` | `mapping.launch.py` (+ tags), robot conduit par `sim2d/conduite_auto.py` à la place du clavier, puis `save_map` | carte enregistrée ; 4 tags notés ; écart de chaque tag noté avec sa vraie place |
| `reperage` | `reperage_tags.launch.py` sur la carte, même parcours | 4 tags repérés ; écarts avec la vérité |
| `course` | `course.launch.py robot:=sim`, ordre publié, départ par `depart` | voir « Ce que juge l'arbitre » |
| `personne` | comme `course`, une personne fait des allers-retours devant le départ (0,4 m/s) | idem, plus distance minimale robot–personne, aucun contact |
| `tag_absent` | comme `course`, sans le tag 9 (`--sans-tag ID` pour un autre) | 3 visites valides, le tag absent sauté, fin avant 10 min |
| `reel` | `course.launch.py robot:=real` avec `robot_reel.yaml` ; le simulateur imite le vrai robot (voir 4) | idem `course` |
| `arbitre` | comme `reel`, jugé en plus par l'arbitre officiel `parc_referee` : armé par son API (comme le juge), ordre sur `/parc/tag_order`, départ `--depart commande` (défaut) ou `--depart voix` (le mot « allez » tel que le publie `mot_depart`) | idem `course`, et pour l'arbitre : `FINISHED`, aucun faux départ, chaque visite « in order », « image match », « hold ok » |
| `tout` | les six, dans l'ordre | |

**Ce que juge l'arbitre** (`outils/verifier_course.py`) :
- le robot reste immobile avant le départ (10 s d'attente après « Prêt ») ;
- chaque visite annoncée : centre du robot à 0,5 m ± 0,10 m du tag, face à lui à ± 12° ;
- les tags dans l'ordre publié ;
- la fin annoncée avant 10 min ;
- aucun contact.

`carte` doit avoir tourné une fois : les autres scénarios réutilisent sa carte
(`simulation/resultats/cartes/sim2d`), ou celle donnée par `--carte`.

### 3.2 Gazebo PARC : `simulation/banc.sh gazebo course --carte DIR`

Le monde PARC n'a pas de tags. On les pose après la cartographie.

```bash
# Terminal 1 : Gazebo PARC
./run.sh gazebo

# Terminal 2, une seule fois : carte faite dans Gazebo
./run.sh carte gazebo:=true map_name:=cafe_test
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.6
ros2 run caytu_nav_solution save_map

# Terminal 2, avant chaque course : Gazebo relancé, robot au départ
simulation/banc.sh gazebo course --carte ~/fosa_maps/cafe_test
simulation/banc.sh gazebo course --carte ~/fosa_maps/cafe_test --personne      # une personne traverse
simulation/banc.sh gazebo course --carte ~/fosa_maps/cafe_test --sans-tag 9    # tag absent
```

Le banc :
- pose les 4 tags (`tools/tags_fictifs.py`) ;
- lance l'arbitre avec leurs vraies positions (`<carte>/tags_fictifs/verite.yaml`) ;
- publie l'ordre, lance la course et donne le départ ;
- avec `--personne`, fait marcher une personne (`outils/obstacle_gazebo.py`) sur
  un segment de 3 m qui coupe le chemin 1,5 m devant le départ.

La vraie pose vient de `/sitoe_robot/pose`, publiée par le monde PARC.

**Avant chaque course, relancez Gazebo** : le robot doit partir du point de
départ de la cartographie.

Pour un repérage manuel dans Gazebo (comme au créneau) :
```bash
python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=sim carte:=$HOME/fosa_maps/cafe_test
# conduire devant chaque tag, puis :
python3 tools/tags_fictifs.py ~/fosa_maps/cafe_test --comparer
```

### 3.3 Mot de départ : `simulation/banc.sh voix [dossier]`

```bash
python3 simulation/outils/generer_audios.py simulation/audios     # jeu d'essai
# Vos voix, la vraie référence (2 s chacune, nommage oui_ / non_) :
arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/oui_allez_hasina.wav
arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/non_bonjour_hasina.wav
simulation/banc.sh voix simulation/audios
```

Chaque fichier est publié comme un segment du ReSpeaker sur `/speech_audio`,
vers un nœud `mot_depart` neuf :
- `oui_*.wav` doit donner le départ ;
- `non_*.wav` ne doit pas (autres mots, phrase trop longue, bruit,
  applaudissements, silence).

Le résultat dit ce que Whisper a entendu pour chaque fichier. Les voix de
synthèse (espeak-ng) sont un premier contrôle : Whisper comprend moins bien
une voix de synthèse qu'une vraie voix.

### 3.4 Ordre des organisateurs

Le format réel est connu depuis le 5 novembre : `parc_msgs/TagOrder` sur
`/parc/tag_order`, publié par l'arbitre officiel (`2d arbitre`). Les autres
formats restent acceptés, pour les anciens essais :
```bash
simulation/banc.sh 2d course --type-ordre entiers     # Int32MultiArray
simulation/banc.sh 2d course --type-ordre octets      # UInt8MultiArray
simulation/banc.sh 2d course --ordre aleatoire        # ordre tiré au sort
```
Ordre publié en retard : la course doit attendre sans bouger.
```bash
python3 simulation/outils/ordre_organisateurs.py 9,4,7,1 --retard 30
```

### 3.5 Déploiement sur le robot (mode B), avec deux PC

Le PC « robot » doit avoir ROS 2 Jazzy et un compte SSH :
1. Sur le portable, mettez son adresse dans `deploy/robot.env`.
2. Sur le portable : `deploy/deploy.sh --map <carte>`, puis `deploy/connect.sh`.
3. Dans ce terminal (sur le PC « robot ») :
   ```bash
   python3 <copie du dossier simulation>/sim2d/simulateur.py --reel
   ```
   Le simulateur n'est pas envoyé par `deploy.sh` : copiez le dossier
   `simulation/` à part.
4. Dans un second terminal tmux :
   ```bash
   ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/<carte> voix:=false ordre:=7,1,9,4
   ```
   puis `ros2 run caytu_nav_solution depart`.
5. Sur le portable : `deploy/clean.sh`. Le dossier doit être effacé.

## 4. Le simulateur 2D

`sim2d/simulateur.py` remplace Gazebo et le robot :
- **monde :** murs du café PARC, pieds de tables absents de la carte, 4 panneaux
  portant les tags 1, 4, 7, 9 à 1 m du sol ;
- **lidar :** 540 rayons ;
- **odométrie :** cap faussé de 15 %, comme le DiffDrive de Gazebo ;
- **caméra couleur :** 640×480, tags dessinés en perspective et cachés par les
  murs, à la taille du détecteur (`perception.apriltag.size_m`) ;
- **contacts :** tout contact est écrit « CONTACT ».

| Option | Effet |
|---|---|
| (aucune) | interfaces du Gazebo PARC : `/clock`, `/odom`, `/imu`, `/robot_base_controller/cmd_vel_unstamped`, `/top_camera_color/*`, nuages des caméras, repère `base_footprint` |
| `--reel` | interfaces du **vrai robot** (relevé officiel du 2 novembre) : horloge réelle, `/botwheel_explorer/odom`, TF `odom -> base_link`, `/scan` dans `laser`, pas d'IMU, `/cmd_vel`, `/camera1/camera1/color/*` (caméra haute), images de profondeur `/camera1` et `/camera2` ; tags de 0,165 m |
| `--personne` | une personne (cylindre de 0,25 m) fait des allers-retours devant le départ, loin des tags. Si le robot est sur son chemin, elle attend puis fait demi-tour au bout de 3 s : elle ne le touche jamais, un contact est donc toujours la faute du robot |
| `--sans-tag 9` | le tag 9 n'existe pas |

Vérité terrain publiée : `/sitoe_robot/pose` (comme le Gazebo PARC),
`/sim/true_pose`, `/sim/tags`, `/sim/obstacles`.

**Ce que `2d reel` vérifie de plus que `2d course` :**
- `robot:=real`, `robot_reel.yaml` ;
- le repère `base_footprint` ajouté sous `base_link` ;
- la navigation sans IMU, en temps réel (sans `/clock`) ;
- `/cmd_vel` à travers le garde-fou ;
- les caméras de profondeur converties par `depth_image_proc` ;
- la taille réelle des tags.

**Ce qu'il ne vérifie pas :**
- la hauteur réelle de `base_link` ;
- le nœud `obstacle_avoidance` des organisateurs ;
- les vraies latences ;
- le vrai lidar.

## 5. Les outils, un par un

Tous se lancent seuls, avec `--help`.

| Outil | Rôle |
|---|---|
| `sim2d/simulateur.py` | simulateur 2D (section 4) |
| `sim2d/conduite_auto.py [--reel]` | conduit le robot du simulateur 2D le long d'un parcours qui passe devant les tags (remplace le clavier) |
| `outils/ordre_organisateurs.py ORDRE [--type] [--retard]` | publie l'ordre comme les organisateurs (latched) |
| `outils/verifier_course.py [--verite F] [--sortie F]` | notre vérificateur : juge la course sur la vraie pose |
| `outils/comparer_tags_parc.py CARTE MONDE.json` | simulateur officiel : tags notés (`tags.yaml`) comparés à la vérité du monde (`/tmp/parc_worlds/parc_random_seed<N>.json`) |
| `outils/arbitre_officiel.py armer\|attendre\|etat` | pilote l'arbitre officiel `parc_referee` par son API web (arme la course comme le juge, attend la fin, bilan). Jamais contre l'arbitre des organisateurs |
| `outils/obstacle_gazebo.py CARTE [--de x,y --a x,y]` | personne mobile dans Gazebo |
| `outils/generer_audios.py DOSSIER` | jeu d'essai audio |
| `outils/test_voix.py DOSSIER` | banc du mot de départ |
| `../tools/tags_fictifs.py CARTE` | 4 tags dans Gazebo (voir `docs/COURSE.md`, 2 bis) |

Exemple à la main, simulateur 2D, quatre terminaux :
```bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST     # dans chaque terminal
python3 simulation/sim2d/simulateur.py --personne
python3 simulation/outils/verifier_course.py
python3 simulation/outils/ordre_organisateurs.py 7,1,9,4
ros2 launch caytu_nav_bringup course.launch.py robot:=sim carte:=simulation/resultats/cartes/sim2d voix:=false
ros2 run caytu_nav_solution depart        # une fois « Prêt » affiché
```
RViz (`rviz2`, Fixed Frame `map`) montre la carte, le robot et les tags (`/fosa/tags_marqueurs`).

## 6. Dépannage

| Symptôme | Cause probable |
|---|---|
| « Workspace non chargé » | `source <ws>/install/setup.bash` ; `colcon build` après chaque modification du code |
| « la course n'est jamais prête » | voir `course.log` du scénario : Nav2 qui ne démarre pas, carte absente |
| `2d arbitre` : « Arbitre injoignable » | `parc_referee` pas compilé, ou port 8080 déjà pris (`FOSA_ARBITRE_PORT=8081`) |
| `2d reel` : le robot ne bouge jamais, `collision_monitor` parle de source invalide | une caméra ne publie pas (ou `depth_image_proc` absent) : le garde-fou arrête le robot. **Même effet sur le vrai robot** si une caméra ne publie pas sa profondeur |
| Des programmes restent après un Ctrl-C | `simulation/banc.sh nettoyer` |
| Le banc gêne un robot ou un Gazebo ouvert | le 2D tourne sur `ROS_DOMAIN_ID=77` en local (`FOSA_SIM_DOMAIN` pour changer) |
| Gazebo : « Aucun monde trouvé » | Gazebo lancé dans un autre `GZ_PARTITION`, ou pas encore prêt |

## 7. Ce qui a été vérifié, et les limites

Le 5 novembre, avec `apriltag_ros` (détecteur par défaut) : `2d carte` 4 tags
notés (1 et 7 à 5 cm au plus, 9 à 0,46 m, 4 à 1,94 m comme avec OpenCV),
`2d reperage` idem, `2d arbitre --ordre 9,4,1,7 --depart voix` RÉUSSI pour les
deux arbitres : `FINISHED` à 286 s, 4 × « in order, image match, hold ok »,
bonus vocal 5, aucun contact.

Simulateur officiel PARC (seed 7), cartographie `robot:=parc` + apriltag_ros sur
une machine sans carte graphique (1/15 du temps réel) : premières vues à 8 cm
(tag 7, à 2,8 m) et 19 cm (tag 1, à 5 m) ; un tour complet sur place fait dériver le cap dans la
carte (tag 9 placé à 4 m). Course complète non jouée ici, faute de vitesse :
à refaire sur un PC rapide (`LANCEMENT.md`, 1.3).

Le 5 novembre, `2d arbitre --ordre 9,4,1,7` : RÉUSSI pour notre vérificateur
(4/4, 0,489 à 0,511 m, +1,2° à +4,0°, aucun contact) et pour l'arbitre
officiel (`FINISHED` à 392 s, 4 visites « in order », « image match »,
« hold ok », aucun faux départ). `2d arbitre --ordre 7,1,9,4 --depart voix` :
départ vocal reçu et compté par l'arbitre (bonus 5), tags 7 et 1 validés, puis
robot bloqué ~140 s en route vers le tag 9 : tag 9 sauté, `TIMED_OUT` à 600 s.

Banc lancé sur notre machine le 4 novembre (`2d tout`, ordre 7,1,9,4) :

| Scénario | Résultat |
|---|---|
| `carte` | RÉUSSI : 4 tags notés ; 1, 7, 9 à 6 cm au plus de leur vraie place, **tag 4 à 1,16 m** |
| `reperage` | RÉUSSI : 1, 7, 9 à 6 cm au plus, **tag 4 à 1,96 m** |
| `course` | RÉUSSI : 4/4 dans l'ordre (0,500 à 0,512 m, ±4°), fin à 330 s, aucun contact |
| `reel` (interfaces du vrai robot) | RÉUSSI : 4/4 (0,488 à 0,514 m, ±3,6°), fin à 461 s, aucun contact |
| `tag_absent` | **ÉCHEC** : tag 7 seul ; tag 1 cherché au mauvais endroit jusqu'à 465 s, puis plus de temps (voir ci-dessous) |
| `personne` (ordre en `Int32MultiArray`) | RÉUSSI après correction du simulateur (la personne attendait le robot qui l'attendait) : 4/4 (0,503 à 0,513 m, ±4,3°), fin à 216 s, aucun contact, garde-fou déclenché 8 fois, 8 cm de marge au plus près |

**Faiblesse trouvée par le banc.** Au départ, le robot fait un tour sur
place pour repérer les tags. Pendant ce tour, son cap estimé est en retard
(l'odométrie simulée exagère la rotation de 15 %, comme le Gazebo PARC). Un
tag vu de loin pendant le tour est alors placé à 1 ou 2 m de sa vraie place.

Le registre écarte ces vues, mais pas la copie que reçoit la course
(`/fosa/tags_vus`). La course part alors chercher au mauvais endroit et perd
du temps. Les visites restent justes : chacune est validée sur une vue
fraîche et proche. Mais une course peut ne pas finir à temps. Le même défaut
explique le tag 4 mal placé en cartographie et en repérage.

Signe du même défaut : une même course dure de 216 à 461 s selon les essais,
suivant que le tour du départ place bien ou mal les tags.

Correction proposée, non appliquée : `course_tags` ignore, pour ses
estimations, les vues lointaines prises pendant une rotation rapide. Il
s'arrête quand il voit le tag, puis le revoit à l'arrêt.

Vérifié à part :
- **Gazebo Harmonic 8.10, sans écran :**
  - tags posés par `tools/tags_fictifs.py` et lus par une caméra Gazebo aux poses d'attente et de visite ;
  - personne de `obstacle_gazebo.py` posée, déplacée à 0,4 m/s, retirée au Ctrl-C.
- **Pas vérifié de bout en bout :**
  - `banc.sh gazebo course` : nous n'avons pas le monde PARC ;
  - `banc.sh voix` : ni Whisper ni `audio_common_msgs` ici.

**Aucune simulation ne vérifie :**
- le vrai robot et son problème actuel ;
- la vitesse du Jetson ;
- le Wi-Fi ;
- l'éclairage et le flou sur les vrais tags ;
- le ReSpeaker ;
- le paquet de statut des organisateurs.
