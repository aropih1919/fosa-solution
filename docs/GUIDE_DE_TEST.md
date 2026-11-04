# Guide de test

Ce guide couvre trois niveaux : tests unitaires, course en simulation, puis
créneau sur le robot. Les commandes se lancent depuis la racine du dépôt.

## 1. Tests unitaires

Ils ne demandent ni robot ni Gazebo. ROS 2 Jazzy doit être sourcé.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to caytu_course
colcon test --packages-select caytu_config caytu_perception caytu_course caytu_nav_solution
colcon test-result --verbose
```

Sans installation, paquet par paquet :

```bash
source /opt/ros/jazzy/setup.bash
cd caytu_config
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=$PWD:$PYTHONPATH \
    python3 -B -m pytest -p no:cacheprovider -q test
```

Répéter dans `caytu_perception`, `caytu_course` et `caytu_nav_solution` en
ajoutant les paquets voisins au `PYTHONPATH` (`../caytu_config`,
`../caytu_perception`, `../caytu_nav_solution`).

Ce que couvrent les tests :

| Paquet | Vérifications |
|---|---|
| `caytu_config` | Validation du YAML, profil simulation, cohérence tags/portées des scans. |
| `caytu_perception` | Détection AprilTag sur images synthétiques, estimation de distance et de cap, conversion profondeur → LaserScan. |
| `caytu_course` | Lecture de l'ordre, séquence, stratégie (sauts), alignement, recherche, mémoire des tags, messages de l'arbitre, mot de départ. |
| `caytu_nav_solution` | Chronomètre, génération du YAML Nav2 (robot et simulation), client Nav2. |

## 2. Course en simulation

Prérequis : le dépôt de simulation PARC (`parc_robot_bringup`,
`parc_robot_description`, `ros_gz_*`) installé dans le workspace, et un
paquet `parc_msgs` pour l'arbitre. Il n'existe que sur le robot : pour la
simulation, en compiler une copie locale avec les champs connus
(`StartSignal` : `uint8 method`, `string heard` ; `TagVisit` : `int32 tag_id`,
`sensor_msgs/CompressedImage image` ; `Finish` : `uint8 reason`) et la
désigner par `FOSA_BASE_WS`.

```bash
colcon build --packages-up-to caytu_course
FOSA_PROFIL=simulation FOSA_BASE_WS=<copie de parc_msgs> ./run.sh course
```

Dans un autre terminal, une fois le nœud de course lancé :

```bash
./run.sh ordre-test "7,1,9,4"   # ESSAIS SEULEMENT
./run.sh depart
./run.sh etat
```

Le profil simulation bascule `use_sim_time`, `base_footprint`, `/odom`,
`/robot_base_controller/cmd_vel_unstamped` et active le filtre du lidar.
Le monde officiel ne contient pas de tags : il faut y ajouter les modèles des
tags 1, 4, 7 et 9 (36h11, 16,5 cm, centre à 1 m du sol) avant l'essai.

Points à contrôler dans le journal :

- `ATTENTE_ORDRE` puis `DEMARRAGE` : le robot n'a pas bougé avant le départ.
- `Plan : [...]` : plan de la stratégie et durée estimée.
- `RECHERCHE`, `APPROCHE`, `ALIGNEMENT` : progression tag par tag.
- `ARBITRE visite du tag N (image : image)`, puis `ARBITRE fin de course`.
- Aucun contact, aucune visite hors ordre, aucun tag revisité après un saut.

Le modèle Gazebo du robot sur-estime d'environ 15 % les rotations dans
l'odométrie (`wheel_separation` du plugin DiffDrive à 0,3409 m pour un
écartement réel de 0,393 m). Corriger cette valeur dans une copie de la
description donne des essais plus proches du robot réel.

## 3. Créneau sur le robot

### Avant le créneau

```bash
./deployer_robot.sh          # sur le robot, une fois le dépôt copié
```

### Premier créneau : interfaces

```bash
./run.sh diagnostic-robot
```

Le script vérifie ROS, Nav2, slam_toolbox, OpenCV, Vosk, le modèle français,
les topics de la configuration, les TF et les caméras. Noter :

- le topic et le type de l'ordre des tags ;
- le type de `/cmd_vel` (`Twist` ou `TwistStamped`) ;
- le nom exact des caméras, de l'IMU et le contenu de `/speech_audio` ;
- la présence de `base_link` et la hauteur du capteur.

Corriger `caytu_config/config/fosa.yaml` si un nom diffère, puis relancer le
diagnostic.

### Arbitre

```bash
source ~/ros2_ws/install/setup.bash          # parc_msgs (run.sh le fait seul)
ros2 topic echo /parc/run_status             # second terminal, garder ouvert
```

1. Demander à un organisateur d'armer une course d'essai : l'ordre est publié.
2. `./run.sh course`, puis `./run.sh depart` (ou « allez »).
3. Vérifier sur `http://ROBOT_IP:8080/` : départ, visites avec image, fin.

Ne publier sur `/parc/*` que pendant notre créneau, jamais sur l'odométrie.
Le rapport du diagnostic contient les définitions de `parc_msgs` : vérifier
la valeur de `reason` et le champ image de `TagVisit`.

### Cartographie et réglages

```bash
./run.sh cartographie
```

SLAM, Nav2 et perception tournent sans la mission. Déplacer le robot à la
vitesse prévue, vérifier la carte dans RViz, puis sauvegarder :

```bash
./run.sh sauver-carte essai1
```

La carte est écrite dans `~/.ros/fosa/cartes/`.

### Course complète

```bash
./run.sh course
./run.sh depart          # ou dire « allez »
./run.sh etat            # dans un autre terminal
```

Ne plus toucher au robot après le départ. `./run.sh arreter` est réservé aux
essais.

### Odométrie en rotation

Robot dégagé, course arrêtée :

```bash
./run.sh mesure-rotation
```

Deux tours lents comparent l'odométrie au gyromètre et au lidar. Une erreur
supérieure à quelques pour cent fausse le cap de SLAM pendant les tours de
recherche : corriger l'écartement des roues ou les coefficients du plugin.

## 4. Pannes fréquentes

| Symptôme | Piste |
|---|---|
| Aucun tag détecté | Caméra ou `camera_info` erroné, taille du tag, distance trop grande, TF absente. |
| Le robot ne bouge pas après le départ | Nav2 non prêt, carte SLAM absente, `/cmd_vel` mal remappé. |
| Visite refusée | Distance ou cap hors tolérance, tag non stable, image de preuve absente. |
| « No image » sur la page de l'arbitre | Champ image de `TagVisit` différent : lire le diagnostic, adapter `caytu_course/arbitre.py`. |
| Rien sur la page de l'arbitre | `parc_msgs` non sourcé (avertissement de `run.sh course`), `FOSA_BASE_WS` faux. |
| Tags sautés trop tôt ou trop tard | Recaler `course.strategy` (vitesse, temps de visite et de recherche) sur le journal `Plan : ...`. |
| Le robot visite dans le désordre | Ordre mal lu : vérifier le topic, le type et la QoS (`transient_local`). |
| Dérive pendant les rotations | Odométrie en rotation ; lancer `mesure-rotation`. |
| Départ vocal absent | Vosk ou modèle non installé, format de `/speech_audio` différent, seuil de confiance trop haut. |

Les journaux de course sont dans `~/.ros/fosa/course_*.log`.
