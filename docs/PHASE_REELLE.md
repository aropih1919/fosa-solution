# Phase réelle PARC 2026 — installer, lancer, démarrer

Ce guide sert aux essais (mardi → jeudi) et au gel du code. La course est
décrite dans `docs/parc-phase-finale-docs/PARC2026_Phase_Reelle_Guide_FR.md`.

## 1. Ce que fait la solution

1. Démarre slam_toolbox (carte construite en direct), Nav2, un détecteur
   AprilTag 36h11 par caméra RealSense, l'écoute du mot « allez » et le nœud
   de course (`caytu_course`).
2. Attend l'ordre des tags sur `course.order_topic`, puis le départ (commande
   ou voix). **Le robot ne bouge pas avant.**
3. Pour chaque tag, dans l'ordre : tour sur place, puis points de recherche de
   la carte ; dès que le tag est vu, approche Nav2 face au tag, alignement
   visuel à 0,5 m (±5°), signalement (ID + image), maintien. Un tag non trouvé
   dans son budget de temps est sauté et jamais revisité.
4. Signale la fin et s'arrête avant 10 minutes.

Toutes les valeurs (topics, repères, vitesses, tolérances) sont dans
`caytu_config/config/fosa.yaml`. Valeurs par défaut = robot réel ; Gazebo =
`FOSA_PROFIL=simulation`.

## 2. Avant d'aller au robot (sur le PC)

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-up-to caytu_course
colcon test --packages-select caytu_config caytu_course caytu_nav_solution caytu_perception
cd src/fosa-solution && ./deployer_robot.sh --preparer   # roues Vosk aarch64 + modèle FR
```

Copier le dépôt **avec `livraison/`** dans le dossier de l'équipe sur le robot :
`<dossier équipe>/ws/src/fosa-solution` (rsync ou scp).

## 3. Installation sur le robot (sans sudo)

```bash
cd <dossier équipe>/ws/src/fosa-solution
export FOSA_BASE_WS=<espace de travail de base du robot>   # à demander
./deployer_robot.sh
```

Le script s'arrête si Nav2 ou slam_toolbox manquent : **demander aux
organisateurs** `ros-jazzy-navigation2` et `ros-jazzy-slam-toolbox`. Le robot
redémarre entre deux créneaux : les fichiers restent, les processus non.

## 4. Lancer et démarrer

| Action | Commande |
|---|---|
| Solution complète | `./run.sh course` (terminal 1, laisser tourner) |
| Départ par commande | `./run.sh depart` (terminal 2) |
| Départ par la voix | dire distinctement « allez » (seul) près du robot |
| Suivre l'état | `./run.sh etat` |
| Cartographier sans mission | `./run.sh cartographie` |
| Sauver la carte | `./run.sh sauver-carte arene` |
| Ordre de test (essais seulement) | `./run.sh ordre-test "7,1,9,4"` |

Ordre exact pour une course : lancer `./run.sh course`, attendre
`[ATTENTE_DEPART]` (ordre reçu), puis départ. Après le départ, ne plus rien
toucher.

## 5. Créneau 1 (15 min) : vérifier et enregistrer

1. (2 min) `./run.sh diagnostic-robot` : topics, type de `/cmd_vel`, résolution
   et hauteur des caméras, débit audio, présence de `base_footprint`.
2. (3 min) Comparer avec `fosa.yaml` : `cmd_vel_topic`, `odom_topic`,
   `scan_topic`, caméras, `start_word.sample_rate`/`channels`. Corriger.
3. (5 min) `./run.sh diagnostic-robot --enregistrer 240` pendant que le robot
   est déplacé dans l'arène (`./run.sh cartographie` dans un autre terminal).
4. (3 min) Mesurer au mètre : empreinte (`navigation.footprint`), position du
   centre (`center_offset_x`), hauteur des tags.
5. Rapatrier `~/.ros/fosa/` (rapport, rosbag, `fosa_tags_course.yaml`).

## 6. Créneau 2 (15 min) : course complète

`./run.sh course`, `./run.sh ordre-test "<ordre>"`, `./run.sh depart` ; noter
pour chaque visite la distance et l'angle réels (mètre ruban, rapporteur).
Tester « allez » dans le bruit. Ajuster les tolérances et vitesses.

## 7. Réglages selon les mesures

| Constat | Clé de `fosa.yaml` |
|---|---|
| Le robot s'arrête trop tôt/tard devant le tag | `perception.apriltag.size_m`, `navigation.center_offset_x` |
| Base qui ne bouge pas pendant l'alignement | `course.alignment.min_linear_mps`, `min_angular_rps` |
| Plateaux de table non vus par le lidar | `navigation.obstacle_sources[camera*].enabled: true` (si les scans caméra sont propres) |
| Faux obstacles près du robot | `obstacle_min_range` de la source |
| Mot de départ refusé / déclenché par le bruit | `start_word.min_confidence` |
| Cartes autorisées | `course.slam_map_file` (carte des essais) |
| Poses de tags mémorisées autorisées | `course.memorized_tags_file` (fichier `~/.ros/fosa_tags_course.yaml` d'un essai) |

## 8. Questions aux organisateurs

- Nom, type et QoS du topic d'ordre ; publié avant le départ, en latched ?
- Paquet de statut (départ, visite + image, fin) : à brancher dans `caytu_course/statut.py`.
- Cartes pré-construites et poses de tags mémorisées autorisées vendredi ?
- Installation de Nav2 et slam_toolbox ; chemin de l'espace de travail de base.
- `/cmd_vel` est-il l'entrée autorisée ? Que fait `obstacle_avoidance` (distance d'arrêt) ? `person_follower` désactivé pendant les courses ?
- Format de `/speech_audio` (fréquence, canaux) ; quelle caméra est la haute ?
- Tolérance des juges sur 0,5 m et durée de maintien attendue.
- Position de départ identique à chaque course ?

## 9. Avant la course notée

- [ ] `./run.sh diagnostic-robot` sans échec ; `ROS_DOMAIN_ID=0`, temps réel.
- [ ] Modèle Vosk présent ; rien à télécharger.
- [ ] `fosa.yaml` gelé (empreinte, vitesses, tolérances mesurées).
- [ ] Topics d'ordre et de statut conformes aux organisateurs.
- [ ] Une course complète de bout en bout réussie pendant les essais.
- [ ] `./run.sh course` lancé, état `[ATTENTE_DEPART]`, robot immobile.

## 10. Simulation

```bash
FOSA_PROFIL=simulation ./run.sh gazebo        # terminal 1
FOSA_PROFIL=simulation ./run.sh course        # terminal 2
FOSA_PROFIL=simulation ./run.sh ordre-test "7,1,9,4" && FOSA_PROFIL=simulation ./run.sh depart
```

Le monde officiel ne contient pas de tags : il faut en ajouter pour une
course complète (modèles OBJ texturés, un nom de matériau par tag).
