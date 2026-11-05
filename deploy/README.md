# Robot réel : flux de commandes (mode B)

La solution est envoyée dans notre dossier d'équipe sur le robot
(`ROBOT_DIR`, `/dev/shm/fosa` en mémoire vive par défaut) le temps d'un
créneau, compilée par-dessus l'espace de travail de base des organisateurs
(`~/ros2_ws`, qui contient `parc_msgs`), puis effacée par `deploy/clean.sh`.
**On redéploie à chaque créneau.** Vue d'ensemble PC + robot :
[`../LANCEMENT.md`](../LANCEMENT.md).

## Où taper quoi

| Où | Quoi |
|---|---|
| **Portable**, dossier `fosa-solution` | uniquement les scripts `deploy/*.sh` |
| **Terminal du robot**, ouvert par `deploy/connect.sh` | les commandes `ros2 launch` / `ros2 run` |

Dans le terminal du robot (tmux) :

| Touches | Effet |
|---|---|
| `Ctrl-b c` | nouveau terminal |
| `Ctrl-b n` | terminal suivant |
| `Ctrl-b d` | se détacher |

Relancer `deploy/connect.sh` rattache au même tmux, même après une coupure
Wi-Fi.

## 0. Une seule fois, sur le portable

```bash
cp deploy/robot.env.example deploy/robot.env
nano deploy/robot.env     # ROBOT=<compte>@<adresse du robot> ; ROBOT_BASE_SETUP='~/ros2_ws/install/setup.bash'
```

Si le robot n'a pas Internet, sur le portable **avec Internet** :
```bash
deploy/telecharger_pour_robot.sh   # faster-whisper (roues aarch64) + modèle Whisper -> robot_data/paquets/
```

## 1. Premier créneau : créer la carte `arene_dakar`

Il n'y a pas encore de carte : **pas de `--map`**.

**Portable :**
```bash
deploy/deploy.sh --installer --paquets   # envoi, installation de ce qui manque, compilation
deploy/connect.sh
```
`--paquets` seulement si `deploy/telecharger_pour_robot.sh` a été lancé. Lire
les lignes « INDISPENSABLE / Optionnel absent » à la fin (section 4).

**Robot, terminal 1.** Avant de lancer, marquez au sol la position et
l'orientation du robot : la course devra partir de là.
```bash
ros2 launch caytu_nav_bringup mapping.launch.py robot:=real map_name:=arene_dakar
```

**Robot, terminal 2** (`Ctrl-b c`) :
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
```
Faites le tour de l'arène. Passez à 1–2 m de chaque tag, face à lui, et
arrêtez-vous 2 secondes devant.

**Robot, terminal 2 :**
```bash
ros2 run caytu_nav_solution save_map
cat $FOSA_MAPS_DIR/arene_dakar/tags.yaml      # les tags 1, 4, 7, 9 doivent y être
```
Puis arrêtez la cartographie avec `Ctrl-C` dans le terminal 1.

**Portable, tout de suite** : la carte n'existe que dans la mémoire du
robot, elle est perdue s'il redémarre.
```bash
deploy/fetch.sh           # -> robot_data/maps/arene_dakar/
```

**S'il reste du temps**, répétez la course (section 2, étape « Répétition »).

**Fin de créneau, portable :**
```bash
deploy/clean.sh           # rapatrie les cartes, arrête tout, efface /dev/shm/fosa
```

## 2. Créneaux suivants : la carte existe

**Portable :**
```bash
deploy/deploy.sh --installer --paquets --map arene_dakar
deploy/connect.sh
```

**Robot, si la carte doit être plus précise : repérage des tags.** Le robot
part de la position de départ marquée.
```bash
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4   # terminal 2
```
Passez devant chaque tag. Ensuite :
- `Ctrl-C` dans le terminal 1 ;
- `deploy/fetch.sh` sur le portable.

**Robot : répétition de la course.** Le robot part de la position de départ.
Avec l'arbitre des organisateurs (le juge arme la course, l'ordre arrive sur
`/parc/tag_order`) :
```bash
ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
```
Sans arbitre armé, ajoutez `ordre:=7,1,9,4` (l'arbitre ignore alors notre
départ, le journal le dit). Dans le terminal 2, une fois « Prêt. Départ sur
/fosa/depart » affiché :
```bash
ros2 run caytu_nav_solution depart
```
Tableau de bord de l'arbitre : `http://<IP du robot>:8080/` (visites « in
order », « image match », « hold ok »).

**Fin de créneau, portable :**
```bash
deploy/clean.sh
```

## 3. Vendredi : la course

**Portable :**
```bash
deploy/deploy.sh --installer --paquets --map arene_dakar
deploy/connect.sh
```

**Robot.** Le robot est à la position de départ marquée.
```bash
ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
```
Le juge arme la course. Attendez deux messages :
- « Ordre des tags reçu : [...] » (lu sur `/parc/tag_order`) ;
- « Prêt. Départ sur /fosa/depart ».

Pour le départ, deux possibilités :
- dire **« allez »** au robot (bonus +5) ;
- ou, dans le terminal 2 : `ros2 run caytu_nav_solution depart`.

Ensuite, ne plus rien toucher. La solution prévient l'arbitre
(`/parc/start`, puis `/parc/tag_visit` avec une image JPEG à chaque tag) et
s'arrête quand `/parc/run_status` passe à `FINISHED`, `TIMED_OUT` ou
`TERMINATED`. Ne publier sur `/parc/*` que pendant notre créneau ; rien ne
publie sur l'odométrie.

Si faster-whisper n'est pas installé (section 4, à refaire à chaque créneau), ajoutez `voix:=false` à la
commande de lancement. Le départ se donne alors par la commande `depart`.

Si les cartes préparées sont interdites, lancez la même commande avec
`carte:=` vide : la carte se fait pendant la course.

**Après la course, portable :**
```bash
deploy/clean.sh
```

## 4. Paquets manquants : `installer_robot.sh`

`deploy.sh` dépose ce script dans `$FOSA_ROOT` (même si rsync manque au
robot) et le lance avec `--installer`. **À la main, dans le terminal du robot :**
```bash
bash $FOSA_ROOT/installer_robot.sh --verifier   # liste seulement ce qui manque
bash $FOSA_ROOT/installer_robot.sh              # installe, après confirmation
```

Options :

| Option | Effet |
|---|---|
| `--oui` | ne pose pas de question |
| `--sans-voix` | ignore faster-whisper |
| `--apriltag-ros` | exige `apriltag_ros` même si caytu_config choisit `opencv` |

Il vérifie :
- les paquets ROS de la solution (Nav2, slam_toolbox, collision_monitor, filtres, cv_bridge…) ;
- colcon, rsync et tmux ;
- le détecteur des tags `apriltag_ros` et `apriltag_msgs` (indispensables :
  c'est le détecteur choisi dans caytu_config) ;
- les modules Python yaml, numpy et OpenCV (aruco seulement pour le détecteur
  de secours `opencv`) ;
- `parc_msgs`, les messages de l'arbitre (dans `~/ros2_ws` ; à défaut,
  `deploy/deploy.sh --parc-msgs` compile celui du dépôt PARC du portable) ;
- pour la voix : audio_common_msgs, faster-whisper et son modèle.

Il installe de deux façons :

| Quoi | Comment | Reste après le redémarrage ? |
|---|---|---|
| Paquets ROS, colcon, rsync, tmux | `apt`, il faut `sudo` | **Oui** (paquets publics, pas notre code) : accord des organisateurs d'abord |
| faster-whisper, son modèle, OpenCV contrib en dernier recours | `pip`, dans `$FOSA_ROOT/py` et `$FOSA_ROOT/hf` (dossier d'équipe) | Non : effacé par `clean.sh`, donc à refaire à chaque créneau |

Il faut Internet sur le robot, sauf si `deploy/telecharger_pour_robot.sh` a
tout préparé sur le portable et que `deploy.sh --paquets` l'a envoyé : pip
prend alors les roues dans `$FOSA_ROOT/paquets/roues`, et le modèle est déjà
dans `$FOSA_ROOT/hf`. Sans `sudo`, le script affiche la commande
`apt-get` à donner aux organisateurs. Les modules Python que le robot a déjà
(numpy, yaml…) ne sont jamais remplacés : leurs copies sont retirées de
`$FOSA_ROOT/py`.

Code de sortie :

| Code | Sens |
|---|---|
| `0` | tout est en place |
| `1` | il manque un élément indispensable |
| `2` | seul un élément optionnel manque |

Essai du mot de départ, une fois faster-whisper installé :
```bash
ros2 run caytu_assistant mot_depart       # seul « allez » doit afficher « MOT DE DÉPART reconnu »
```

## 5. Carte faite depuis le portable (mode A)

`deploy.sh --map` cherche la carte dans `robot_data/maps/`. Copiez-la depuis
le portable :
```bash
mkdir -p robot_data/maps && cp -r ~/fosa_maps/arene_dakar robot_data/maps/
```

## 6. En cas de problème

| Situation | Commande |
|---|---|
| Arrêter le robot | `Ctrl-C` sur le lancement ; sinon, au robot : `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"` |
| SSH coupé | portable : `deploy/connect.sh` (les programmes tournent toujours dans tmux) |
| « INDISPENSABLE absent » après `deploy.sh` | `deploy/deploy.sh --installer` (sans sudo : commande à donner aux organisateurs) |
| « parc_msgs » absent | vérifier `ROBOT_BASE_SETUP` (`'~/ros2_ws/install/setup.bash'`) ; sinon `deploy/deploy.sh --parc-msgs` |
| Robot sans Internet | portable : `deploy/telecharger_pour_robot.sh`, puis `deploy/deploy.sh --paquets --installer` |
| `Failed to find a free participant index for domain 0` | CycloneDDS des organisateurs limité à 10 processus ROS (`MaxAutoParticipantIndex`). `env.sh` (deploy.sh du 5 novembre au soir) relève la limite pour nos processus ; dans un terminal ouvert avant : voir LANCEMENT.md, section 4 |
| « Arbitre … non ARMED » au départ | le juge n'a pas armé la course (ou elle est finie) |
| « Optionnel absent » | ne concerne que `voix:=true` ou le secours `detecteur:=opencv` ; même script |
| « Envoi impossible (rsync absent du robot ?) » ou « colcon absent » | `deploy/connect.sh`, puis `bash $FOSA_ROOT/installer_robot.sh`, puis relancer `deploy/deploy.sh` |
| `--map` : « Carte … introuvable » | la carte n'est pas encore sur le portable : `deploy/fetch.sh` ou section 5 |
| Récupérer les cartes en cours de créneau | portable : `deploy/fetch.sh` |
