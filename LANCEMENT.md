# Lancer la solution Fosa : sur notre PC et sur le robot réel

Ce guide suit le déroulé de la phase finale, de l'installation à la course de
vendredi. Les détails sont dans `deploy/README.md` (robot),
`docs/COURSE.md` (course) et `simulation/README.md` (bancs d'essai).

## 0. Le déroulé, en une page

1. **Carte.** Le robot est conduit au clavier dans l'arène ; slam_toolbox fait
   la carte, la caméra du haut note au passage la position des AprilTags
   (`<carte>/tags.yaml`).
2. **Repérage des tags** (si besoin). Sur la carte, on repasse devant chaque tag
   pour préciser sa position.
3. **Course.** Le robot part de la position de départ marquée au sol :
   - l'arbitre officiel publie l'ordre des tags sur `/parc/tag_order` quand le
     juge arme la course ;
   - au départ (commande, ou mot **« allez »**, « go », « vai » : bonus vocal), la
     solution prévient l'arbitre sur `/parc/start` ;
   - pour chaque tag, dans l'ordre : le robot s'arrête à 0,5 m, face au tag,
     sans le toucher, publie `/parc/tag_visit` (numéro du tag et image JPEG de
     la caméra du haut), puis reste immobile 3,5 s ;
   - tout objet devant le robot, personnes comprises, l'arrête ou le fait
     contourner (Nav2 et le garde-fou `nav2_collision_monitor`) ;
   - l'arbitre clôt la course quand le dernier tag est signalé dans l'ordre ;
     la solution s'arrête aussi si `/parc/run_status` passe à `FINISHED`,
     `TIMED_OUT` ou `TERMINATED`. `/parc/finish` n'est envoyé que pour finir en
     avance.

### Topics de l'arbitre (paquet `parc_msgs`)

| Topic | Sens | Type | Ce que fait la solution |
|---|---|---|---|
| `/parc/tag_order` | arbitre → robot | `TagOrder` (verrouillé) | lit `tag_ids` ; liste vide tant que rien n'est armé : ignorée |
| `/parc/run_status` | arbitre → robot | `RunStatus` (verrouillé, 2 Hz) | journal des états, faux départ signalé, arrêt aux états finaux |
| `/parc/start` | robot → arbitre | `StartSignal` | une fois : `method 0` (commande) ou `method 1` + `heard` (voix) |
| `/parc/tag_visit` | robot → arbitre | `TagVisit` | `tag_id` + `CompressedImage` JPEG, robot arrêté |
| `/parc/finish` | robot → arbitre | `Finish` | seulement pour terminer avant le dernier tag |

Les noms de ces topics sont dans `caytu_config/config/fosa.yaml`
(`course.order_topic`, `course.referee`). Sans `parc_msgs`, la course tourne
quand même mais l'arbitre n'est pas prévenu (avertissement au lancement).

### Règles des organisateurs

- Ne publier sur `/parc/*` **que pendant notre créneau**, et **jamais** sur
  l'odométrie du robot (`/botwheel_explorer/odom`). La solution ne publie pas
  sur l'odométrie.
- Le robot ne doit pas bouger entre l'armement et le départ (faux départ,
  contrôlé sur l'odométrie), ni pendant les 3 s qui suivent chaque visite.
- Sur le robot, `parc_msgs` est dans l'espace de travail de base :
  `source ~/ros2_ws/install/setup.bash`, puis notre espace de travail par-dessus
  (`deploy/` le fait).
- Tableau de bord de l'arbitre : `http://<IP du robot>:8080/`.
- Le compte du robot est partagé : le code n'y reste que le temps du créneau,
  `deploy/clean.sh` efface tout à la fin.

---

## 1. Sur notre PC

### 1.1 Installation (une fois, Ubuntu 24.04)

```bash
cd ~/ros2_ws/src/fosa-solution
./install.sh --models
```

Il installe :
- ROS 2 Jazzy, Nav2, slam_toolbox et `nav2_collision_monitor` ;
- `apriltag_ros` (détecteur des tags de la solution : `ros-jazzy-apriltag-ros`,
  par rosdep), OpenCV (image JPEG pour l'arbitre ; aruco seulement pour le
  détecteur de secours), `depth_image_proc` et `audio_common_msgs` ;
- le dépôt officiel PARC : simulateur, arbitre `parc_referee` et messages
  `parc_msgs`. Un ancien clone est mis à jour par `git pull` ;
- faster-whisper et le modèle Whisper du mot de départ.

Puis il compile le workspace.

Contrôle :
```bash
cd ~/ros2_ws
colcon test --packages-select caytu_config caytu_nav_bringup caytu_nav_solution \
    caytu_semantic_map caytu_assistant caytu_perception && colcon test-result
ros2 pkg prefix parc_referee        # arbitre officiel compilé
```

Dans chaque nouveau terminal (sans `run.sh`) :
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/.venv/bin/activate
export PYTHONPATH=~/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH
```

### 1.2 Essai complet sans Gazebo : banc 2D jugé par l'arbitre officiel

Le simulateur 2D publie les interfaces du vrai robot : `/cmd_vel`,
`/botwheel_explorer/odom`, `/scan`, caméras `camera1` en haut et `camera2` en
bas, temps réel. L'arbitre `parc_referee` juge la course :
```bash
simulation/banc.sh 2d carte          # une fois : carte du café 2D
simulation/banc.sh 2d reperage       # une fois : position des tags
simulation/banc.sh 2d arbitre --ordre 9,4,1,7                  # départ par commande
simulation/banc.sh 2d arbitre --ordre 7,1,9,4 --depart voix    # départ « vocal »
```

Le rapport donne deux verdicts :
- celui de notre vérificateur (vraie pose du robot, contacts, distance aux tags) ;
- celui de l'arbitre officiel, ligne par ligne :
  `VISITE tag 9 … in_order, image match …, pause ok`.

Les fichiers de l'arbitre (`run.json` et les images reçues) sont dans
`simulation/resultats/<date>_2d_arbitre/arbitre/parc_runs/`.

Mot de départ seul (Whisper sur des fichiers audio) : `simulation/banc.sh voix`.

### 1.3 Simulateur officiel PARC (Gazebo, depuis le 5 novembre)

Le simulateur officiel publie maintenant les mêmes topics que le vrai robot. Il
pose de vrais AprilTags (36h11, 1, 4, 7, 9, carré noir de 0,12 m) et des
obstacles du quotidien, dans une arène tirée d'une graine (`seed`). Profil de
navigation : `robot:=parc`. Réglages caytu_config : `sim_parc.yaml` (temps
simulé, caméras `camera1`/`camera2`, tags de 0,12 m). Détecteur des tags :
`apriltag_ros`, comme sur le vrai robot.

Installation (une fois, dans le même workspace que fosa-solution ; `install.sh`
le fait aussi) :
```bash
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/PARC2026-Engineers-League.git   # ou git pull s'il existe
cd .. && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

```bash
export FOSA_CONFIG_OVERRIDE=~/ros2_ws/src/fosa-solution/caytu_config/config/sim_parc.yaml
```
Cette ligne est à taper dans **chaque** terminal Fosa.

| Terminal | Commande |
|---|---|
| 1, monde | `ros2 launch parc_robot_bringup phase2_hard.launch.py seed:=7` |
| 2, arbitre | `ros2 launch parc_referee referee.launch.py odom_topic:=/botwheel_explorer/odom time_limit_s:=20000.0` |
| 3, carte | `ros2 launch caytu_nav_bringup mapping.launch.py robot:=parc map_name:=parc_seed7` |
| 4, clavier | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4` |

Terminal 1 qui affiche en boucle `Exception sending a multicast message: Network is unreachable` : Gazebo cherche ses pairs sur un réseau absent, et ces milliers de lignes ralentissent tout le PC. Taper `export GZ_IP=127.0.0.1` dans le terminal 1, puis relancer le monde.

Conduisez le robot devant les quatre tags, puis :
```bash
ros2 run caytu_nav_solution save_map        # -> ~/fosa_maps/parc_seed7/ (avec tags.yaml)
# Contrôle : tags notés comparés à la vérité du monde (essais seulement)
python3 simulation/outils/comparer_tags_parc.py ~/fosa_maps/parc_seed7 /tmp/parc_worlds/parc_random_seed7.json
```
Arbitre dans ce simulateur : son chronomètre compte en temps **réel**. À 6 %
du temps réel, ses 10 minutes ne font que 36 s simulées : d'où
`time_limit_s:=20000.0` pour les essais. La course ignore de toute façon un
`TIMED_OUT` en temps simulé (message dans le journal) ; sur le vrai robot, elle
s'arrête.

Départ : `ros2 run caytu_nav_solution depart` répète le signal jusqu'à
l'accusé de la course et affiche « Départ reçu par la course ». S'il affiche
« ÉCHEC : aucun accusé », la course n'est pas lancée ou pas dans le même
`ROS_DOMAIN_ID`.

Garde-fou dans ce simulateur : lidar seul (`collision_cameras: false` du
profil `parc`). Sur un PC sans carte graphique, le simulateur tourne bien
moins vite que le temps réel. Les nuages des caméras arrivent alors avec des
secondes de retard, et un garde-fou qui les attend arrête le robot
(« invalid source »). Nav2 utilise quand même les caméras pour contourner.
Pour essayer avec les caméras : `garde_fou_cameras:=true`.

Avant chaque relance du monde, vérifiez qu'aucun Gazebo ne survit
(`pgrep -af "gz sim"`, sinon `pkill -f "gz sim"`), puis relancez la course
**après** le monde : le temps simulé repart de zéro.

Conduite pour la carte : avancez en courbes douces devant chaque tag, à 1–2 m,
avec des arrêts de 2 s. **Ne tournez pas longtemps sur place.** Dans notre
essai, un tour complet sur place a fait dériver le cap du robot dans la carte
(+70° à 270° de rotation) : les tags vus en fin de tour étaient mal placés. Les
premières vues, robot presque immobile, étaient justes à 8 cm (tag 7, vu à
2,8 m) et 19 cm (tag 1, vu à 5 m). Vérifiez avec `comparer_tags_parc.py` après chaque carte.

Le micro du PC est publié comme le ReSpeaker (`/speech_audio`) : dites
« allez » pour le départ vocal. Sans micro : `audio:=false` au terminal 1, puis
départ par la commande. Les topics `/sim/...` servent aux essais seulement : la
solution ne les lit pas.
Arrêtez la carte (`Ctrl-C` dans le terminal 3) et relancez le monde (terminal 1) pour
remettre le robot au départ. Ensuite :
```bash
# terminal 3 : course
ros2 launch caytu_nav_bringup course.launch.py robot:=parc carte:=$HOME/fosa_maps/parc_seed7
# navigateur : http://localhost:8080/judge?token=parc2026 -> Arm (ou, en ligne de commande :)
python3 simulation/outils/arbitre_officiel.py armer --ordre 9,4,1,7
# terminal 4, après « Ordre des tags reçu » et « Prêt. Départ » :
ros2 run caytu_nav_solution depart          # ou dire « allez » au micro du PC
python3 simulation/outils/arbitre_officiel.py attendre      # bilan de l'arbitre
```

---

## 2. Sur le robot réel

Les scripts `deploy/*.sh` se lancent sur le **portable**. Les commandes
`ros2 …` se tapent dans le **terminal du robot**, ouvert par
`deploy/connect.sh` (tmux : `Ctrl-b c` nouveau terminal, `Ctrl-b n` suivant ;
une coupure Wi-Fi n'arrête rien, relancer `connect.sh` rattache).

### 2.1 Une fois, sur le portable

```bash
cp deploy/robot.env.example deploy/robot.env
nano deploy/robot.env
```

| Réglage | Valeur |
|---|---|
| `ROBOT` | `<compte>@<IP du robot>` (donnés par les organisateurs) |
| `ROBOT_BASE_SETUP` | `'~/ros2_ws/install/setup.bash'` (garder les apostrophes) |
| `ROBOT_DIR` | `/dev/shm/fosa` (mémoire vive), ou `'~/fosa'` si les organisateurs veulent un dossier d'équipe sur disque |

Si le robot n'a pas Internet, préparez sur le portable, **avec Internet** :
```bash
deploy/telecharger_pour_robot.sh      # interroge le robot ; sinon --sans-robot
```
Ce script télécharge faster-whisper et ses dépendances (roues aarch64, au
Python du robot, numpy du robot inchangé), ainsi que le modèle Whisper. Le
tout va dans `robot_data/paquets/` (environ 250 Mo).

### 2.2 Début de chaque créneau

```bash
deploy/deploy.sh --installer --paquets              # 1er créneau : pas encore de carte
deploy/deploy.sh --installer --paquets --map arene_dakar
deploy/connect.sh
```

Ce que fait `deploy.sh` :
- il envoie les sept paquets Fosa dans le dossier d'équipe, sans tests, outils
  ni modèles d'IA ;
- avec `--paquets`, il envoie aussi faster-whisper et le modèle ;
- avec `--installer`, il lance `installer_robot.sh` sur le robot. Ce script
  vérifie tout (ROS, Nav2, `apriltag_ros`, OpenCV, `parc_msgs`, voix) et installe ce qui
  manque après confirmation. Les paquets apt passent par sudo ou par les
  organisateurs : la commande est affichée ;
- il compile, puis liste ce qui manque encore (« INDISPENSABLE absent »).

Si « parc_msgs » manque, vérifiez d'abord `ROBOT_BASE_SETUP`. Le dernier recours
est `deploy/deploy.sh --parc-msgs` : il compile le `parc_msgs` du dépôt PARC du
portable, et seulement si le robot n'en a pas.

Premier créneau, deux vérifications sur le robot :
```bash
ros2 run tf2_ros tf2_echo base_link camera1_link   # camera1 doit être la haute (z ≈ 1 m)
ros2 topic echo --once /parc/run_status            # l'arbitre des organisateurs répond
```
La taille des tags (`perception.apriltag.size_m` dans
`caytu_config/config/robot_reel.yaml`) est à **mesurer** : 0,165 m si 16,5 cm
est le carré noir, 0,132 m si 16,5 cm compte la bordure blanche.

### 2.3 Carte (premier créneau)

Marquez au sol la position et l'orientation du robot : la course partira de là.
```bash
# terminal 1
ros2 launch caytu_nav_bringup mapping.launch.py robot:=real map_name:=arene_dakar
# terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
```
Faites le tour de l'arène. Passez à 1–2 m de chaque tag, face à lui, et arrêtez-vous 2 s.
```bash
ros2 run caytu_nav_solution save_map
cat $FOSA_MAPS_DIR/arene_dakar/tags.yaml           # tags 1, 4, 7 et 9 présents ?
```
`Ctrl-C` dans le terminal 1, puis **tout de suite, sur le portable** :
```bash
deploy/fetch.sh                                    # -> robot_data/maps/arene_dakar/
```

### 2.4 Repérage des tags (si une position est fausse ou manque)

Le robot part de la position de départ marquée.
```bash
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
```
Conduisez au clavier devant chaque tag (terminal 2). Ensuite, `Ctrl-C`, puis
`deploy/fetch.sh` sur le portable.

### 2.5 Répétition de la course (créneaux d'essai)

Pendant notre créneau, avec l'arbitre des organisateurs :
1. le juge arme la course sur le tableau de bord ;
2. on lance :
   ```bash
   ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
   ```
3. on attend « Ordre des tags reçu : [...] » puis « Prêt. Départ » ;
4. on donne le départ : `ros2 run caytu_nav_solution depart` (terminal 2), ou on dit **« allez »**.

Sans arbitre armé, `ordre:=7,1,9,4` donne l'ordre à la main. L'arbitre ignore
alors notre départ (« non ARMED » dans le journal).

À lire dans le journal de la course :

| Ligne | Sens |
|---|---|
| `Arbitre : ARMED` puis `RUNNING` | l'arbitre a reçu notre départ |
| `Arbitre prévenu de la visite du tag 9 (image JPEG 39 ko)` | visite envoyée avec image |
| `Arbitre : FINISHED` | course close par l'arbitre (dernier tag dans l'ordre) |
| `Arbitre : FAUX DÉPART` | le robot a bougé avant le départ |
| `parc_msgs introuvable` | l'arbitre ne reçoit rien : sourcer `~/ros2_ws`, ou `--parc-msgs` |

Le tableau de bord (`http://<IP du robot>:8080/`) montre, pour chaque visite :
- « in order » ;
- « image match » (l'arbitre lit le bon tag sur notre image) ;
- « hold ok ».

### 2.6 Vendredi : la course notée

Avant le créneau :
- le code est gelé (voir 3) ;
- la carte `arene_dakar` est dans `robot_data/maps/` du portable.

```bash
deploy/deploy.sh --installer --paquets --map arene_dakar
deploy/connect.sh
ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
```
Robot à la position de départ, immobile. Le juge arme la course. Après
« Ordre des tags reçu » et « Prêt. Départ », dites **« allez »** (bonus vocal)
ou tapez `ros2 run caytu_nav_solution depart`. Ensuite, ne touchez plus à rien.

- Sans faster-whisper : ajoutez `voix:=false` ; le départ se donne par la commande.
- Si les cartes préparées sont interdites : `carte:=''` ; la carte se fait pendant la course.

### 2.7 Fin de chaque créneau

```bash
deploy/clean.sh        # rapatrie les cartes, arrête nos programmes, efface le dossier d'équipe
```

---

## 3. Gel du code (avant vendredi)

```bash
cd ~/ros2_ws/src/fosa-solution
simulation/banc.sh 2d arbitre        # dernier contrôle (tests unitaires : 1.1)
git add -A && git commit -m "Gel du code pour la course" && git tag gel-course
```

Ensuite :
- plus aucune modification ;
- vendredi, `deploy/deploy.sh` envoie exactement cette version.

---

## 4. En cas de problème

| Situation | Que faire |
|---|---|
| Arrêter le robot | `Ctrl-C` sur le lancement ; sinon `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"` |
| SSH coupé | `deploy/connect.sh` (tout tourne encore dans tmux) |
| « INDISPENSABLE absent » | `deploy/deploy.sh --installer` ; sans sudo, donner la commande apt affichée aux organisateurs |
| Pas d'Internet sur le robot | `deploy/telecharger_pour_robot.sh` sur le portable, puis `deploy/deploy.sh --paquets --installer` |
| Lancement arrêté : « apriltag_ros absent » | `sudo apt install ros-jazzy-apriltag-ros` (robot : `installer_robot.sh` ou organisateurs). Secours seulement : `detecteur:=opencv` |
| Robot : `Failed to find a free participant index for domain 0` | CycloneDDS des organisateurs : 10 processus ROS au plus, déjà pris. Relancer `deploy/deploy.sh` (son `env.sh` relève la limite pour nos processus), ou dans le terminal du robot : `export CYCLONEDDS_URI="${CYCLONEDDS_URI:+$CYCLONEDDS_URI,}<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"` puis `ros2 daemon stop` |
| Simulateur : `Exception sending a multicast message: Network is unreachable` en boucle | `export GZ_IP=127.0.0.1` avant `ros2 launch parc_robot_bringup ...` (sinon le PC est ralenti par l'affichage) |
| Recovery, ou robot immobile pendant la course | 2e terminal, après le lancement et avant `depart` : `ros2 run caytu_nav_solution diagnostic` (ou le fichier seul : `python3 diagnostic_course.py`, option `--bag`). Écoute seule. Rapport toutes les 5 s : à quel étage la commande s'arrête (Nav2 → garde-fou → organisateurs → base), points gênants du lidar, coût sous le robot, recoveries de Nav2. `Ctrl-C` → BILAN et PISTES ; fichiers dans `$FOSA_ROOT/diag` |
| `depart` affiche « ÉCHEC : aucun accusé » | course_tags pas lancé, planté, ou autre `ROS_DOMAIN_ID` |
| Journal : `Garde-fou : ARRÊT, une source n'a pas de données fraîches` | le garde-fou bloque le robot : `ros2 topic hz /scan_clean` (et les caméras si `garde_fou_cameras:=true`) |
| Journal : `Garde-fou : ARRÊT (obstacle dans ZoneArret)` en permanence | un point du lidar dans l'emprise du robot : version d'avant le filtre d'emprise, ou robot collé à un obstacle |
| Le robot ne bouge pas ; journal Nav2 : `Start occupied`, `Goal failed` | La carte contient le robot lui-même : le lidar voit ses roues (0,17–0,28 m). Les versions d'avant le 5 novembre au soir les gardaient. La version actuelle les filtre (boîte de l'emprise du robot). Carte déjà faite, à sauver sans la refaire : `ros2 run caytu_nav_solution edit_map <carte> --erase -0.40 -0.35 0.35 0.35` (efface l'emprise autour de la pose de départ ; la course lit ensuite `nav_map.yaml`) |
| Le robot ne bouge pas après le départ ; journal : `invalid source`, `Ignoring the source`, `differ on 18 seconds` | Le garde-fou n'a pas de données fraîches d'une caméra. Vérifier `ros2 topic hz /camera1/camera1/depth/image_rect_raw` et `/top_camera_scan`. Simulateur lent : `garde_fou_cameras:=false` (défaut du profil `parc`). Vrai robot : caméra muette, la relancer ; en dernier recours `garde_fou_cameras:=false` (lidar seul) |
| « Ordre jamais reçu » | le juge n'a pas armé ; `ros2 topic echo --once /parc/tag_order` |
| « non ARMED : il ignorera ce départ » | course pas armée par le juge (ou déjà finie) |
| Image « not detected » au tableau de bord | robot pas assez en face ou tag hors champ : vérifier `camera1` en haut et `size_m` |
| Carte introuvable avec `--map` | `deploy/fetch.sh` (carte encore sur le robot) |
