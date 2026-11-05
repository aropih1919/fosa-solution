# Tests : localisation des AprilTags (simulée) et mot de départ

Ce guide teste deux parties de la course sans le robot :
- **A. Les AprilTags** : détection, position de chaque tag sur la carte, puis
  visites à 0,5 m. En simulation (simulateur 2D ou Gazebo PARC), comparée à la
  vraie position des tags.
- **B. La reconnaissance vocale** du mot de départ (« allez », « go », « vai ») :
  au micro du PC, avec des fichiers audio comme le ReSpeaker, puis pour lancer
  une course.

Chaque test donne la commande, ce qu'on doit voir et le critère de réussite. Une
fiche à remplir est à la fin.

## 0. Préparation (une fois par PC)

```bash
cd ~/ros2_ws/src/fosa-solution
./install.sh --models          # ROS, Nav2, AprilTags, faster-whisper + modèle Whisper « base »
cd ~/ros2_ws && colcon build --symlink-install && cd -
```

Si votre `.venv` date d'avant le 5 novembre (erreur `process() takes no keyword
arguments` au téléchargement du modèle) :
```bash
~/ros2_ws/.venv/bin/python -m pip install "brotli>=1.2"
~/ros2_ws/.venv/bin/python -c "from faster_whisper import WhisperModel; WhisperModel('base', device='cpu', compute_type='int8')"
```

**Dans chaque terminal de test :**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/.venv/bin/activate
export PYTHONPATH=~/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH   # faster-whisper pour ros2 run / launch
unset FOSA_CONFIG_OVERRIDE         # en simulation : jamais le réglage du vrai robot
cd ~/ros2_ws/src/fosa-solution
```

`./run.sh` fait tout cela lui-même ; les commandes `ros2 ...` ci-dessous en ont
besoin.

Contrôle rapide :
```bash
ros2 pkg prefix apriltag_ros        # le détecteur des tags de la solution
python3 -c "import cv2; print('OpenCV', cv2.__version__, 'aruco', hasattr(cv2, 'aruco'))"
python3 -c "import faster_whisper, brotli; print('faster-whisper OK, brotli', brotli.__version__)"
```

Attendu :
- OpenCV 4.6 ou plus, avec aruco ;
- brotli 1.2 ou plus.

---

# A. Localisation des AprilTags

Depuis le 5 novembre, le détecteur de la solution est `apriltag_ros`
(`perception.apriltag.detector`). Les essais A2 et suivants l'utilisent sans
réglage. A1 vérifie le détecteur de secours OpenCV (`detecteur:=opencv`).
Dans le simulateur officiel PARC, qui a ses propres tags, la comparaison à la
vérité du monde se fait avec `simulation/outils/comparer_tags_parc.py`
(voir `LANCEMENT.md`, 1.3).

## A1. Détection seule, sans simulateur (2 min)

Vérifie que le détecteur lit les tags 36h11 avec votre OpenCV. Ce test aurait
attrapé le plantage d'OpenCV 4.6, corrigé le 5 novembre.

```bash
ros2 run caytu_perception essai_tags --sortie /tmp/essai_tags_$(date +%H%M%S)
```

**Attendu :**
- les tags 1, 4, 7 et 9 trouvés, code de sortie 0 ;
- dans le dossier : `annotees/ensemble.png` (les quatre tags encadrés et numérotés)
  et `rapport.json`.

**Échec typique :**
- `exit code -11` ou « Segmentation fault » : version de `tags_image.py`
  antérieure au correctif. Reprendre la dernière version, puis `colcon build`.

## A2. Simulateur 2D : les tags sur la carte (automatique, ~12 min)

Le simulateur 2D contient 4 panneaux à tags dont on connaît la vraie position.
Le banc fait la carte (en notant les tags), puis un repérage sur la carte, et
mesure l'écart de chaque tag noté avec sa vraie place.

```bash
simulation/banc.sh 2d carte
simulation/banc.sh 2d reperage
```

**Attendu dans le rapport :**
```
[carte]      RÉUSSI  carte .../cartes/sim2d, 4 tags notés
               tag 1 : noté à 0.01 m de sa vraie place
               ...
[reperage]   RÉUSSI  ...
```

**Critères :**
- 4 tags notés ;
- écart de chaque tag ≤ 0,30 m.

Un écart plus grand sur un tag est la faiblesse connue des vues prises pendant
une rotation (`simulation/README.md`, section 7). La course y résiste, car
chaque visite repose sur une vue fraîche.

**La même chose à la main, avec RViz** (5 terminaux, préparés comme en 0) :
```bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST        # dans les 5 terminaux
python3 simulation/sim2d/simulateur.py                                  # 1 : le monde et le robot
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=sim \
    carte:=simulation/resultats/cartes/sim2d                            # 2 : localisation + tags
python3 simulation/sim2d/conduite_auto.py                               # 3 : parcours devant les tags
ros2 topic echo /fosa/tags_connus --once                                # 4 : tags retenus
rviz2                                                                   # 5 : Fixed Frame « map »
```
Pour conduire vous-même à la place de `conduite_auto.py` :
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.4
```

Dans RViz, ajouter :
- `Map` sur `/map` ;
- `MarkerArray` sur `/fosa/tags_marqueurs` : une flèche par tag, du tag vers sa
  pose de visite ;
- `Image` sur `/top_camera_color/image_raw`.

Dans les journaux du terminal 2 : `Tag 7 repéré en (x, y), pose de visite (...)`.
Les tags sont écrits dans `simulation/resultats/cartes/sim2d/tags.yaml`.

## A3. Gazebo PARC : tags fictifs, repérage, comparaison (~20 min)

Le monde PARC n'a pas de tags : `tools/tags_fictifs.py` en pose 4 après la
cartographie, à des places choisies sur la carte, et garde leur vraie position.

**1. Carte faite dans Gazebo (une fois)**
```bash
./run.sh gazebo                                                    # terminal 1, reste ouvert
./run.sh carte gazebo:=true map_name:=cafe_test                     # terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.4   # terminal 3
ros2 run caytu_nav_solution save_map                                # quand la carte est complète
```
Arrêter la cartographie (Ctrl-C dans le terminal 2).

**2. Poser les tags** (Gazebo toujours ouvert, robot au point de départ)
```bash
./run.sh tags_fictifs ~/fosa_maps/cafe_test --apercu    # choisir sans poser : regarder apercu.png
./run.sh tags_fictifs ~/fosa_maps/cafe_test             # poser les 4 tags dans Gazebo
```

**Attendu :**
- un tableau des 4 tags (repères map et monde) et « 4 tags posés » ;
- les panneaux visibles dans Gazebo ;
- `~/fosa_maps/cafe_test/tags_fictifs/apercu.png` : carte, tags en rouge (flèche =
  face), poses de visite en vert ;
- `verite.yaml` : les vraies positions.

Options :
- `--graine 2` : autres places ;
- `--positions "x,y,cap; ..."` : places imposées (repère map, cap en degrés) ;
- `--retirer` : enlever les tags.

**3. Repérage, comme au créneau**
```bash
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=sim carte:=$HOME/fosa_maps/cafe_test
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.4
```
Passer à 1–2 m de chaque tag, face à lui, et s'y arrêter 2 secondes. Le
registre préfère les vues proches prises sans tourner. Pour voir ce que voit
la caméra :
```bash
ros2 run rqt_image_view rqt_image_view /top_camera_color/image_raw
```

**4. Comparer à la vérité**
```bash
./run.sh tags_fictifs ~/fosa_maps/cafe_test --comparer
```
```
 tag  écart position  écart normale  vues
   1          0.08 m          3.1 °    24
```

**Critères :**
- les 4 tags vus ;
- écart de position ≤ 0,30 m, écart de normale ≤ 15°, au moins 3 vues par tag.

**À savoir :**
- Gazebo relancé : les tags disparaissent. Relancer l'étape 2 : les places
  restent les mêmes tant que la `--graine` ne change pas.
- La carte doit avoir été faite dans Gazebo (`world_from_map` dans
  `fosa_map.yaml`), sinon le script refuse.
- La taille du tag vient de `perception.apriltag.size_m` : le script le dit, et
  avertit si le workspace n'est pas sourcé.

## A4. La course : visites à 0,5 m jugées sur la vérité (~6 min par course)

```bash
simulation/banc.sh 2d course                                       # simulateur 2D
simulation/banc.sh gazebo course --carte ~/fosa_maps/cafe_test      # Gazebo PARC (relancé, robot au départ)
```

L'arbitre juge chaque visite sur la vraie pose du robot :
- centre du robot à 0,5 m ± 0,10 m du tag ;
- face au tag à ± 12° ;
- tags dans l'ordre publié ;
- robot immobile avant le départ, fin avant 10 min, aucun contact.

**Attendu :**
```
[course    ] RÉUSSI tags valides [7, 1, 9, 4] (ordre [7, 1, 9, 4]) ; fin 330 s ; ...
               VISITE tag 7 à 129 s : distance vraie 0.500 m, alignement +3.2° -> VALIDE
```

**Variantes :**
- `simulation/banc.sh 2d tag_absent` : un tag absent, il doit être sauté ;
- `simulation/banc.sh 2d reel` : interfaces du vrai robot ;
- `--type-ordre entiers|octets` : format de l'ordre ;
- `--ordre aleatoire`.

## A5. Avec le détecteur `apriltag_ros` (facultatif)

Le détecteur par défaut est OpenCV. Le nœud officiel `apriltag_ros` peut le
remplacer (`sudo apt install ros-jazzy-apriltag-ros`). Ajouter
`detecteur:=apriltag_ros` au repérage ou à la course :
```bash
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=sim carte:=$HOME/fosa_maps/cafe_test detecteur:=apriltag_ros
```
Même critère qu'en A3 (`--comparer`).

## A6. Ce qu'on peut regarder pendant un test

| Topic | Contenu |
|---|---|
| `/perception/tags` | détections brutes (identifiant, pose dans `map`, distance) |
| `/fosa/tags_vus` | chaque vue valide, telle que la course la reçoit |
| `/fosa/tags_connus` | position retenue de chaque tag (médiane des vues proches), toutes les secondes |
| `/fosa/tags_marqueurs` | flèches pour RViz |
| `<carte>/tags.yaml` | positions enregistrées (repère de la carte) |

```bash
ros2 topic hz /top_camera_color/image_raw            # la caméra publie-t-elle ?
ros2 topic echo /perception/tags --field detections  # que voit le détecteur ?
```

## A7. Dépannage AprilTags

| Symptôme | Cause et remède |
|---|---|
| `tag_detector` : `exit code -11` | ancienne version du détecteur avec OpenCV 4.6 : dernière version, `colcon build` |
| Aucun « Tag … repéré » | pas d'image (`ros2 topic hz` sur le topic caméra), tag trop loin ou trop de biais (> 65°), `FOSA_CONFIG_OVERRIDE` exporté par erreur en simulation |
| Distances fausses d'un même pourcentage | `perception.apriltag.size_m` différent de la taille du carré noir du tag |
| Un tag placé à 1–2 m de sa place | vues prises en tournant vite : repasser devant lui lentement et s'arrêter |
| `tags_fictifs` : « world_from_map absent » | carte faite hors de Gazebo : refaire la carte avec `gazebo:=true` |
| `tags_fictifs` : « Aucun monde Gazebo trouvé » | Gazebo pas lancé, ou lancé avec un autre `GZ_PARTITION` |

---

# B. Reconnaissance vocale du mot de départ

Le nœud `mot_depart` transcrit chaque segment de parole avec faster-whisper,
hors ligne (modèle `base`, réglages dans `fosa.yaml`, section `course.voice`).
Il publie le départ sur `/fosa/depart` seulement si le segment :
- dure entre 0,2 et 3 s, et est assez fort (`min_rms`) ;
- donne au plus 3 mots transcrits, dont « allez », « go » ou « vai » ;
- est transcrit avec confiance (sinon il est ignoré).

Il ne donne le départ **qu'une fois** : relancez-le pour un nouvel essai.

Deux façons de lui donner du son :
- **le micro du PC** (`audio_topic` vide) : il découpe lui-même les segments ;
- **le topic `/speech_audio`** (`audio_common_msgs/AudioData`, PCM 16 bits mono
  16 kHz) : les segments du ReSpeaker des organisateurs sur le robot, ou des
  fichiers WAV rejoués par le banc.

## B1. Au micro du PC (2 min)

```bash
ros2 topic echo /fosa/depart                                                   # terminal 1
ros2 run caytu_assistant mot_depart --ros-args -p audio_topic:="''"             # terminal 2
```
Attention à la syntaxe : `-p audio_topic:=""` est refusé par ROS 2.

**Attendu dans le terminal 2 :**
- `Modèle Whisper « base » prêt en … s. Mots de départ : allez, go, vai.` ;
- puis, pour chaque mot prononcé : `Entendu : « … » (… s).`

Dites, une seconde de silence entre chaque, et relancez le nœud après chaque
départ :

| Dire | Attendu |
|---|---|
| « bonjour » | `Entendu : « Bonjour. »`, pas de départ |
| « on y va » | pas de départ |
| applaudir, tousser | rien, ou une transcription sans départ |
| « allez » | `MOT DE DÉPART reconnu : départ de la course.` et des messages `{}` dans le terminal 1 |
| « go », « vai » | départ |

**Critères :**
- les trois mots de départ reconnus, au moins 4 fois sur 5 chacun ;
- aucun départ pour les autres mots ni pour les bruits.

Rien ne s'affiche quand vous parlez : le micro est trop faible. Montez-le
(`alsamixer`, ou les paramètres son), ou baissez `course.voice.min_rms` dans
`fosa.yaml`.

## B2. Banc audio automatique, comme le ReSpeaker (5 min)

Rejoue des fichiers WAV sur `/speech_audio`, vers un `mot_depart` neuf pour
chaque fichier. Il faut `ros-jazzy-audio-common-msgs`.

```bash
python3 simulation/outils/generer_audios.py simulation/audios       # bruits (+ voix de synthèse si espeak-ng)
# Vos voix, la vraie référence : 2 s chacune, nommées oui_* (doit partir) ou non_* (ne doit pas)
arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/oui_allez_hasina.wav
arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/oui_go_hasina.wav
arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/non_bonjour_hasina.wav
arecord -f S16_LE -r 16000 -c 1 -d 2 simulation/audios/non_allez_on_y_va_hasina.wav
simulation/banc.sh voix simulation/audios
```

`espeak-ng` (`sudo apt install espeak-ng`) ajoute des voix de synthèse. Whisper
les comprend moins bien que de vraies voix : seuls vos enregistrements font
foi. Enregistrez plusieurs personnes de l'équipe, à 1–2 m du micro, et avec
du bruit autour.

**Attendu :**
```
OK     oui_allez_hasina.wav             départ attendu : oui, obtenu : oui ; entendu « Allez. » (1.1 s).
OK     non_applaudissements.wav         départ attendu : non, obtenu : non ; entendu (segment ignoré : durée ou volume)
RESULTAT : 9/9 justes
```
Le détail est dans `simulation/audios/resultat_voix.json`, le journal du nœud
dans `journal_mot_depart.log`.

**Critère :** tous les fichiers justes. Un `oui_` raté : regarder ce que Whisper a
entendu (colonne « entendu »). Un `non_` qui fait partir la course est plus
grave : signalez-le.

## B3. Une course qui part à la voix (simulation, ~6 min)

Le PC n'a pas de ReSpeaker. On lance la course sans sa voix, et un
`mot_depart` au micro donne le départ à sa place, sur le même topic.

```bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST        # dans les 5 terminaux
python3 simulation/sim2d/simulateur.py                                  # 1
python3 simulation/outils/verifier_course.py                            # 2 : l'arbitre
python3 simulation/outils/ordre_organisateurs.py 7,1,9,4                # 3
ros2 launch caytu_nav_bringup course.launch.py robot:=sim \
    carte:=simulation/resultats/cartes/sim2d voix:=false                # 4
ros2 run caytu_assistant mot_depart --ros-args -p audio_topic:="''"     # 5, une fois « Prêt » affiché en 4
```
Dites d'abord « bonjour », puis « allez ».

**Attendu :**
- le robot ne bouge pas avant « allez » : l'arbitre l'écrit au départ (« le
  robot a bougé de 0.0 cm ») ;
- puis la course part (`STATUT … "evenement": "depart"` dans le terminal 4) et
  l'arbitre juge les visites.

La carte `simulation/resultats/cartes/sim2d` vient de A2 (`banc.sh 2d carte`).

## B4. Sur le robot, avec le ReSpeaker (au créneau)

Dans le terminal du robot (`deploy/connect.sh`) :
```bash
bash $FOSA_ROOT/installer_robot.sh            # faster-whisper + modèle en mémoire vive (Internet requis)
ros2 topic info /speech_audio -v              # type audio_common_msgs/msg/AudioData ?
ros2 topic hz /speech_audio                   # parler : un message par phrase
ros2 run caytu_assistant mot_depart           # écoute /speech_audio (réglage par défaut)
```
Mêmes essais qu'en B1, en parlant au robot.

**À confirmer auprès des organisateurs :** l'encodage de `/speech_audio`. Le
nœud suppose du PCM 16 bits mono à 16 kHz (`course.voice.sample_rate`). Si
Whisper n'entend que du bruit alors que la voix est claire, c'est le premier
suspect.

## B5. Dépannage voix

| Message | Remède |
|---|---|
| `TypeError: process() takes no keyword arguments` (téléchargement du modèle) | `~/ros2_ws/.venv/bin/python -m pip install "brotli>=1.2"` |
| `No module named 'faster_whisper'` | `export PYTHONPATH=~/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH`, ou lancer par `./run.sh` |
| `No module named 'audio_common_msgs'` | `sudo apt install ros-jazzy-audio-common-msgs` (ou `./install.sh`) |
| Téléchargement du modèle impossible (pas d'Internet) | copier `~/.cache/huggingface/hub/models--Systran--faster-whisper-base` depuis un PC qui l'a |
| `PortAudioError` / pas de micro | `arecord -l` doit lister le micro ; `sudo apt install libportaudio2` |
| `Couldn't parse parameter override rule: '-p audio_topic:='` | écrire `-p audio_topic:="''"` |
| Rien n'est « Entendu » | segment trop court ou trop faible : parler plus fort, `course.voice.min_rms` |
| Départ sur un faux mot | relever le mot entendu (`Entendu : « … »`) et me le signaler |

---

# Fiche de résultats

| Test | Date | Résultat | Remarques |
|---|---|---|---|
| A1 détection seule (4 tags, code 0) | | | |
| A2 2D carte : 4 tags, écarts ≤ 0,30 m | | | |
| A2 2D repérage : écarts ≤ 0,30 m | | | |
| A3 Gazebo `--comparer` : 4 tags ≤ 0,30 m, ≤ 15° | | | |
| A4 2D course : RÉUSSI | | | |
| A4 Gazebo course : RÉUSSI | | | |
| B1 micro : allez / go / vai reconnus, aucun faux départ | | | |
| B2 banc audio : tous justes | | | |
| B3 course partie à la voix, immobile avant | | | |
| B4 robot : `/speech_audio` reçu, « allez » reconnu | | | |
