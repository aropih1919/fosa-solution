# ÉQUIPE FOSA — PARC Engineers League 2026

Solution globale pour le robot d'accueil et de guidage CAYTU Sito-É combinant navigation autonome robuste sous Nav2, perception multimodale (LiDAR, caméras de profondeur, balises AprilTag) et assistant vocal intelligent avec validation déterministe.

**Pays :** Madagascar
**Équipe FOSA**

> **Lancer la solution (PC et robot réel, arbitre officiel) :
> [`LANCEMENT.md`](LANCEMENT.md).**

---

## 1. Architecture des 7 paquets

Le projet est conçu de façon modulaire et lisible :

| Paquet | Description & Rôle |
|---|---|
| [`caytu_config`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config) | **Source unique de configuration** : tous les seuils, vitesses, topics, modèles, lieux et FAQ sont centralisés dans 4 fichiers YAML validés par schémas stricts. |
| [`caytu_interfaces`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_interfaces) | **Messages et services ROS 2** : contrats d'échange typés (`SemanticLocation`, `DetectedEntity`, `TagDetection`, `DialogueState`, etc.). |
| [`caytu_semantic_map`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_semantic_map) | **Carte sémantique du site** : résolution des noms de stands, alias et tags AprilTag avec calcul de la pose d'approche devant chaque point d'intérêt. |
| [`caytu_perception`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_perception) | **Vision et détection d'obstacles** : détection de balises AprilTag, classification 3D par nuages de points (personnes vs mobilier) et suivi de visiteurs. |
| [`caytu_assistant`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_assistant) | **Intelligence et interaction vocale** : reconnaissance vocale (Whisper local/API), compréhension contrainte avec LangChain (Ollama local ou API compatible OpenAI), dispatcher d'actions déterministe et synthèse vocale (Piper/API). |
| [`caytu_nav_solution`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_nav_solution) | **Solution de navigation** : point d'entrée officiel Phase 1 (`task_solution.py`), client d'action `Nav2Navigator` réutilisable, cartographie manuelle (`mapping_session`, `save_map`, `edit_map`). |
| [`caytu_nav_bringup`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_nav_bringup) | **Bringup matériel et Nav2** : filtrage des capteurs (masquage châssis LiDAR, caméras), profils de contrôleurs (RPP, DWB), localisation odom+IMU ou slam_toolbox, cartographie (`mapping.launch.py`) et profils de robot (`config/robots/` : Gazebo, robot réel). |

> [!IMPORTANT]
> **Règle de sécurité absolue** : Le modèle de langage (LLM) ne génère **jamais** de coordonnées spatiales ni de vitesse moteur. Il extrait une intention et un nom de lieu. Le dispatcher déterministe valide le lieu et pilote Nav2 en toute sécurité.

---

## 2. Guide des commandes — Que lancer et comment ?

### 2.1 Installation sur un nouveau PC
```bash
cd ~/ros2_ws/src/fosa-solution
./install.sh --models
```
L'installateur prépare Ubuntu 24.04 et installe ce que la course utilise :
- ROS 2 Jazzy et le dépôt officiel PARC (simulateur, arbitre `parc_referee`,
  messages `parc_msgs`) ;
- Nav2, slam_toolbox et le garde-fou `nav2_collision_monitor` ;
- les AprilTags (`apriltag_ros`, le détecteur de la course ; OpenCV pour
  l'image JPEG de l'arbitre), `depth_image_proc`, `audio_common_msgs` ;
- faster-whisper et le modèle Whisper du mot de départ (`course.voice.model`).

Puis il compile le workspace. Sans `--models`, il demande s'il faut télécharger
le modèle ; plus tard : `./install.sh --models`. Ce script ne concerne que les
PC : le robot se prépare avec `deploy/`.

Copier le modèle Whisper sur un PC sans Internet : copiez le dossier
`~/.cache/huggingface/hub/models--Systran--faster-whisper-base` du premier PC au
même endroit sur l'autre.

`run.sh` source automatiquement ROS, le workspace compilé et l'environnement Python.

---

### 2.2 Solution complète — Simulation et assistant
> Phases précédentes. L'assistant vocal (LLM, voix Piper, YOLO) n'est plus
> installé par `install.sh` et ne sert pas à la course : ce mode ne fonctionne
> plus tel quel.

```bash
./run.sh complet
```
Par défaut, l'assistant utilise le mode microphone configuré. Pour saisir le
texte au clavier :
```bash
./run.sh complet input_mode:=text
```

### 2.3 Navigation autonome officielle
La simulation doit d'abord être lancée dans un terminal :

**Terminal 1 — Lancement de la simulation officielle :**
```bash
./run.sh gazebo
```

**Terminal 2 — Lancement de la solution Fosa :**
```bash
./run.sh navigation
```

Ce mode lance `task_solution.py` avec le moniteur actif et enregistre un rapport
par navigation dans `results/`. Le contrôleur, le profil et la localisation
proviennent de `caytu_config/config/fosa.yaml`. Les sorties détaillées restent
dans chaque dossier de rapport; les erreurs et le suivi du robot restent visibles.

---

### 2.4 Assistant seul et test TTS
> Phases précédentes. L'assistant vocal (LLM, voix Piper, YOLO) n'est plus
> installé par `install.sh` et ne sert pas à la course : ce mode ne fonctionne
> plus tel quel.


**Mode interactif en console (Idéal pour tester sans microphone) :**
```bash
./run.sh assistant input_mode:=text
```
*Tapez directement vos phrases dans le terminal (`Où sont les toilettes ?`, `Oui`, `Stop`, `À quelle heure est la finale ?`, `J'ai mal`).*

**Mode vocal complet (avec microphone et haut-parleur) :**
```bash
./run.sh assistant input_mode:=microphone
```

**Essai direct de la synthèse vocale :**
```bash
./run.sh tts "Bonjour, ceci est un essai de synthèse vocale."
```

---

### 2.5 Outil de diagnostic en direct
Permet de mesurer en temps réel la dérive de localisation, les faux obstacles et les contacts :
```bash
./run.sh diagnostic --ros-args -p use_sim_time:=true
```

---

Les outils du laboratoire restent accessibles avec `./run.sh benchmark` et
`./run.sh compare`.

### 2.6 Lancement des tests unitaires
Vérifie la robustesse de tous les modules sans nécessiter de simulateur :
```bash
colcon test --packages-select caytu_config caytu_nav_bringup caytu_nav_solution caytu_semantic_map caytu_assistant caytu_perception
colcon test-result --verbose
```

---

### 2.7 Cartographie manuelle (phase finale)
L'arène de Dakar est un monde nouveau : on en fait d'abord la carte en
conduisant le robot au clavier, puis la navigation se localise dessus.
```bash
./run.sh carte gazebo:=true map_name:=cafe_test rviz:=true        # essai dans Gazebo
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/robot_base_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.6
ros2 run caytu_nav_solution save_map                              # carte dans ~/fosa_maps/
ros2 run caytu_nav_solution task_solution.py --ros-args -p slam_map:=$HOME/fosa_maps/cafe_test
```
Mode d'emploi complet (retouches avec `edit_map`, rejeu, diagnostic) :
[`docs/CARTOGRAPHIE.md`](docs/CARTOGRAPHIE.md).

### 2.8 Course de la phase finale (AprilTags)
Préparation pendant les essais : la cartographie note aussi la position des
tags (`<carte>/tags.yaml`). Sur un PC où le dépôt est installé :
```bash
./run.sh course robot:=real_remote carte:=$HOME/fosa_maps/arene_dakar   # robot immobile
./run.sh depart                                                          # ou dire « allez »
```
Sur le robot (`run.sh` n'y est pas envoyé), dans le terminal ouvert par
`deploy/connect.sh` :
```bash
ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
ros2 run caytu_nav_solution depart                                       # ou dire « allez »
```
L'ordre des tags est lu sur `/parc/tag_order` (arbitre officiel). Le départ
est signalé sur `/parc/start`. Le robot s'arrête à 0,5 m face à chaque tag et
publie `/parc/tag_visit` (identifiant et image JPEG), puis passe au suivant.
La course s'arrête quand l'arbitre la déclare finie (`/parc/run_status`). Un garde-fou (`nav2_collision_monitor`) l'arrête devant tout objet,
personnes comprises. Mode d'emploi : [`docs/COURSE.md`](docs/COURSE.md).
En simulation, `./run.sh tags_fictifs ~/fosa_maps/<carte>` pose 4 tags fictifs
dans Gazebo après la cartographie (section 2 bis du mode d'emploi).

### 2.9 Robot réel de Dakar
Les topics du robot réel diffèrent de Gazebo (relevé officiel du 2 novembre) :
commande sur `/cmd_vel`, odométrie `/botwheel_explorer/odom`, pas d'IMU de
base, caméras sans nuages de points, `/scan_filtered` réservé aux
organisateurs (le nôtre devient `/fosa/scan_filtered`). Deux réglages vont
ensemble :
- la navigation : `robot:=real` (solution sur le robot) ou `robot:=real_remote`
  (solution sur le portable, sans caméras) ;
- caytu_config : `export FOSA_CONFIG_OVERRIDE=$PWD/caytu_config/config/robot_reel.yaml`
  (temps réel, domaine 0, `/cmd_vel`, caméras du robot pour la perception).

Le compte du robot est partagé entre les équipes : la solution n'y reste que
le temps du créneau (mode B). `deploy/deploy.sh --installer --map <carte>`
envoie la solution dans notre dossier d'équipe, installe ce qui manque et la
compile par-dessus `~/ros2_ws` des organisateurs. `deploy/connect.sh` ouvre un
terminal sur le robot, `deploy/clean.sh` rapatrie les cartes et efface tout.
Pour un robot sans Internet, `deploy/telecharger_pour_robot.sh` prépare
faster-whisper et son modèle sur le portable (`deploy.sh --paquets`). Sans
code sur le robot (mode A), la solution tourne sur le portable par le Wi-Fi
(`deploy/check_link.sh`). Ces scripts se lancent sur le portable ; les commandes `ros2`
se tapent dans le terminal du robot. Détails et vérifications du premier créneau :
[`docs/ROBOT_REEL.md`](docs/ROBOT_REEL.md).

### 2.10 Tout simuler sans le robot
`simulation/banc.sh 2d tout` rejoue toute la phase finale dans un simulateur 2D
(cartographie, repérage, course, personne qui traverse, tag absent, interfaces
du vrai robot) et juge chaque course sur la vraie pose du robot ;
`simulation/banc.sh gazebo course --carte <carte>` fait de même dans le Gazebo PARC,
`simulation/banc.sh 2d arbitre` fait juger la course par l'arbitre officiel
`parc_referee`, `simulation/banc.sh voix` teste le mot de départ. Mode d'emploi :
[`simulation/README.md`](simulation/README.md).

---

## 3. Documentation de référence

Pour approfondir, six guides complets sont à votre disposition (le lancement
pas à pas est dans [`LANCEMENT.md`](LANCEMENT.md)) :
1. **[Contexte & Architecture détaillée](file:///home/johny-olivier/ros2_ws/src/fosa-solution/docs/CONTEXTE_ET_ARCHITECTURE.md)** : description approfondie des 7 paquets, de la configuration centralisée (`fosa.yaml`) et des conventions de code `refactor/franglais`.
2. **[Guide des tests pas-à-pas](file:///home/johny-olivier/ros2_ws/src/fosa-solution/docs/GUIDE_TESTS.md)** : scénarios complets de validation manuelle pour la Phase 1 et la Phase 2.
3. **[Cartographie manuelle](docs/CARTOGRAPHIE.md)** : faire, vérifier, retoucher et réutiliser la carte d'un monde nouveau.
4. **[Robot réel](docs/ROBOT_REEL.md)** : topics officiels, vérifications du premier créneau, travail par SSH sans téléverser le code.
5. **[Course finale](docs/COURSE.md)** : carte et tags pendant les essais, lancement et départ de la course, démarrage vocal, questions ouvertes.
6. **[Tests AprilTags et voix](docs/TESTS_TAGS_ET_VOIX.md)** : localisation des tags en simulation (2D et Gazebo, comparée à la vérité) et reconnaissance du mot de départ (micro, fichiers audio, course partie à la voix), avec fiche de résultats.
