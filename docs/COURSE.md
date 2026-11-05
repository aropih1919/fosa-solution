# Course de la phase finale : mode d'emploi

Tâche (guide officiel PARC 2026, phase réelle) : visiter les tags 1, 4, 7, 9
dans l'ordre publié au départ. Une visite compte quand le robot s'arrête,
**centre à 0,5 m du tag, face à lui à ±12°**, et garde la position. Chaque
visite doit être signalée (identifiant + image de la caméra). 10 minutes au
plus, aucune intervention après le départ, bonus de 5 points si la course
démarre au mot de départ.

## 1. Ce que fait la solution

| Besoin | Où | Comment |
|---|---|---|
| Carte de l'arène | `mapping.launch.py` | slam_toolbox, robot piloté au clavier pendant les essais |
| Position des tags | `tags.launch.py` (inclus) | détecteur OpenCV ou `apriltag_ros`, puis `registre_tags` : médiane des vues proches, écrite dans `<carte>/tags.yaml` |
| Ordre des tags | `course_tags` | lu sur `course.order_topic` = `/parc/tag_order` (`parc_msgs/TagOrder` de l'arbitre officiel, verrouillé ; liste vide ignorée). Le type est détecté seul : texte « 7,1,9,4 » ou tableau d'entiers acceptés aussi en simulation |
| Départ | `mot_depart` ou `depart` | mot « allez » (Whisper hors ligne) ou commande ; rien ne bouge avant |
| Aller au tag | `course_tags` + Nav2 | tour de repérage, Nav2 jusqu'à 1 m face au tag, puis approche finale guidée par la vue du tag jusqu'à 0,5 m, aligné à 4° près. Une visite n'est déclarée que si la caméra voit le tag à l'arrivée |
| Positions notées pendant les essais | `<carte>/tags.yaml` | simples indices pour savoir où chercher : la carte est corrigée après coup, ces positions peuvent être décalées de plusieurs dizaines de centimètres |
| Prévenir l'arbitre | `arbitre_parc.py` | `/parc/start` au départ (commande ou voix + mot entendu), `/parc/tag_visit` à chaque tag (identifiant + image JPEG de la caméra du haut), `/parc/finish` seulement pour finir en avance ; suit `/parc/run_status` et s'arrête à `FINISHED`, `TIMED_OUT`, `TERMINATED` |
| Journal de la course | `statut_course.py` | JSON de statut (départ, visites, fin) dans le journal et sur `/fosa/...`, images enregistrées |
| Tag introuvable | `course_tags` | exploration de la carte, puis tag sauté après `search_timeout_sec` (règle : un tag sauté ne peut plus compter) |
| Obstacles mobiles, personnes | `nav2_collision_monitor` + costmaps | arrêt net à 8 cm du pare-chocs, ralenti jusqu'à 0,70 m, quel que soit l'objet |
| Cartes interdites vendredi ? | `course.launch.py carte:=` | cartographie en direct pendant la course, tags cherchés en explorant |

Réglages : `caytu_config/config/fosa.yaml`, section `course` (topics, distances,
vitesses, temps) ; robot réel : `caytu_config/config/robot_reel.yaml`.

## 2. Pendant les créneaux d'essai : carte + tags

Où taper ces commandes (détail : [ROBOT_REEL.md](ROBOT_REEL.md), section 4) :

| | Mode A : solution sur le portable | Mode B : solution envoyée sur le robot |
|---|---|---|
| Terminal | portable, dossier du dépôt | robot, ouvert par `deploy/connect.sh` après `deploy/deploy.sh` |
| Profil | `robot:=real_remote` (lidar seul) | `robot:=real` (caméras actives) |
| Dossier des cartes | `$HOME/fosa_maps` | `$FOSA_MAPS_DIR` (mémoire vive, rapatrié par `deploy/clean.sh`) |
| `FOSA_CONFIG_OVERRIDE` | à exporter (ligne ci-dessous) | déjà réglé |

Les exemples ci-dessous sont écrits pour le mode A ; en mode B, remplacer le
profil et le dossier des cartes. Les scripts `deploy/*.sh` se lancent
toujours sur le portable, jamais sur le robot.

```bash
export FOSA_CONFIG_OVERRIDE=$PWD/caytu_config/config/robot_reel.yaml   # robot réel
ros2 launch caytu_nav_bringup mapping.launch.py robot:=real_remote map_name:=arene_dakar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
```

Conduisez lentement, en passant **à 1–2 m de chaque tag, face à lui, et en vous
y arrêtant 2 secondes** : le registre ne garde que les vues prises sans
tourner vite, et préfère celles à moins de 1,6 m. Les journaux annoncent
« Tag 7 repéré en (x, y), pose de visite (...) ». Dans RViz, `/fosa/tags_marqueurs`
montre une flèche du tag vers sa pose de visite.

```bash
ros2 run caytu_nav_solution save_map
```

La carte est corrigée par slam_toolbox jusqu'à la fin : les tags notés pendant
la cartographie peuvent être décalés dans la carte finale (jusqu'à 1–2 m dans
notre simulateur, dont l'odométrie est volontairement mauvaise). Si le créneau
le permet, **refaites un passage de repérage** sur la carte enregistrée, robot
reparti de la position de départ : les tags sont alors notés dans le repère
même de la course.

```bash
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=real_remote \
    carte:=$HOME/fosa_maps/arene_dakar
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4
cat ~/fosa_maps/arene_dakar/tags.yaml        # 4 tags attendus
```

Dans tous les cas, ces positions ne sont que des indices : la course revoit
chaque tag avant de le déclarer visité.

Puis répétition de la course (ordre imposé, essais seulement) :

```bash
ros2 launch caytu_nav_bringup course.launch.py robot:=real_remote \
    carte:=$HOME/fosa_maps/arene_dakar voix:=false ordre:=7,1,9,4
ros2 run caytu_nav_solution depart            # autre terminal
```

## 2 bis. En simulation, sans tags : tags fictifs

Le monde Gazebo de la cartographie n'a pas de tags. `tools/tags_fictifs.py`
en pose 4 (identifiants de `course.expected_ids`) **après** la
cartographie, choisis sur la carte enregistrée :
- **où :** devant un mur droit, avec de la place devant ; les poses
  d'attente et de visite sont atteignables depuis le départ ;
- **écart :** au moins 1,5 m entre deux tags ;
- **forme :** un panneau blanc sur un pied, centre du tag à 1 m du sol, posé
  20 cm devant le mur ;
- **taille :** celle du détecteur (`perception.apriltag.size_m`).

Le motif est fait de pavés noirs, sans texture : le rendu ne peut pas
l'inverser.

Il faut une carte faite **dans Gazebo** : `fosa_map.yaml` contient alors
`world_from_map`, qui relie la carte au monde.

```bash
# 1. Cartographie dans Gazebo (comme d'habitude), puis save_map
./run.sh carte gazebo:=true map_name:=cafe_test
ros2 run caytu_nav_solution save_map

# 2. Gazebo toujours ouvert, workspace sourcé : poser les tags
./run.sh tags_fictifs ~/fosa_maps/cafe_test          # ou : python3 tools/tags_fictifs.py ...
#    --apercu : choisir seulement (image <carte>/tags_fictifs/apercu.png), rien dans Gazebo
#    --graine 2 : autres places ; --positions "x,y,cap_deg; ..." : places imposées (repère map)
#    --retirer : enlever les tags de Gazebo

# 3a. Repérage, comme sur le vrai robot : passer devant chaque tag
ros2 launch caytu_nav_bringup reperage_tags.launch.py robot:=sim carte:=$HOME/fosa_maps/cafe_test
./run.sh tags_fictifs ~/fosa_maps/cafe_test --comparer   # écart registre / vérité, par tag
# 3b. Ou, pour aller vite : positions exactes écrites dans tags.yaml
./run.sh tags_fictifs ~/fosa_maps/cafe_test --tags-yaml

# 4. Course
ros2 launch caytu_nav_bringup course.launch.py robot:=sim \
    carte:=$HOME/fosa_maps/cafe_test voix:=false ordre:=7,1,9,4
ros2 run caytu_nav_solution depart
```

Les tags vivent dans Gazebo : si Gazebo est relancé, relancer l'étape 2. Les
places sont les mêmes, car le tirage est fixe tant que `--graine` ne change
pas. Le robot doit repartir du point de départ de la cartographie, le spawn
de la tâche. Les positions vraies sont dans `<carte>/tags_fictifs/verite.yaml`.

Vérifié dans Gazebo Harmonic 8.10, sans écran :
- les 4 tags apparaissent à la pose prévue ;
- une caméra placée à la pose de visite (0,5 m, de face) lit le tag 7 ;
- une caméra à la pose d'attente (1 m, 30° de biais) lit le tag 4 ;
- `--retirer` les enlève, une relance les remplace.

Pas vérifié : avec la caméra du robot PARC (sa hauteur et son champ de vue) ;
si le tag sort de l'image de près, changer `--hauteur`.

Pour enchaîner toutes ces simulations et obtenir un rapport « réussi / raté »,
voir [`simulation/README.md`](../simulation/README.md) (`simulation/banc.sh`).

## 3. Vendredi : la course

1. Robot à la position de départ, **celle où la cartographie a commencé**
   (slam_toolbox repart de là).
2. Lancer la solution (le robot reste immobile) :
   ```bash
   ros2 launch caytu_nav_bringup course.launch.py robot:=real carte:=$FOSA_MAPS_DIR/arene_dakar
   ```
   (mode B, après `deploy/deploy.sh --installer --paquets --map arene_dakar` et
   `deploy/connect.sh`, voir [`LANCEMENT.md`](../LANCEMENT.md)).
   Le juge arme la course : l'ordre arrive sur `/parc/tag_order`.
   Attendre « Ordre des tags reçu : [...] » et « Prêt. Départ sur /fosa/depart ».
3. Départ : dire **« allez »**, ou `ros2 run caytu_nav_solution depart`. La
   solution envoie `/parc/start` (méthode voix + mot entendu, ou commande).
4. Ne plus toucher à rien. Le robot fait un tour de repérage (~16 s), visite
   les tags (un `/parc/tag_visit` avec image à chaque tag, puis 5 s
   d'immobilité ; l'arbitre en exige 3). L'arbitre clôt la course au dernier
   tag signalé dans l'ordre ; le robot s'arrête dès que `/parc/run_status`
   n'est plus `RUNNING`.

Le robot ne doit pas bouger entre l'armement et le départ (faux départ
contrôlé sur l'odométrie). Ne publier sur `/parc/*` que pendant notre créneau.

Si les cartes préparées sont interdites : même commande avec `carte:=` (vide).

## 4. Démarrage vocal (bonus +5)

Le guide exige une reconnaissance **hors ligne, sur le robot**, avec le
ReSpeaker. `mot_depart` écoute les segments de parole `/speech_audio` du nœud
ReSpeaker des organisateurs et les transcrit avec faster-whisper (modèle
`base`, int8, CPU). Il ne donne le départ que pour un segment court (0,2 à 3 s),
assez fort, transcrit en trois mots au plus dont « allez » (ou « go », « vai »),
avec une transcription confiante : un bruit du public ne doit jamais faire
partir le robot.

Installation à chaque créneau, dans notre dossier d'équipe comme le reste
(rien n'est téléchargé pendant la course) : `deploy/deploy.sh --installer`.
Si le robot n'a pas Internet, le portable prépare tout avant
(`deploy/telecharger_pour_robot.sh` : roues aarch64 et modèle) et
`deploy/deploy.sh --paquets --installer` l'installe sans réseau. `env.sh` met
`$FOSA_ROOT/py` dans le `PYTHONPATH` et le modèle dans `$FOSA_ROOT/hf` :
effacés par `clean.sh`.
Essai : `ros2 run caytu_assistant mot_depart`, puis dire « allez » au robot,
« bonjour », « on y va », applaudir : seul « allez » doit afficher
« MOT DE DÉPART reconnu ».

`/speech_audio` : un message par phrase, après 0,5 s de silence, PCM 16 bits
signé, 16 kHz, mono (README officiel du 5 novembre ; `course.voice.sample_rate`).
Le mot entendu part vers l'arbitre dans `StartSignal.heard`.

## 5. Détecteur d'AprilTags : apriltag_ros

Choix de l'équipe (5 novembre) : `perception.apriltag.detector: apriltag_ros`
dans `fosa.yaml`, pour la carte, le repérage et la course, sur le PC comme sur
le robot.
- `apriltag_ros` (défaut) : nœud ROS `apriltag_ros/apriltag_node`
  (`sudo apt install ros-jazzy-apriltag-ros`), sur l'image couleur et le
  `camera_info` de la caméra du haut (`camera1`). Il publie `/detections` et la
  pose de chaque tag en TF `tag36h11:<id>`. `registre_tags` les place dans la
  carte (`tags.yaml`, `/fosa/tags_vus`) ; la course s'en sert pour viser et
  confirmer chaque visite. Paquet absent : le lancement s'arrête avec la
  commande d'installation, plutôt que de partir sans détecteur.
- `opencv` : `tag_detector` de caytu_perception (cv2.aruco), **secours
  seulement**, au lancement : `detecteur:=opencv`.

OpenCV reste utilisé ailleurs, mais pas pour détecter : il encode l'image JPEG
envoyée à l'arbitre (`/parc/tag_visit`), comme le demande le README officiel.

Taille des tags : `perception.apriltag.size_m` = côté du carré noir (0,165 m
selon le guide, dans `robot_reel.yaml` ; 0,132 m si 16,5 cm compte la bordure
blanche ; 0,12 m dans le simulateur officiel, `sim_parc.yaml`). À mesurer au
premier créneau : une erreur de taille fausse la distance dans la même
proportion.

## 6. Ce qui reste inconnu (à demander aux organisateurs)

Réponses des organisateurs (5 novembre) : ordre sur `/parc/tag_order`,
départ, visites et fin sur `/parc/*` (`parc_msgs`), 3 s d'immobilité après
chaque visite, `/speech_audio` en PCM 16 bits 16 kHz. Il reste :

| Question | Réglage |
|---|---|
| Taille exacte des tags (carré noir) | `perception.apriltag.size_m` |
| L'`obstacle_avoidance` des organisateurs bloque-t-il l'approche à 0,5 m ? | essai au premier créneau |
| `camera1` est-elle bien la caméra haute du vrai robot ? | `tf2_echo base_link camera1_link` |

## 7. Ce qui a été vérifié

**Avec apriltag_ros** (5 novembre, après le passage à ce détecteur) : banc 2D
complet, carte → repérage → course jugée par l'arbitre officiel, ordre
`9,4,1,7`, départ vocal. Carte : 4 tags notés. Course : `FINISHED` à 286 s,
4 visites « in order », « image match », « hold ok », bonus vocal compté,
distance vraie 0,515 à 0,516 m, alignement ±3,2°, aucun contact. Ce banc a
révélé un défaut, corrigé : apriltag_node publie `/detections` avant la TF du
tag. Sans attente, le registre jetait toutes les vues (0 tag noté) ; il garde
désormais chaque image jusqu'à 0,5 s, le temps que la TF arrive.

**Simulateur officiel PARC** (seed 7, sans fenêtre, sur une machine sans carte
graphique : environ 1/15 du temps réel). Notre cartographie tourne avec
`robot:=parc`, `sim_parc.yaml` et apriltag_ros. Premières vues, robot presque
immobile : tag 7 à 8 cm de sa vraie place (vu à 2,8 m), tag 1 à 19 cm (vu à
5 m). La taille de 0,12 m et la caméra `camera1` sont donc cohérentes. Pendant un
tour complet sur place, en revanche, le cap du robot dans la carte a dérivé
(+70° à 270° de rotation) : en fin de tour, le tag 9 a été placé à 4 m de sa
place et le tag 7 a glissé à 0,8 m. La cause (odométrie du simulateur en
rotation, ou slam_toolbox ralenti par cette machine) reste à établir sur un
PC rapide. La course complète dans ce simulateur n'a pas pu être jouée ici, faute
de vitesse.

**Avec l'arbitre officiel** (`parc_referee` du dépôt PARC, 5 novembre) :
simulateur 2D aux interfaces du vrai robot (`camera1` en haut), armé par son
API comme le juge, ordre `9,4,1,7` sur `/parc/tag_order`, départ par commande.
Verdict de l'arbitre : `FINISHED` (dernier tag dans l'ordre) à 392 s, aucun
faux départ, chaque visite « in order », « image match » (l'arbitre lit le bon
tag sur notre JPEG), « hold ok ». Vérité terrain : 0,489 à 0,511 m, alignement
+1,2° à +4,0°, aucun contact. Commande : `simulation/banc.sh 2d arbitre`.

Deuxième essai, ordre `7,1,9,4`, **départ vocal** (le mot « allez » tel que le
publie `mot_depart`) : l'arbitre a reçu `method 1`, `heard « allez »`, et
compte le bonus vocal (5). Tags 7 et 1 : « in order », « image match »,
« hold ok ». En allant au tag 9, le robot est resté bloqué environ 140 s
contre un meuble du café 2D (arrêts du garde-fou, « Failed to make progress »).
Le tag 9 a été sauté à 532 s, puis la course s'est arrêtée à 600 s
(`TIMED_OUT`). Les interfaces de l'arbitre ne sont pas en cause : c'est la
navigation, déjà lente dans les passages étroits en mode `reel`
(461 s le 4 novembre).

Simulateur 2D du café : 4 panneaux portant les tags 1, 4, 7, 9 à 1 m de haut,
répartis sur ~8 m (plus grand que l'arène de 5 m), caméra couleur simulée
640×480 (~69°), odométrie au cap faussé de 15 % comme Gazebo.

Chaîne complète : cartographie + tags → repérage sur la carte → course avec
l'ordre `7,1,9,4` publié sur `/tag_order` (String, type détecté seul) → départ
par `ros2 run caytu_nav_solution depart`. Vérité terrain du simulateur :

| Tag | Visite à | Distance vraie centre → tag | Alignement |
|---|---|---|---|
| 7 | 114 s | 0,509 m | +4,7° |
| 1 | 181 s | 0,513 m | +2,5° |
| 9 | 229 s | 0,522 m | −4,7° |
| 4 | 322 s | 0,513 m | +3,5° |

Fin signalée à 330 s, aucun contact, image publiée à chaque visite. Règle :
0,5 m et ±12°.

Deuxième essai, solution envoyée par `deploy/deploy.sh` sur un « robot »
simulé par SSH et lancée depuis son espace en mémoire (mode B), ordre
`9,4,7,1` : tag 9 à 54 s (0,516 m, +3,2°), 4 à 125 s (0,493 m, +2,9°),
7 à 181 s (0,499 m, −2,8°), 1 à 254 s (0,510 m, +1,9°), fin à 262 s, aucun
contact, puis `deploy/clean.sh` a tout effacé.

Aussi vérifié : détection et enregistrement avec `apriltag_ros` (nœud ROS
officiel) sur la caméra simulée ; robot immobile tant que le départ n'est pas
donné ; tests unitaires (ordre des tags, géométrie, approche, registre, mot de
départ, zones du garde-fou).

Défauts trouvés et corrigés pendant ces essais : visite déclarée sans voir le
tag (désormais impossible), positions notées décalées (devenues de simples
indices), valeurs NaN qui empoisonnaient une position, Nav2 qui ne démarrait
pas (relancé par la course), approche qui hésitait (cible lissée).

**Pas vérifié :** le vrai robot, de vraies images (éclairage, flou de
mouvement), Whisper sur le Jetson (modèle non téléchargeable ici : seule la
logique de décision est testée), le paquet de statut des organisateurs. Deux
essais complets à 4/4, sur la même arène simulée : les essais précédents,
avant corrections, en avaient réussi 2 ou 3.
