# Audit des interfaces PARC et Fosa

Date : 5 novembre 2026. Code Fosa examiné : merge `9b34c23`, branche
`travail/phase-reelle-merge-9b34c23`.

## Conclusion

Les noms et types des entrées matérielles utilisées par la course correspondent
à l'inventaire du robot réel. Les cinq interfaces de l'arbitre correspondent
aux définitions présentes dans `../parc_msgs/msg` et au code de `../parc_referee`.
La configuration Nav2 générée utilise les bons topics et le type `Twist`.

Cela valide le contrat déclaré, pas la disponibilité actuelle sur le robot.
Les points restant à vérifier sont surtout les liaisons TF, le format audio,
la réception effective des événements par l'arbitre et l'isolation des builds.

## Sources et portée

- `../README.md`, `../parc_robot_bringup/config/gz_bridge.yaml`,
  `../parc_robot_bringup/launch/robot_interface.launch.py` : simulateur PARC
  présent dans le workspace d'analyse de Liantsoa.
- `../parc_msgs/msg/*.msg` et `../parc_referee/parc_referee/referee_node.py` :
  messages et endpoints de l'arbitre local.
- `parc-phase-finale-docs/parc_ros2_topics_updated.md` : inventaire du robot
  physique au 2 novembre, avec types, QoS et arbre TF. Il ne contient pas encore
  les topics de l'arbitre.
- `parc-phase-finale-docs/informations-parc.txt` : annonce ultérieure de
  l'installation de l'arbitre et de `parc_msgs` sur le robot réel.
- `../parc_robot` contient un métapaquet ; ce n'est pas le code des pilotes
  matériels du vrai robot. Le simulateur ne suffit donc pas à prouver leur
  comportement réel.
- Côté solution : configuration, launch de course, génération Nav2, perception,
  entrées/sorties de course, départ vocal et outils de diagnostic.

## 1. Interfaces matérielles utilisées

Dans les tableaux, `N` vaut séparément `1` et `2`.
Les types abrégés sont des types ROS `paquet/msg/Type`.

| Topic | Type | Robot réel documenté / simulateur | Utilisation Fosa |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Abonné réel : `obstacle_avoidance` ; simulation : pont Gazebo | Publication Nav2 et course, remappings corrects |
| `/botwheel_explorer/odom` | `nav_msgs/Odometry` | Présent dans les deux | Nav2 et suivi technique |
| `/scan_filtered` | `sensor_msgs/LaserScan` | Présent dans les deux ; filtrage simulé approximatif | SLAM, costmaps et garde de proximité de la course |
| `/cameraN/cameraN/color/image_raw` | `sensor_msgs/Image` | Présent dans les deux | Détection AprilTag et image de preuve |
| `/cameraN/cameraN/color/camera_info` | `sensor_msgs/CameraInfo` | Présent dans les deux | Calibration de la détection |
| `/cameraN/cameraN/depth/image_rect_raw` | `sensor_msgs/Image` | Présent dans les deux | Projection des obstacles dans le repère robot |
| `/cameraN/cameraN/depth/camera_info` | `sensor_msgs/CameraInfo` | Présent dans les deux | Calibration de la projection |
| `/speech_audio` | `audio_common_msgs/AudioData` | Présent dans les deux | Reconnaissance du mot de départ |
| `/tf` | `tf2_msgs/TFMessage` | Présent dans les deux | Transformations dynamiques ; Fosa ajoute `map -> odom` via SLAM |
| `/tf_static` | `tf2_msgs/TFMessage` | Présent dans les deux | Transformations des capteurs |

Références : `caytu_config/config/fosa.yaml:9-160`,
`caytu_course/launch/course.launch.py:43-100`,
`caytu_nav_solution/caytu_nav_solution/parametres_nav2.py:170-211`.

### QoS et données

- Caméras et laser : les abonnements Python utilisent `qos_profile_sensor_data`
  (`BEST_EFFORT`, `VOLATILE`), compatible avec les publications `RELIABLE`,
  `VOLATILE` observées sur le vrai robot.
- Audio : abonnement `RELIABLE`, `VOLATILE`, compatible avec `respeaker_node`
  dans l'inventaire.
- Commande : publications `Twist` ; `enable_stamped_cmd_vel: false` est bien
  présent pour le contrôleur et les comportements Nav2. Le topic matériel
  `/botwheel_explorer/cmd_vel` utilise au contraire `TwistStamped` : Fosa ne
  publie pas directement dessus.
- Couleur : conversion par `CvBridge` en `mono8` pour les tags et `bgr8` pour
  le JPEG. Les encodages effectivement reçus restent à mesurer sur le robot.
- Profondeur : `16UC1` et `mono16` sont interprétés en millimètres, `32FC1`
  en mètres. Cela correspond au convertisseur du simulateur ; l'inventaire
  physique ne donne pas les encodages réels.
- La projection de profondeur ne traite pas explicitement `is_bigendian`.
  Ce n'est pas un échec démontré sur le robot Orin, mais le contrat accepté est
  plus limité que tout message `Image` possible.

## 2. Arbitre : sens, types et contenu

| Topic | Sens pour Fosa | Type `parc_msgs/msg/…` | Résultat |
| --- | --- | --- | --- |
| `/parc/tag_order` | Réception | `TagOrder` | Détection automatique ; extraction de `tag_ids`, y compris `uint8[]` |
| `/parc/start` | Publication et écoute du départ externe | `StartSignal` | `method=0/1`, `heard` remplis |
| `/parc/tag_visit` | Publication | `TagVisit` | `tag_id`, `header`, `image` en `CompressedImage`, JPEG encodé localement |
| `/parc/finish` | Publication | `Finish` | `reason=0` configuré |
| `/parc/run_status` | Réception | `RunStatus` | États terminaux 3, 4, 5 reconnus |

Références : `caytu_course/caytu_course/arbitre.py`, `statut.py`,
`entrees_course.py:84-148,201-215` ; définitions dans `../parc_msgs/msg`.

- L'ordre possède un abonnement `RELIABLE/TRANSIENT_LOCAL` pour recevoir
  l'ordre antérieur au démarrage, plus un abonnement volatile. Les doublons
  d'ordre identiques sont ignorés.
- Les événements Fosa sont publiés `RELIABLE/TRANSIENT_LOCAL`. Les abonnements
  de l'arbitre local sont `RELIABLE/VOLATILE` : ils sont compatibles, mais
  ne récupèrent pas l'historique si leur connexion arrive après l'événement.
- `run_status` est reçu en `BEST_EFFORT/VOLATILE` : compatible, mais sans
  récupération immédiate du statut mémorisé. La publication à 2 Hz de l'arbitre
  compense normalement cette attente.
- `StartSignal.stamp` et `Finish.stamp` restent à zéro : les fonctions de
  remplissage ne reçoivent aucun instant. Le referee local ignore ces champs
  et utilise sa réception ; cela n'empêche pas ce referee de fonctionner.
- `TagVisit.header.stamp` reprend l'instant de l'image, alors que le commentaire
  officiel décrit l'instant de la visite. Une image mise en cache peut donc
  donner un instant antérieur à la visite.
- En cas d'échec d'encodage ou d'image absente, une visite peut être envoyée
  sans JPEG. Le contrat du type est satisfait, pas l'exigence de preuve visuelle.
- L'absence de `parc_msgs` produit des erreurs mais n'interdit pas la course.
  `PublicateurArbitre.pret()` existe sans être utilisé pour conditionner le départ.
  La solution ne valide pas non plus explicitement la réception du départ ou
  de chaque visite via `RunStatus.start_method/tags_reported`.

L'arbitre local écoute `/odom` par défaut. Notre
`simulation/simulation.sh:267-268` passe bien
`odom_topic:=/botwheel_explorer/odom`. Le paramètre du referee installé sur le
robot doit être contrôlé séparément.

## 3. Topics produits par la solution

Leur absence dans l'inventaire matériel initial est normale.

| Topic | Type | Producteur → consommateur |
| --- | --- | --- |
| `/fosa/scan_camera1`, `/fosa/scan_camera2` | `sensor_msgs/LaserScan` | `depth_scan` → costmaps Nav2 |
| `/perception/tags` | `caytu_interfaces/TagDetectionArray` | Deux détecteurs → course |
| `/perception/tags/image` | `sensor_msgs/Image` | Deux détecteurs → cache des preuves de course |
| `/fosa/course/depart` | `std_msgs/String` | Commande ou Vosk → course |
| `/fosa/course/etat` | `std_msgs/String` (JSON) | Course → départ vocal et affichage |
| `/fosa/course/evenements` | `std_msgs/String` (JSON) | Course et départ vocal → suivi |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM → course et costmap globale |
| `/plan` | `nav_msgs/Path` | Nav2 → suivi technique |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | Nav2 → suivi technique |

Nav2 fournit aussi ses topics de costmaps, plans locaux, feedback et lifecycle.
Les services `/global_costmap/clear_entirely_global_costmap` et
`/local_costmap/clear_entirely_local_costmap`, ainsi que l'action
`/navigate_to_pose`, sont fournis par la pile lancée par Fosa : ils ne sont pas
attendus du bringup matériel PARC.

## 4. Interfaces PARC disponibles mais non nécessaires à la course

- `/scan` : laser brut, utilisé par le filtre des organisateurs et certains
  outils ; la course utilise `/scan_filtered`.
- `/camera1/scan`, `/camera2/scan`, `/scan_merged` : présents mais non abonnés
  par la course. Fosa produit ses propres scans à partir des images de profondeur
  pour sélectionner les obstacles selon leur hauteur. Ce n'est pas une omission
  de topic ; la qualité de cette projection dépend des TF.
- `/cameraN/cameraN/imu` : disponible, utilisé par l'outil de mesure de rotation,
  pas fusionné dans la localisation de course.
- `/audio` : diagnostic et source configurable du mode audio continu ;
  `/audio/channel0` à `/audio/channel5`, `/is_speeching`, `/sound_direction`,
  `/sound_localization` ne sont pas nécessaires au départ vocal configuré.
- `/joint_states` et `/robot_description` : exploités par l'infrastructure du
  robot ; la course consomme les TF résultantes.
- Topics RealSense `accel/*`, `gyro/*`, `*/metadata` et `extrinsics/*` :
  inventoriés sur le robot, non requis par le chemin de perception actuel.
- Commandes et visualisations de la pile matérielle :
  `/botwheel_explorer/cmd_vel`, `/stamped_vel`, `/auto_vel`, `/safe_vel`,
  `/goal`, `/nav_goal`, `/desired_traj`, `/collision_obstacles`, `/closest_gap`,
  `/gaps`, `/virt_gaps`, `/footprint` : pas des entrées nécessaires de Fosa.
- Accessoires et intégration : `/accessory_view/*`, `/battery_level`,
  `/network/*`, `/volume/*`, `/status_led`, `/webrtc/*`, `/ros_to_iot`,
  `/client_count`, `/connected_clients` : non consommés par la course.
- Suivi de personne : `/person_following_robot/*`, `/tracked_person/position` :
  non consommés par la course.
- Infrastructure : `/controller_manager/*`, `/dynamic_joint_states`,
  `/botwheel_explorer/transition_event`,
  `/joint_state_broadcaster/transition_event`, `/diagnostics`, `/parameter_events`,
  `/rosout` : gestion et diagnostic, sans dépendance métier directe.

Le simulateur reproduit seulement une partie de ces interfaces. En particulier,
il ne reproduit pas la couche matérielle `obstacle_avoidance`.

## 5. Écarts réel / simulation

| Élément | Réel configuré | Simulation configurée |
| --- | --- | --- |
| Repère robot | `base_link` | `base_footprint` |
| Horloge | Système, `use_sim_time=false` | `/clock`, `use_sim_time=true` |
| Domaine ROS par défaut | 0 | 42 |
| Entrées profondeur | Images de profondeur | Mêmes images, converties depuis Gazebo |
| Filtre laser Fosa | Désactivé | Désactivé, filtre fourni par PARC |
| Taille du carré noir du tag | 0,165 m | 0,12 m selon l'analyse de texture du profil |
| Commande | `/cmd_vel` → `obstacle_avoidance` | `/cmd_vel` → pont Gazebo |

`/sim/pose`, `/sim/*_collisions` et les profondeurs internes `/sim/cameraN/...`
ne sont pas des dépendances de la course. Le moniteur de simulation utilise
pose et contacts pour l'observation externe.

Un domaine ROS déjà exporté reste prioritaire dans `run.sh`. Le profil `robot`
ne supprime pas un `FOSA_CONFIG_OVERRIDE` hérité : vérifier l'environnement
effectif lors du passage de la simulation au robot.

## 6. Points à résoudre ou confirmer

### A. Chaîne TF réelle potentiellement discontinue

L'inventaire physique (`parc_ros2_topics_updated.md:2491-2522`) donne notamment :

```text
odom -> base_link
base_link -> ncamera1_link
camera1_link -> ncamera1_depth_frame
camera1_depth_frame -> ncamera1_depth_optical_frame
base_link -> nlaser
```

Aucune liaison `ncamera1_link -> camera1_link` n'y apparaît, et le même motif
existe pour d'autres repères caméra. C'est une anomalie de la description
disponible, pas une preuve que le graphe actuel du robot est cassé.

Il faut relever les `header.frame_id` des images couleur, profondeur et scans,
puis vérifier leur transformation vers `base_link`, et vers `map` après SLAM.
Les noms sont lus depuis les messages par Fosa ; renommer seulement
`base_frame` ne répare pas une chaîne TF manquante.

Conséquences possibles : détections sans pose robot/carte, absence de
`/fosa/scan_cameraN`, scan inutilisable par SLAM. La profondeur journalise
l'absence de TF ; le détecteur peut continuer à publier des détections sans
poses transformées.

### B. Format audio physique non établi

Les 16 kHz, PCM signé 16 bits, mono sont explicites dans le simulateur. Le
snapshot réel ne renseigne pas ce format, seulement le type `AudioData` et
environ 15,6 messages/s sur `/audio`. Ce débit de messages ne démontre pas
la fréquence d'échantillonnage. Confirmer également que `/speech_audio`
porte bien une phrase complète par message sur le robot actuel.

### C. Diagnostic existant incomplet

`outils/diagnostic_robot.sh` vérifie surtout la présence des noms. Il ne compare
pas systématiquement les types attendus et les QoS des capteurs, ni le nombre
de producteurs réels. Un topic peut être visible uniquement grâce à un abonné.
Il ne vérifie pas explicitement toutes les entrées de profondeur.
`mesures_robot.py` mesure les calibrations couleur mais ne signale pas toutes
les TF capteur manquantes comme des échecs. Les scans `/fosa/*` ne peuvent être
attendus avant le lancement de leurs producteurs.

### D. Worktrees Git et workspace ROS partagés

Les dossiers `fosa-solution`, `fosa-phase-finale`, `fosa-phase-reelle` sous le
même `src` contiennent plusieurs paquets de même nom. Les deux nouveaux
worktrees ont été créés pendant cette session. Une découverte colcon globale
rencontre ces doublons si ces chemins ne sont pas explicitement exclus.

En outre, `run.sh:5-6,82` déduit le workspace par `../..` : tous ces worktrees
sourcent le même espace `install/setup.bash`. La configuration peut venir de
cette branche alors que les exécutables installés viennent d'une autre.
L'isolation Git ne garantit donc pas l'isolation de l'exécution ROS.

Avant de compiler/lancer cette branche, prévoir un workspace propre avec ses
propres `build`, `install`, `log` et une sélection explicite des sources PARC
et Fosa. Ne pas reconstruire le workspace partagé de l'autre session.

## 7. Vérifications effectuées et suite sur le robot

Effectué ici :

- Comparaison automatique des chemins de topics de `fosa.yaml` avec les types
  de l'inventaire physique ; examen des producteurs internes pour les absences.
- Lecture des cinq définitions officielles locales et de leurs endpoints.
- Chargement réel de `caytu_config` et génération des paramètres Nav2 : succès.
  SLAM reçoit `/scan_filtered`, Nav2 `/botwheel_explorer/odom`, les costmaps
  les trois scans attendus et `base_link`. Les commandes restent non horodatées.
- Vérification des remappings, QoS Python et profils.
- Aucune session ROS lancée, aucune commande moteur envoyée. `/opt/ros` absent
  sur ce poste. Tests pytest ciblés tentés mais non exécutés : module `pytest`
  absent.

À collecter sur le robot, après sourcing du workspace correct :

```bash
ros2 topic list -t
ros2 topic info -v /cmd_vel
ros2 topic info -v /scan_filtered
ros2 topic info -v /botwheel_explorer/odom
ros2 topic info -v /speech_audio
ros2 topic info -v /parc/tag_order
ros2 topic info -v /parc/run_status
ros2 param get /parc_referee odom_topic
ros2 run tf2_tools view_frames
```

Faire également `topic info -v` pour les huit topics images/calibrations des
deux caméras ; relever encodages, `frame_id`, timestamps et débits. Utiliser
ensuite `tf2_echo base_link <frame_id_observé>` pour chaque capteur. Après
lancement de Fosa, contrôler `/map`, les deux `/fosa/scan_cameraN`, les poses
des tags et le JPEG reçu par l'arbitre lors d'une visite d'essai.
