# Interfaces PARC — intégration du 6 novembre 2026

Références : inventaire matériel du 02/11 dans
`docs/parc-phase-finale-docs/parc_ros2_topics_updated.md`, consignes arbitre
`informations-parc.txt`, et sources officielles voisines à `4ac1876`.
La présence déclarée d’une interface ne prouve pas sa disponibilité au créneau.

## Entrées matérielles

| Topic | Type ROS | Usage |
|---|---|---|
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | Nav2, garde et SLAM si activé |
| `/botwheel_explorer/odom` | `nav_msgs/msg/Odometry` | Navigation et suivi |
| `/cameraN/cameraN/color/image_raw` | `sensor_msgs/msg/Image` | Tags et preuve JPEG |
| `/cameraN/cameraN/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration |
| `/cameraN/cameraN/depth/image_rect_raw` | `sensor_msgs/msg/Image` | Obstacles hauts |
| `/cameraN/cameraN/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Projection profondeur |
| `/speech_audio` | `audio_common_msgs/msg/AudioData` | Mot de départ |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Repères dynamiques et fixes |

`N=1,2`. Annonce du 06/11 : camera1 basse et camera2 avant sur le robot,
disposition inverse dans Gazebo. Les noms de topics ne sont pas renommés.
Les capteurs sont reçus avec `qos_profile_sensor_data` ; l’audio en reliable/volatile.

## Commande et navigation

`/fosa/cmd_vel` reçoit les commandes **Twist** de Nav2, de l’alignement et du
guidage manuel. La garde Fosa publie sur `/cmd_vel`, consommé par les protections
organisateurs. Fosa ne publie pas directement sur `/botwheel_explorer/cmd_vel`
(`TwistStamped`) et conserve `enable_stamped_cmd_vel: false` dans Nav2.

Robot : domaine 0, `base_link`. Simulation : domaine 42, `base_footprint`.
Avec carte : `map -> odom` vient de SLAM. Sans carte : repère global `odom`,
pas de SLAM ; `/global_costmap/costmap` remplace `/map` pour la recherche.
La garde utilise les trois scans activés, notamment `/fosa/scan_camera1` et
`/fosa/scan_camera2`, générés par la perception Fosa.

## Arbitre officiel

| Topic | Sens Fosa | Type `parc_msgs/msg/…` |
|---|---|---|
| `/parc/tag_order` | réception | `TagOrder`, champ `tag_ids` |
| `/parc/start` | publication et écoute externe | `StartSignal`, méthode 0/1, mot entendu, instant |
| `/parc/tag_visit` | publication | `TagVisit`, identifiant, en-tête caméra, `CompressedImage` JPEG |
| `/parc/finish` | publication | `Finish`, raison et instant |
| `/parc/run_status` | réception | `RunStatus`, états et signalements reçus |

L’ordre est reçu en transient-local pour récupérer un ordre déjà publié. Les
événements publiés sont reliable/transient-local, compatibles avec les abonnements
reliable/volatile du referee. Le statut officiel est publié à 2 Hz ; la garde
reçoit aussi son dernier état mémorisé.

La visite possède l’instant du signalement et le repère de la caméra ; l’image
compressée conserve son instant d’acquisition. Aucun signalement sans JPEG.
Le maintien est de 5 s réelles, même si le dernier tag déclenche `FINISHED`.
`TIMED_OUT` et `TERMINATED` interrompent le maintien. Les états terminaux annulent
les actions ; le budget officiel est en temps monotone, même dans Gazebo.

**`tags_reported` désigne des signalements, pas des visites acceptées par les
juges.** Le referee conserve séparément le verdict et les contrôles d’image/maintien.

## Vérifier au créneau

`./run.sh diagnostic-robot` produit un rapport des types, QoS, TF, caméras et
audio. Avant lancement, l’absence des scans internes `/fosa/scan_cameraN` est
normale : ils apparaissent avec la perception Fosa.

Vérifier l’encodage profondeur (16 bits millimètres ou 32 bits mètres), la
calibration, la taille noire des tags, l’empreinte et le centre du robot.
Le format PCM du ReSpeaker n’est pas précisé par l’inventaire ; confirmer
fréquence/canaux avant d’utiliser la voix. L’essai doit confirmer les échanges
réels avec le referee, le maintien et l’arrêt demandé par un juge.
