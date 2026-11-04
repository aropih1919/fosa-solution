# Obstacles mobiles — fonctionnement, réglages et validation

Ce document décrit le comportement implémenté à ce stade pour les personnes et
obstacles mobiles. Le tableau convenu est un périmètre de tests, pas une
garantie de couverture de tous les cas du terrain.

## 1. Comportement actuel

- Les scans LiDAR et les scans dérivés des caméras alimentent les costmaps Nav2.
  Le contrôleur ralentit/régule sa vitesse près des obstacles (RPP) et peut
  s'arrêter si sa trajectoire de collision est imminente. Nav2 replanifie à
  2 Hz.
- Cette régulation est celle du contrôleur RPP sélectionné par défaut ; tout
  essai avec DWB ou MPPI doit refaire les tests de ralentissement et d'arrêt.
- Les observations mobiles ne sont plus effacées par des nettoyages automatiques
  de costmap après un échec de navigation. Les couches de capteurs doivent
  confirmer la zone libre par leurs rayons de dégagement.
- Le BT recalcule le chemin à 2 Hz pendant le suivi normal. Si Nav2 échoue, la
  tâche reste active, observe pendant la durée configurée puis renvoie le
  **même but**. Les manœuvres sans progrès sont séparées par une période
  d'observation.
- Les reculs automatiques `BackUp` et les rotations de récupération non
  surveillées ont été retirés. Un recul court contrôlé existe, mais il est
  **désactivé par défaut** : sans couverture arrière mesurée et stable, le
  comportement sûr est de rester immobile.
- La détection personne ne salue pas et ne commande aucun suivi physique dans
  ce périmètre. `mission.greeting_enabled` est désactivé par défaut. Les demandes
  explicites par dialogue restent indépendantes de la détection de personne.

Les scènes et règles à tester sont décrites plus loin ; le comportement réel
dépend de la visibilité, du champ des capteurs, de leurs délais et de la
configuration Nav2.

## 2. Flux de perception et suivi du mouvement

`obstacle_classifier` combine maintenant une paire image/profondeur proche dans
le temps :

1. Il apparie les messages par leurs `header.stamp` dans la tolérance
   `perception.tracking.image_cloud_sync_slop_sec`.
2. Les paires reçues trop tard sont rejetées selon
   `perception.tracking.max_observation_age_sec`. Un horodatage nul ou une TF
   vers `base_footprint`/`odom` manquante fait rejeter l'observation.
3. Les points du nuage sont transformés à l'horodatage du nuage vers `odom`,
   `base_footprint` et, si YOLO est actif, le repère optique de l'image.
4. Les boîtes YOLO sont projetées sur les pixels avec `CameraInfo` (matrice,
   distorsion, dimensions) et associées aux points 3D effectivement présents
   dans la boîte. Une simple proximité angulaire ne suffit plus.
5. Les catégories publiées dans `DetectedEntity.class_name` sont uniquement
   `personne` et `obstacle`. Une classe YOLO autre que la classe personne est
   regroupée sous `obstacle` ; la géométrie du nuage reste disponible si YOLO
   n'est pas chargé.
6. Les observations successives sont associées dans `odom`. La vitesse est
   calculée avec les horodatages capteur puis tournée dans le repère du robot
   pour le champ vectoriel du message. Le déplacement du robot ne doit donc plus
   transformer à lui seul un objet fixe en objet mobile.

Les identifiants `track_id` sont locaux à l'exécution du nœud et ne sont pas des
identifiants persistants.

### Fichiers principaux

- `caytu_perception/caytu_perception/classificateur_obstacles.py` : association
  temporelle, projection image/profondeur et suivi en `odom`.
- `caytu_perception/caytu_perception/essai_obstacles.py` : validation YOLO
  hors ROS sur une image ou vidéo avec export annoté.
- `caytu_nav_bringup/config/nav2_params.yaml` et
  `caytu_nav_bringup/behavior_trees/navigate_bounded_recovery.xml` : costmaps,
  replanification et retour des échecs à la tâche sans recul aveugle.
- `caytu_nav_solution/caytu_nav_solution/task_solution.py` : conservation du
  but, contrôle de progrès et recul explicitement gardé.
- `caytu_config/config/fosa.yaml` : réglages modifiables et seuils.

## 3. Préparer le PC de test

Sur Ubuntu 24.04 avec ROS 2 Jazzy installé, depuis le dépôt :

```bash
./install.sh
./install.sh --models
```

Le modèle `yolo11n.onnx` est installé sous `caytu_config/config/models/`, un
répertoire ignoré par Git. Il n'est pas présent dans ce dépôt : vérifier sa
présence après installation avant de conclure sur les performances YOLO.
L'absence du modèle laisse la classification géométrique active, mais ne valide
pas la reconnaissance visuelle.

Compiler les paquets concernés depuis la racine du workspace :

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to \
  caytu_config caytu_interfaces caytu_perception caytu_nav_solution caytu_nav_bringup
source install/setup.bash
```

Vérifier les dépendances et le modèle :

```bash
python3 -c 'import cv2; print(cv2.__version__, hasattr(cv2, "dnn"))'
test -s ~/ros2_ws/src/fosa-solution/caytu_config/config/models/yolo11n.onnx
```

Le nœud charge les modèles ONNX avec OpenCV DNN. Ne pas remplacer OpenCV système
par un paquet pip incompatible avec `cv_bridge`.

## 4. Contrôler les entrées avant la reconnaissance

Avec le pilote caméra, la localisation et les TF déjà lancés :

```bash
ros2 topic list
ros2 topic hz /top_camera_color/image_raw
ros2 topic hz /top_camera_depth/points
ros2 topic echo --once /top_camera_color/camera_info
ros2 topic info -v /top_camera_color/image_raw
ros2 topic info -v /top_camera_depth/points
```

Vérifier que :

- image, nuage de profondeur et `CameraInfo` proviennent du même montage de
  caméra et de la même résolution/coupe d'image ;
- les horodatages sont non nuls, sur la même horloge et espacés de moins que la
  tolérance de synchronisation ;
- `CameraInfo.width/height`, `k`, `d` et `header.frame_id` sont cohérents avec
  le flux image ;
- le TF entre le repère du nuage et le repère optique de l'image est publié à
  l'horodatage des messages, ainsi que les transformations vers `base_footprint`
  et `odom`.

Pour inspecter les repères, remplacer les exemples par les `frame_id` réels :

```bash
ros2 topic echo --once /top_camera_color/image_raw
ros2 topic echo --once /top_camera_depth/points
ros2 run tf2_ros tf2_echo base_footprint <frame_nuage_profondeur>
ros2 run tf2_ros tf2_echo <frame_image_optique> <frame_nuage_profondeur>
ros2 run tf2_ros tf2_echo odom <frame_nuage_profondeur>
```

Une TF disponible uniquement à l'instant présent ne garantit pas qu'elle soit
disponible aux horodatages enregistrés dans un rosbag.

## 5. Valider YOLO sur des images ou vidéos annotées

Utiliser des exemples représentatifs : personne entière et partiellement
masquée, chaises, tables, objets bas, contre-jour, mouvement et distances utiles.
Créer un dossier de sortie neuf :

```bash
ros2 run caytu_perception essai_obstacles \
  --source /chemin/vers/scene.jpg \
  --sortie /tmp/fosa-obstacles/scene-annotee.jpg

ros2 run caytu_perception essai_obstacles \
  --source /chemin/vers/scene.mp4 \
  --sortie /tmp/fosa-obstacles/scene-annotee.mp4
```

Les boîtes affichent `personne` ou `obstacle` et la confiance. Une suppression
non maximale par catégorie retire les boîtes superposées selon
`perception.yolo.nms_iou_threshold` et limite le résultat à
`perception.yolo.max_detections`. Vérifier les faux positifs, les personnes
manquées et les classes hors périmètre qui seraient rangées comme `obstacle`.
Les seuils `perception.yolo.confidence` et `nms_iou_threshold` ne doivent être
ajustés qu'après comparaison d'annotations sur un lot représentatif.

## 6. Vérifier la fusion image/profondeur en ROS

Lancer le nœud sur les topics déclarés dans `fosa.yaml` :

```bash
ros2 run caytu_perception obstacle_classifier
```

Dans un autre terminal :

```bash
ros2 topic hz /perception/entities
ros2 topic echo /perception/entities
```

Pour chaque observation, contrôler `header.stamp`, `header.frame_id`,
`position`, `distance_m`, `bearing_rad`, `class_name`, `is_dynamic`, `track_id`
et `velocity`. Si les detections sont absentes, contrôler dans cet ordre :

1. arrivée et horodatages des trois topics ;
2. valeur de `image_cloud_sync_slop_sec` par rapport à l'écart mesuré ;
3. dimensions et repères de `CameraInfo` et de l'image ;
4. TF profondeur→image optique, profondeur→base et profondeur→odom aux stamps ;
5. qualité du nuage et nombre de points par cluster ;
6. modèle ONNX, sortie du modèle, seuil de confiance et correspondance pixel/3D.

Pour mesurer l'effet du mouvement du robot, enregistrer une scène avec un
obstacle fixe : robot immobile, puis translation/rotation lente du robot.
L'obstacle fixe doit garder un `track_id` dans la fenêtre de suivi et une
vitesse proche de zéro dans `odom`. Ensuite déplacer l'obstacle devant un robot
immobile : la vitesse doit dépasser `perception.person.dynamic_speed_mps` si le
mouvement est suffisamment grand et si l'association reste valide.

## 7. Vérifier Nav2 et la couverture arrière

Le lancement de navigation utilise les scans capteurs en direct. Vérifier au
minimum :

```bash
ros2 topic hz /scan
ros2 topic hz /top_camera_scan
ros2 topic hz /bottom_camera_scan
ros2 topic echo --once /scan
ros2 topic echo --once /local_costmap/costmap
ros2 topic echo --once /global_costmap/costmap
```

Dans RViz, vérifier qu'un obstacle mobile est marqué dans la costmap locale,
qu'il disparaît seulement quand un rayon voit la zone libre, et que Nav2
replanifie autour d'un passage disponible. La zone visible dans un seul sens ne
doit pas être prise pour une couverture arrière.

**Le recul est désactivé par défaut** (`navigation.obstacle_retreat_enabled: false`).
Avant de l'activer, confirmer sur le montage réel, avec le robot immobile, que :

- le LaserScan couvre complètement le secteur arrière
  `obstacle_rear_sector_half_angle_rad` ;
- le secteur est observé avec au moins `obstacle_rear_min_samples` rayons et
  aucun intervalle angulaire supérieur à
  `obstacle_rear_max_angular_gap_rad` ;
- le `range_max` dépasse le recul demandé plus la marge de sécurité ;
- le scan est frais, transformable dans `base_footprint` et continue à être
  publié durant le déplacement ;
- les distances de freinage du robot et le dégagement autour de son empreinte
  sont mesurés.

Si un seul point est incertain, garder le recul désactivé. Lorsqu'il est activé,
le recul est refusé si l'une de ces conditions manque ; il s'interrompt si un
rayon devient occupé/invalide, si le scan devient ancien, si l'odométrie dévie
ou si la durée maximale expire. Le nombre de tentatives physiques est borné.

## 8. Réglages centralisés

Les valeurs initiales ci-dessous sont des points de départ de test, pas des
mesures de sûreté certifiées.

| Réglage YAML | Valeur initiale | Méthode de réglage |
|---|---:|---|
| `perception.tracking.image_cloud_sync_slop_sec` | 0,08 s | Mesurer la différence des stamps image/nuage ; réduire autant que possible sans perdre les paires. |
| `perception.tracking.max_observation_age_sec` | 0,50 s | Mesurer la latence de bout en bout en rosbag et sur le PC ; rejeter les mesures au-delà du délai acceptable. |
| `perception.tracking.track_max_gap_sec` | 0,75 s | Régler selon la fréquence réelle et les occlusions courtes ; trop haut associe des cibles différentes. |
| `perception.person.tracker_match_m` | 0,45 m | Mesurer le déplacement apparent maximal entre messages après compensation odom. |
| `perception.person.dynamic_speed_mps` | 0,10 m/s | Comparer les vitesses observées sur robot immobile et cibles mobiles ; séparer bruit et mouvement utile. |
| `navigation.obstacle_min_progress_m` | 0,10 m | Définir le progrès minimum mesurable sur la distance au but, au-dessus du bruit odométrique. |
| `navigation.obstacle_retries_before_retreat` | 3 | Ajuster après mesure du délai de blocage/replanification ; ce seuil n'active pas seul le recul. |
| `navigation.obstacle_observation_sec` | 4 s | Mesurer le temps nécessaire à un passage pour se libérer et aux scans/costmaps pour se renouveler. |
| `navigation.obstacle_retreat_distance_m` | 0,20 m | Recul minimum utile, validé à très basse vitesse sur le robot réel ; le schéma limite la valeur à 0,25 m. |
| `navigation.obstacle_retreat_speed_mps` | 0,08 m/s | Vitesse lente compatible avec distance d'arrêt réellement mesurée ; le schéma limite la valeur à 0,10 m/s. |
| `navigation.obstacle_retreat_clearance_m` | 0,60 m | Marge ajoutée à la distance restante ; inclure l'empreinte et la distance de freinage mesurées. |
| `navigation.obstacle_retreat_timeout_sec` | 5 s | Borner la manœuvre selon distance/vitesse et temps de réaction observé ; le schéma impose un maximum de 8 s et une tentative de recul au plus par mission. |

Ne pas régler le seuil de mouvement sur une seule séquence. Noter au minimum la
latence, la fréquence de chaque capteur, le taux de paires valides, les erreurs
de position, la vitesse d'un obstacle fixe lorsque le robot bouge, et la
distance de freinage.

## 9. Campagne de tests progressive

Utiliser un mannequin ou une cible contrôlée, jamais une personne placée dans
la trajectoire d'essai. Démarrer avec une vitesse réduite et une zone dégagée.

| Étape | Scène | Critère de réussite |
|---:|---|---|
| 1 | Robot immobile, personne/cible mobile | Détection annotée correcte, profondeur associée, vitesse dans `odom` cohérente. |
| 2 | Robot mobile, obstacle fixe | ID suivi aussi longtemps que possible ; vitesse estimée proche de zéro, pas de faux obstacle en mouvement dû au robot. |
| 3 | Robot et cible mobiles | Vitesse/position suivent la cible avec horodatage plausible ; mesure périmée rejetée. |
| 4 | Obstacle hors trajet | Navigation continue avec marge, obstacle maintenu dans les capteurs. |
| 5 | Traversée devant le robot | Le contrôleur ralentit/s'arrête ; il ne reprend qu'après dégagement observé. |
| 6 | Trajet bloqué, passage latéral libre | Costmap et plan replanifiés, destination initiale inchangée. |
| 7 | Passage bloqué, arrière couvert puis occupé | Aucun recul ; robot immobile et observation maintenue. |
| 8 | Passage bloqué, arrière entièrement couvert et libre | Seulement en mode de test autorisé : un recul lent, court, surveillé et borné ; reprise du plan initial. |
| 9 | Obstacle entrant pendant le recul/contournement | Arrêt ou interruption rapide ; pas de collision. |
| 10 | Perte image, profondeur, TF, scan ou timestamps | Données invalides rejetées ; le robot ne recule pas sur la base d'une absence d'observation. |
| 11 | Blocage qui persiste sans progrès | Pas de boucle de reculs ; attente répétée, but conservé jusqu'à arrivée, annulation ou limite de temps. |

Enregistrer rosbag, sorties annotées, logs Nav2, fréquence capteur, distances
minimales et commandes de vitesse pour chaque essai. La validation des distances
d'arrêt, de la couverture arrière et de la reprise de mission reste à faire sur
le robot réel ; elle ne peut pas être remplacée par les tests unitaires.

## 10. Tests automatisés du dépôt

Depuis la racine du workspace ROS :

```bash
colcon test --packages-select caytu_config caytu_perception caytu_nav_solution caytu_nav_bringup
colcon test-result --verbose
```

Les tests ajoutés couvrent le parsing YOLO, l'association aux pixels de
profondeur, le suivi statique/mobile dans `odom`, le rejet d'un secteur arrière
incomplet/occupé et l'absence de récupération `BackUp`/nettoyage de costmap dans
le BT. Ils ne valident pas les drivers, modèles, TF enregistrés ou mesures du
robot réel.
