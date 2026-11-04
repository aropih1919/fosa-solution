# Guide détaillé — AprilTags, SLAM et carte sémantique

Ce guide explique ce que fait actuellement la solution AprilTag, comment
l'installer et la lancer progressivement, et quels réglages vérifier sur le
robot. Il distingue la carte géométrique de `slam_toolbox` de la carte
sémantique Fosa : ce ne sont pas la même chose.

## 1. Résumé en une phrase

Le détecteur trouve un ID dans une image, estime la pose du tag par rapport à la
caméra, puis, si la calibration et les TF sont disponibles, transforme cette
pose dans le repère `map` et la publie sur `/perception/tags`. La carte
sémantique associe ensuite cet ID à un lieu connu.

```text
Image + CameraInfo                         SLAM / robot TF
        │                                       │
        ▼                                       ▼
 OpenCV repère le tag et estime caméra→tag   map→caméra
        └──────────────────┬────────────────────┘
                           ▼
             pose du tag dans le repère map
                           ▼
              /perception/tags (ROS 2)
                           ▼
        carte sémantique : ID → lieu → approche
```

## 2. Ce que la solution fait — et ne fait pas

### Ce qu'elle fait

1. `caytu_perception` reçoit l'image et les paramètres intrinsèques de la
   caméra (`CameraInfo`).
2. OpenCV détecte les tags de la famille configurée et donne leurs coins dans
   l'image.
3. `solvePnP` utilise les coins, la calibration et la taille physique déclarée
   du tag pour estimer sa position et son orientation par rapport à la caméra.
4. TF2 récupère la transformation entre le repère de l'image et `map`, à
   l'instant de l'image. Le détecteur compose les transformations et publie
   l'ID, la pose du tag dans `map` et la distance.
5. `caytu_semantic_map` mémorise la dernière pose reçue pour chaque ID. Il
   associe l'ID au lieu déclaré dans `locations.yaml` et peut calculer la pose
   d'approche devant ce tag.

### Ce qu'elle ne fait pas actuellement

- Elle ne transmet pas des contraintes de balises à `slam_toolbox` et ne
  modifie pas directement la carte d'occupation. Le SLAM continue à construire
  sa carte avec ses capteurs et sa configuration habituels.
- Elle ne connaît pas magiquement l'emplacement absolu d'un tag. La pose dans
  `map` est calculée à partir de la pose TF du robot/caméra dans `map` et de la
  mesure caméra→tag.
- Les scènes de l'essai hors ROS ne valident ni la précision métrique ni la
  chaîne TF réelle.

Pour que le SLAM utilise les tags comme repères/corrections de sa carte, il
faudrait développer une intégration SLAM supplémentaire. Le topic actuel est un
flux de perception consommé par la carte sémantique, pas une entrée de
`slam_toolbox`.

## 3. Fichiers et responsabilités

| Fichier | Rôle |
|---|---|
| `caytu_perception/caytu_perception/tags_image.py` | Détection OpenCV réutilisable, annotation et scènes de test, sans ROS. |
| `caytu_perception/caytu_perception/detecteur_tags.py` | Nœud ROS : image, calibration, estimation caméra→tag, TF vers `map`, publication. |
| `caytu_interfaces/msg/TagDetection.msg` | Contrat d'une détection : `tag_id`, `pose_map`, `distance_m`, `confidence`. |
| `caytu_interfaces/msg/TagDetectionArray.msg` | Tableau publié sur `/perception/tags`. |
| `caytu_semantic_map/caytu_semantic_map/noeud_carte_semantique.py` | Consomme les détections, mémorise les poses, publie les lieux et expose les services sémantiques. |
| `caytu_semantic_map/caytu_semantic_map/registre.py` | Associe ID↔lieu et calcule une pose d'approche. |
| `caytu_config/config/fosa.yaml` | Famille, taille, portée, topics, mode publication et réglages d'enregistrement. |
| `caytu_config/config/locations.yaml` | Noms des lieux, IDs de tags associés et distance d'approche. |

## 4. Installation

### 4.1 Poste complet du projet

Le projet cible Ubuntu 24.04 et ROS 2 Jazzy. Depuis le dépôt :

```bash
./install.sh
```

L'installateur prépare l'environnement du projet et les dépendances. Les
modèles IA optionnels peuvent être installés ensuite avec `./install.sh
--models` ; ils ne sont pas nécessaires au seul détecteur AprilTag.

### 4.2 Dépendances spécifiques AprilTag

Le détecteur utilise Python, ROS 2 (`rclpy`, `sensor_msgs`, TF2), `cv_bridge`,
NumPy et OpenCV avec `cv2.aruco`. Éviter d'installer un OpenCV pip qui
remplacerait l'OpenCV système utilisé par `cv_bridge`.

Vérifier OpenCV dans l'environnement ROS :

```bash
source /opt/ros/jazzy/setup.bash
python3 -c 'import cv2; print(cv2.__version__); print(hasattr(cv2, "aruco"))'
```

Le second résultat doit être `True`.

### 4.3 Compiler les paquets nécessaires

Depuis la racine du workspace (ici `~/ros2_ws`, pas depuis le sous-dossier du
dépôt) :

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to caytu_perception caytu_semantic_map
source install/setup.bash
```

`--packages-up-to` compile aussi les dépendances des deux paquets demandés.
Après chaque changement de code ou d'interface, reconstruire puis sourcer le
workspace dans chaque nouveau terminal.

## 5. Démarrer progressivement

Ne lancer d'abord que le test d'images, puis le flux caméra sans poses, et enfin
la chaîne complète. Cela sépare les problèmes de détection des problèmes de
calibration, TF ou SLAM.

### Étape A — Test d'images sans ROS ni robot

Depuis `~/ros2_ws`, après le build et le `source install/setup.bash` :

```bash
ros2 run caytu_perception essai_tags --sortie /tmp/fosa-tags-test-01
```

Le dossier de sortie doit être nouveau. Examiner `annotees/ensemble.png` et
`rapport.json`. Le jeu de test utilise les IDs configurés dans
`fosa.yaml` (`test_ids`, par défaut 1, 4, 7 et 9). Ce réglage ne limite pas les
IDs détectables sur le vrai flux.

Pour des photos :

```bash
ros2 run caytu_perception essai_tags --sortie /tmp/fosa-tags-photos-01 \
  --images /chemin/photo1.png /chemin/photo2.jpg
```

### Étape B — Vérifier la caméra, sans calcul de pose

Confirmer les topics disponibles :

```bash
ros2 topic list
ros2 topic info /top_camera_color/image_raw
ros2 topic info /top_camera_color/camera_info
```

Si les noms réels sont différents, utiliser ceux du robot. Lancer d'abord le
nœud du pilote caméra, puis, dans un autre terminal :

```bash
ros2 run caytu_perception tag_detector --ros-args \
  -p publish_map_pose:=false \
  -p image_topic:=/top_camera_color/image_raw \
  -p camera_info_topic:=/top_camera_color/camera_info \
  -p annotated_directory:=/tmp/fosa-tags-camera
```

Dans ce mode, les IDs apparaissent dans les logs et les images annotées sont
écrites dans un sous-dossier `session_*`. Il n'y a **pas** de publication de
`/perception/tags` et `CameraInfo`/TF ne sont pas nécessaires pour obtenir les
IDs. Ajuster le nom des topics selon le matériel.

### Étape C — Lancer la solution avec SLAM/localisation et TF

Le SLAM ou le système de localisation doit être actif, tout comme le pilote de
caméra et la publication TF du robot. Il faut notamment une chaîne cohérente
entre `map` et le `frame_id` de l'image. Vérifier :

```bash
ros2 topic echo --once /top_camera_color/camera_info
ros2 run tf2_ros tf2_echo map <frame_id_de_l_image>
```

Remplacer `<frame_id_de_l_image>` par la valeur réelle du `header.frame_id` de
l'image. Si `tf2_echo` ne trouve pas la transformation, le détecteur ne peut
pas exprimer le tag dans `map`.

Lancer le détecteur en mode pose :

```bash
ros2 run caytu_perception tag_detector --ros-args \
  -p publish_map_pose:=true \
  -p image_topic:=/top_camera_color/image_raw \
  -p camera_info_topic:=/top_camera_color/camera_info
```

Dans un autre terminal, vérifier la sortie :

```bash
ros2 topic info /perception/tags
ros2 topic echo /perception/tags
```

Une détection utile doit contenir le bon `tag_id`, un `pose_map.header.frame_id`
égal à `map`, une position plausible et une distance plausible. Si aucune
détection n'apparaît, vérifier successivement caméra, IDs visibles, calibration,
taille physique configurée, puis TF à l'horodatage des images.

### Étape D — Ajouter la carte sémantique

Avec le même environnement ROS sourcé :

```bash
ros2 run caytu_semantic_map semantic_map_node
```

Le nœud s'abonne à `/perception/tags`, publie `/semantic_map/locations` et
fournit les services déclarés dans `locations.yaml`. Le détecteur et ce nœud
peuvent tourner séparément ; ils doivent toutefois utiliser la même
configuration et le même topic.

Le lancement `./run.sh navigation` est le point d'entrée de navigation Fosa,
mais le dépôt ne fournit pas actuellement un lancement dédié qui démarre
ensemble le détecteur AprilTag et la carte sémantique. Pour un essai maîtrisé,
lancer les composants explicitement dans des terminaux distincts comme ci-dessus.

## 6. Où déclarer les tags et les lieux

Dans `caytu_config/config/locations.yaml`, chaque lieu peut associer un ou
plusieurs IDs :

```yaml
- name: accueil
  aliases: [reception, information]
  tag_ids: [10]
  description: "Point d'accueil des visiteurs."
  category: service
  approach_offset_m: 0.8
```

Remplacer les IDs d'exemple par les IDs réellement imprimés et installés sur le
site. L'association permet de répondre « tag 10 = accueil ». `approach_offset_m`
indique à quelle distance du tag proposer la pose du robot. Une pose
`approach_pose` fixe, si configurée, est utilisée à la place de la pose
d'approche calculée depuis le tag.

Cette configuration est une association sémantique, pas un relevé fixe des
coordonnées du tag dans la carte. Les poses observées sont reçues dynamiquement
par `/perception/tags`.

## 7. Réglages à faire à partir de mesures

Ne pas régler les paramètres au hasard. Mesurer et noter d'abord le matériel,
les topics et les erreurs observées.

| Réglage | Où | Comment le choisir |
|---|---|---|
| `family` | `fosa.yaml → perception.apriltag.family` | Doit correspondre exactement à la famille imprimée, par exemple `36h11`. |
| `size_m` | `fosa.yaml → perception.apriltag.size_m` | Mesurer le côté du motif AprilTag entre les limites externes du carré noir, en mètres. Ne pas mesurer le papier blanc autour. La valeur actuelle `0.18` est un réglage de simulation ; confirmer la dimension réelle avant d'utiliser des distances. |
| `max_distance_m` | `fosa.yaml → perception.apriltag.max_distance_m` | Fixer une portée maximale correspondant à la distance où le tag reste détectable de façon fiable. Comparer plusieurs distances mesurées au sol. |
| `top_color_topic`, `top_info_topic` | `fosa.yaml → perception` ou paramètres de lancement | Prendre les noms réels du graphe ROS. L'image et son `CameraInfo` doivent provenir de la même caméra et de la même résolution/crop. |
| `publish_map_pose` | `fosa.yaml → perception.apriltag` ou paramètre ROS | `false` pour identifier/annoter seulement ; `true` seulement quand la calibration et les TF sont prêtes. |
| `approach_offset_m` | `locations.yaml` | Mesurer le dégagement nécessaire devant le tag pour que le robot s'arrête sans toucher le support ni un obstacle. |
| `annotated_directory`, `save_interval_sec`, `max_saved_images` | `fosa.yaml → perception.apriltag` ou paramètres ROS | Activer temporairement pour recueillir des images ; garder une fréquence et une limite adaptées à l'espace disque. |

### Ordre conseillé pour régler les mesures

1. Confirmer la taille physique du tag et sa famille.
2. Vérifier que le flux d'images affiche nettement le tag à différentes
   distances, angles et conditions d'éclairage.
3. Régler `size_m` avec la mesure du tag réel, puis comparer la distance publiée
   à une mesure au ruban à plusieurs endroits.
4. Vérifier que la position dans `map` est stable quand le robot est immobile,
   puis quand il bouge lentement. Une pose qui saute indique souvent un souci de
   calibration, de TF/horodatage ou de localisation.
5. Régler `max_distance_m` selon la portée effectivement fiable, pas selon la
   portée où un tag est détecté une fois.
6. Enfin, choisir les IDs et les offsets dans `locations.yaml`, puis vérifier
   physiquement les poses d'approche avant toute navigation vers ces lieux.

Une erreur de taille du tag fausse principalement l'échelle de la pose estimée
(notamment la distance). Une mauvaise transformation TF fausse la pose globale,
même si l'ID est détecté correctement.

## 8. Calibration caméra et référence des tags : attention à la distinction

Le détecteur ne calcule une pose métrique fiable que s'il reçoit les valeurs
`CameraInfo` correctes pour la caméra utilisée. Le pilote caméra publie
normalement ce message. Vérifier sa matrice `k`, ses coefficients `d`, sa
résolution et son `frame_id` ; ne pas substituer un topic d'une autre caméra.

Le programme `tag_calibrator` écoute les poses déjà publiées sur
`/perception/tags` et enregistre une moyenne des poses observées dans le fichier
indiqué par `calibration_file`. **Il ne calibre pas les paramètres optiques de
la caméra et le détecteur actuel ne relit pas ce fichier pour calculer ses
poses.** Il sert à produire une référence séparée ; il ne faut pas le confondre
avec la calibration nécessaire à `solvePnP` ni avec un module de correction
de `slam_toolbox`.

## 9. Temps simulation et rosbag

- Robot réel : utiliser le temps réel (`use_sim_time:=false`) et les vrais
  topics caméra/TF.
- Gazebo : lancer avec le temps simulation, et vérifier que tous les nœuds
  concernés utilisent la même horloge.
- Rosbag : démarrer le lecteur avec `--clock`, puis utiliser `use_sim_time:=true`
  pour les nœuds qui traitent ces images. Les TF doivent être disponibles aux
  horodatages rejoués.

Les souscriptions image et `CameraInfo` utilisent le QoS capteur. Si aucun
message n'arrive, contrôler le nom du topic et les profils avec `ros2 topic
info -v <topic>`.

## 10. Diagnostic rapide

| Symptôme | Contrôles prioritaires |
|---|---|
| Aucun ID dans les logs | Topic image, encodage, visibilité, famille, `cv2.aruco`, netteté et éclairage. |
| ID visible mais aucune pose publiée | `publish_map_pose`, arrivée de `CameraInfo`, topic de sortie et distance maximale. |
| Message publié mais position incohérente | Valeur `size_m`, calibration de la bonne caméra, `frame_id`, TF `map→caméra`, horodatage et cohérence de l'horloge. |
| La carte sémantique ne reconnaît pas le lieu | Vérifier que `tag_ids` dans `locations.yaml` contient l'ID reçu et que les deux nœuds utilisent `/perception/tags`. |
| Le SLAM ne change pas grâce au tag | Comportement attendu avec le code actuel : les tags ne sont pas injectés dans `slam_toolbox`. |

Commandes utiles :

```bash
ros2 node list
ros2 topic list
ros2 topic hz /top_camera_color/image_raw
ros2 topic echo --once /top_camera_color/camera_info
ros2 topic echo /perception/tags
ros2 run tf2_ros tf2_echo map <frame_id_de_l_image>
```

## 11. Tests automatisés

Depuis la racine du workspace :

```bash
colcon test --packages-select caytu_config caytu_interfaces caytu_semantic_map caytu_perception
colcon test-result --verbose
```

Le test OpenCV des scènes AprilTag peut aussi être lancé séparément selon
`docs/APRILTAGS_PREMIER_ESSAI.md`. Les tests ne remplacent pas la vérification
des vrais topics caméra et TF sur le robot ou en simulation.

## 12. Résumé avant un essai sur robot

- [ ] OpenCV contient `cv2.aruco` dans l'environnement ROS utilisé.
- [ ] Le tag réel est de la famille configurée et sa taille a été mesurée.
- [ ] Le topic image et son `CameraInfo` correspondent à la même caméra.
- [ ] Le `frame_id` de l'image est connu et TF le relie à `map`.
- [ ] La localisation/SLAM est stable avant d'activer la publication des poses.
- [ ] `publish_map_pose:=true` est utilisé seulement pour le mode pose.
- [ ] `/perception/tags` publie des IDs et des poses plausibles dans `map`.
- [ ] Les IDs de `locations.yaml` correspondent aux tags réellement installés.
- [ ] La pose d'approche a été vérifiée sans envoyer le robot vers un lieu non validé.
