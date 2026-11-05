# Premier essai AprilTags : IDs et images annotées

Objectif : reconnaître la famille 36h11 et dessiner les contours et IDs, sans
carte, sans TF, sans calibration et sans Gazebo. Les scènes sont générées par
OpenCV, puis traitées par le même moteur que le nœud ROS. Elles ne valident ni
la portée réelle, ni les distances, ni le comportement sous l'éclairage du robot.

## Sur le PC ROS 2 Jazzy

Depuis la racine de votre workspace `ros2_ws`, après installation des dépendances
du projet (notamment `python3-opencv`, `python3-numpy`, `python3-yaml`, `cv_bridge`) :

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to caytu_perception
source install/setup.bash
python3 -c 'import cv2; print(cv2.__version__); print(hasattr(cv2, "aruco"))'
ros2 run caytu_perception essai_tags --sortie /tmp/fosa-tags-premier-essai
```

La dernière commande refuse un dossier déjà existant : changer le nom pour un
nouvel essai. Ouvrir `annotees/ensemble.png` dans ce dossier. Les quatre IDs
attendus sont 1, 4, 7 et 9. Le dossier contient aussi les images sources et
`rapport.json` (version OpenCV, IDs attendus et trouvés). Un échec de détection
sur les scènes générées donne un code de sortie non nul.

Le module `cv2.aruco` doit être présent. Éviter de remplacer l'OpenCV système
utilisé par `cv_bridge` avec une installation pip globale.

## Sans ROS, sur ce dépôt

Depuis la racine du dépôt `fosa-solution`, avec Python, OpenCV contenant aruco,
NumPy et PyYAML disponibles :

```bash
PYTHONPATH=caytu_config:caytu_perception python3 -m caytu_perception.essai_tags --sortie /tmp/fosa-tags-hors-ros
```

Pour de vraies photos, ajouter `--images photo1.png photo2.jpg` ; les IDs attendus
ne sont alors pas connus automatiquement. Vérifier les annotations visuellement.
Les scènes générées comprennent une image vide, chaque tag de face, une rotation,
une perspective avec flou et une image réunissant les quatre tags.

## Avec le flux de la caméra du robot

Confirmer le topic d'image dans le graphe ROS courant. Exemple basé sur
l'inventaire fourni de la caméra 1 :

```bash
ros2 run caytu_perception tag_detector --ros-args \
  -p use_sim_time:=false \
  -p publish_map_pose:=false \
  -p image_topic:=/camera1/camera1/color/image_raw \
  -p annotated_directory:=/tmp/fosa-tags-camera
```

Un sous-dossier `session_*` est créé et annoncé dans les logs. Par défaut, au
maximum une image par seconde est enregistrée, avec une limite de 100 images
par lancement. Les images sans tag sont aussi enregistrées. Les logs indiquent
les IDs détectés. Les paramètres `save_interval_sec` et `max_saved_images`
permettent d'ajuster cette collecte ; leurs valeurs par défaut viennent de
`caytu_config/config/fosa.yaml`. Le mode IDs ne publie pas de pose fictive sur
`/perception/tags`. Il n'a pas besoin de recevoir `camera_info`.

Dans Gazebo, utiliser le vrai topic image de la simulation et
`use_sim_time:=true`. Pour un rosbag, rejouer avec `--clock` et utiliser également
`use_sim_time:=true`. Les abonnements image et calibration utilisent le QoS
capteur best-effort pour accepter les publications de capteurs best-effort
comme reliable.

## Pour la suite

Le mode historique reste `publish_map_pose:=true` : il exige une calibration et
une transformation vers `map` à la date de l'image pour publier des poses.
Les annotations restent disponibles même si la calibration ou la TF manque.
Le paramètre `size_m: 0.18` existant a été conservé pour la simulation : il
n'intervient pas dans la lecture des IDs. Avant toute estimation de distance
sur le robot réel, vérifier la dimension annoncée de 0,165 m selon les coins
du marqueur (pas les marges de la feuille) et adapter le profil réel.

La détection accepte tous les IDs de la famille ; `test_ids` définit seulement
les tags du jeu d'essai. Aucun réglage de mouvement du robot n'est changé.

Référence : [détection et annotation OpenCV](https://docs.opencv.org/4.10.0/d5/dae/tutorial_aruco_detection.html).

## Vérification du code

Lancer séparément des anciens tests de perception qui remplacent OpenCV par un mock :

```bash
PYTHONPATH=caytu_config:caytu_perception python3 -m pytest caytu_perception/test/test_tags_image.py
PYTHONPATH=caytu_config python3 -m pytest caytu_config/test/test_config.py
```

Le test du traitement ROS remplace les interfaces ROS, mais utilise le vrai
OpenCV. Ce n'est pas une validation du transport ROS : réaliser le lancement
ci-dessus sur le PC Jazzy avant le créneau robot.
