# Perception caméra : obstacles en hauteur invisibles au LiDAR

## Principe
`top_camera_depth/points` (15 Hz) → décimation (1 ligne/2, 1 colonne/4) → hauteur au-dessus du sol via TF
→ garde les points entre **0.30 m et 1.40 m** → pour chaque secteur d'azimut (0.015 rad), distance du
premier obstacle (quantile 30 %, ≥ 2 points) → `LaserScan` `/top_camera/obstacles_scan`.
Pas d'OpenCV : la profondeur suffit. `crowd_detector` (RGB) reste optionnel.

## Faits géométriques (calculés depuis le URDF, pas supposés)
- Caméra : (−0.0525, 0, **1.0126**) dans `base_footprint`, axes identiques (pitch 0, roll 0, yaw 0).
- Le LiDAR est à **z = 0.038 m** : il voit ras du sol.
- Robot : x ∈ [−0.29, 0.20], y ∈ [−0.22, 0.22], z ∈ [0.10, **1.349**] m.
- Plateau de `cafe_table` : z ≈ 0.735–0.775 m, 0.913 × 0.913 m, sur un pied de 4 cm → **invisible au LiDAR, dans la hauteur du corps du robot**.
- Le corps du robot ne masque pas la caméra (aucun sommet du mesh dans son champ).
- Le nuage est publié dans `top_camera_depth` (`gz_frame_id` du SDF), pas dans `..._optical_frame` comme l'écrit le plan : le node lit `header.frame_id`, donc peu importe, **mais vérifiez-le** (voir plus bas).

## Pourquoi 0.30 m et 1.40 m
- Sol : bruit ±0.10 m sur la distance ⇒ erreur de hauteur ≈ 0.10·sin(angle) ≤ 0.07 m (1σ) au premier point de sol visible (1.7 m). 0.30 m = plus de 3σ. Test : 0 faux positif sur 300 images simulées.
- 1.40 m = hauteur du robot (1.349) + marge. Au-dessus, ce n'est pas un obstacle.
- Trou assumé 0.10–0.30 m (pas de capteur) : rien de tel dans le monde fourni (pieds de tables/chaises touchent le sol → LiDAR).

## Angle mort proche (raison de `raytrace_min_range: 1.0` côté Nav2)
Le champ vertical est ±36.9°. Un objet dont le haut est à la hauteur h n'est visible qu'à partir de d = (1.0126 − h)/0.75 :

| Objet | h | Visible à partir de |
|---|---|---|
| Plateau de table | 0.775 m | 0.32 m de la caméra (= 7 cm avant le pare-chocs avant, à 0.25 m) |
| Assise de chaise | ~0.45 m | 0.75 m |
| Bas de la bande | 0.30 m | 0.95 m |

Dans cette zone la caméra est aveugle : Nav2 doit **se souvenir** de ce qu'elle a vu de plus loin.
D'où (1) une couche de coûts séparée, (2) aucun effacement caméra à moins de 1 m.

## Résultats de test (sans ROS, scène synthétique, bruit σ = 0.10 m) : `python3 -m pytest -q -s test`
| Test | Résultat |
|---|---|
| Sol seul, 300 images | 0 faux positif |
| Chant de plateau à 1.0 / 2.0 / 3.0 / 3.5 m | détecté 100 % ; biais +7 / +4 / +1 / −5 cm |
| Secteurs libres / hors champ | `+inf` / `NaN` comme prévu |
| Temps de calcul (38 400 points) | ~1 ms / image |

**Non testé ici** : les nodes ROS (`camera_obstacle_node`, watchdog) n'ont pu être que compilés (pas de ROS ni Gazebo dans mon environnement). Le bruit réel du simulateur et la vraie orientation des axes du nuage restent à confirmer.

## Checklist de validation en simulation (~15 min)
1. `ros2 topic echo /top_camera_depth/points --once --field header.frame_id` → attendu `top_camera_depth`.
2. `ros2 launch caytu_nav_bringup perception.launch.py` puis lire la ligne de stats (toutes les 5 s) : `z(base) p5` doit être ≈ 0 (le sol). Sinon le repère du nuage est mal interprété.
3. RViz : afficher `/top_camera/obstacles_scan` face à une `cafe_table` : un arc doit apparaître à la distance du plateau.
4. `ros2 topic hz /top_camera/obstacles_scan` ≈ 15 Hz ; `ros2 topic echo /camera_perception_ready` → `true`.
5. Robot devant une table, puis tourner sur place : la marque du plateau doit rester dans `local_costmap`.

## Limites connues
- Un objet plus fin que ~2.2° d'angle (colonne décimée) peut passer entre deux colonnes (pied de table : couvert par le LiDAR).
- Champ horizontal 90° : hors champ, seule la mémoire du costmap protège.
- Les obstacles mobiles vus puis partis hors champ restent marqués jusqu'à ce que la caméra revoie la zone à plus de 1 m (ou clear costmap du BT).
