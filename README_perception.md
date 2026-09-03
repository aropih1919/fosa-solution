# Perception, Localisation & Cartographie — Trinôme 1 (Mpiaro · Edinah · Jeannie)

## Résumé
Ce module fournit une carte statique du stade et une localisation AMCL fiable,
avec un signal `/localization_ready` indiquant quand le robot est prêt pour la navigation.

## Frames confirmées (via view_frames)
- `odom` → `base_footprint` → `base_link` → `{lidar_link, imu_link, ...}`
- Le LiDAR (`lidar_link`) est positionné à `z=0.000` par rapport à `base_link`
  → cause une auto-détection du robot (voir "Problème résolu" ci-dessous)

## Topics utilisés
- `/scan` (brut, LiDAR) → filtré vers `/scan_filtered`
- `/odom` : odométrie roue
- `/map` : carte statique republiée par `map_server`
- `/amcl_pose` : pose estimée par AMCL
- `/localization_ready` (nouveau, publié par notre watchdog) : signal booléen

## ⚠️ Découvertes critiques pour l'équipe
1. **Le but de la tâche est communiqué via `/goal_pose`.**
2. **Le topic de commande réel du robot est `/robot_base_controller/cmd_vel_unstamped`**,
   pas `/cmd_vel` — nécessite un remapping dans la config Nav2 de Trinôme 2.
3. **Le LiDAR se détecte lui-même** (auto-collision du scan avec le châssis) à quatres plages
   angulaires fixes. Corrigé via `laser_filters` (voir `config/laser_filter_params.yaml`).
   Solution durable recommandée : ajuster la hauteur (`z`) du LiDAR dans l'URDF fourni,
   via un fichier de surcharge Xacro (non fait à ce stade, filtre logiciel utilisé en attendant).

## Pipeline complet (ordre de lancement)
1. `parc_robot_bringup task.launch.py`
2. `laser_filters scan_to_scan_filter_chain` (filtre self-detection)
3. `nav2_map_server map_server` (recharge `maps/stadium_map.yaml`) + lifecycle configure/activate
4. `nav2_amcl amcl` + lifecycle configure/activate
5. Donner une pose initiale (2D Pose Estimate dans RViz2)
6. `localization_watchdog.py` → publie `/localization_ready`

## Fichiers livrés
- `config/slam_toolbox_params.yaml`
- `config/laser_filter_params.yaml`
- `config/amcl_params.yaml`
- `maps/stadium_map.yaml` + `.pgm`
- `caytu_nav_solution/localization_watchdog.py`

## État des tâches
- [x] Tâche 1 — Audit robot
- [x] Tâche 2 — Validation TF
- [x] Tâche 3 — Cartographie du stade
- [x] Tâche 4 — Localisation AMCL
- [x] Tâche 5 — Robustesse en environnement dynamique (résultat encourageant, protocole à approfondir — voir détails ci-dessous)
- [x] Tâche 6 — Node localization_watchdog
- [x] Tâche 7 — Ce README

## Tâche 5 — Résultats du test de robustesse

### Protocole
Insertion de deux obstacles statiques (sphère + cube) devant le robot,
pendant que celui-ci était en mouvement (translation), avec observation
de `/amcl_pose` avant/après sur ~84 secondes de déplacement.

### Résultat mesuré
| | Avant | Après (avec obstacles) |
|---|---|---|
| cov_xx | 0.2458 | 0.2298 |
| cov_yy | 0.2782 | 0.1299 |
| cov_yaw | 0.1144 | 0.1112 |

**Conclusion** : la covariance a diminué (pas divergé) malgré la présence
des obstacles insérés — signal positif, AMCL n'a pas été déstabilisé par
ce test.

### ⚠️ Limites connues de ce test (à traiter si le temps le permet)
1. Pas de "groupe témoin" : on n'a pas comparé à une trajectoire équivalente
   SANS obstacle, donc on ne peut pas isoler formellement l'effet spécifique
   de l'obstacle par rapport à l'amélioration normale liée au mouvement seul.
2. Objets STATIQUES testés (sphère/cube posés), alors que la vraie tâche
   PARC2026 implique une foule qui BOUGE en continu — un test plus exigeant,
   pas encore réalisé.
3. Recommandation : approfondir ce test pendant l'intégration bout-en-bout
   avec Trinôme 2 (Jour 3), quand le robot naviguera sur de vraies
   trajectoires longues en présence d'obstacles mobiles scriptés.

## Problème de fond identifié (indépendant des obstacles)
La covariance AMCL se stabilise à un plateau relativement élevé même en
conditions normales (~0.24 sur x/y), probablement lié à des contours
légèrement dédoublés sur `stadium_map.pgm` (carte pas encore parfaite,
voir Tâche 3). Pistes non testées faute de temps :
- Ajuster `alpha3`/`alpha4` (réduire la confiance accordée à l'odométrie)
- Réduire `laser_max_range` pour limiter l'impact des zones de carte floues
- Refaire un passage de cartographie plus lent avec loop closure appuyé