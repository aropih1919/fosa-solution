# Phase réelle PARC 2026 — Plan d'implémentation

> **Pour l'agent qui exécute :** utiliser `executing-plans` (exécution en ligne, lot par lot) ; chaque lot se termine par build + tests + commit. Les étapes utilisent des cases `- [ ]`.

**Objectif :** rendre la solution Fosa exécutable sur le CAYTU Sito-É réel et capable de faire la course officielle : lire l'ordre des tags 1, 4, 7, 9 sur un topic, attendre le départ (commande ou mot « allez »), cartographier en direct, trouver chaque tag, s'arrêter à 0,5 m face au tag (±12°), signaler départ / visites / fin, sans intervention pendant 10 minutes.

**Architecture :** un nouveau paquet `caytu_course` orchestre la course (machine d'états ROS + modules purs testables). La navigation réutilise Nav2 (Smac 2D + RPP + BT bornée) avec des paramètres **générés depuis `caytu_config`** (repères, topics, empreinte, vitesses, sources d'obstacles), et `slam_toolbox` remplace la localisation odométrique. La perception AprilTag est réécrite autour d'un module OpenCV pur compatible 4.6/4.7+. La configuration par défaut devient celle du robot réel ; la simulation passe par un profil de surcharge.

**Pile technique :** ROS 2 Jazzy, Nav2 1.3, slam_toolbox (lifecycle), OpenCV aruco (`DICT_APRILTAG_36H11`, `SOLVEPNP_IPPE_SQUARE`), Vosk (grammaire restreinte), Python 3.12, pytest.

**Sources des exigences :** `docs/parc-phase-finale-docs/PARC2026_Phase_Reelle_Guide_FR.md` (guide officiel), `parc_ros2_topics_updated.md` (inventaire du robot au 02/11), audit du 03/11.

---

## 0. Hypothèses et inconnues (toutes configurables)

| Inconnue | Valeur provisoire | Où la changer |
|---|---|---|
| Topic et type de l'ordre des tags | `/parc/tag_order`, `std_msgs/String` (« 7,1,9,4 », JSON accepté) | `course.order_topic`, `course.order_type` |
| Paquet de statut des organisateurs | topics Fosa `/fosa/statut/*` | `course.status.*` + `caytu_course/statut.py` (seul fichier à adapter) |
| Caméra haute | les deux caméras détectent les tags | `perception.apriltag.cameras` |
| Format de `/speech_audio` | 16 kHz, mono, int16 | `start_word.*` |
| Empreinte réelle, centre géométrique | empreinte simulée, centre = `base_link` | `navigation.footprint`, `navigation.center_offset_x` |
| Cartes pré-construites autorisées | non : cartographie en direct | `course.slam_map_file` |
| Poses de tags mémorisées autorisées | non | `course.memorized_tags_file` |
| Sources d'obstacles caméra (`/camera*/scan`) | désactivées tant que non vérifiées | `navigation.obstacle_sources[*].enabled` |

Règle de sûreté conservée : aucun mouvement avant le départ ; le robot ne publie un statut de visite que pour le tag attendu ; un tag sauté n'est jamais revisité.

## 1. Carte des fichiers

**Créés**
- `caytu_config/caytu_config/modeles_course.py` — schémas `course`, `start_word`, sources d'obstacles, caméras de tags.
- `caytu_config/config/profils/simulation.yaml` — surcharge Gazebo (temps simulé, `base_footprint`, `/odom`, topics simulés).
- `caytu_perception/caytu_perception/apriltag_opencv.py` — détection + pose, sans ROS.
- `caytu_perception/caytu_perception/transformations.py` — quaternions et changements de repère, sans ROS.
- `caytu_course/` (nouveau paquet ament_python) :
  - `ordre_tags.py` — lecture et validation de l'ordre publié.
  - `sequence.py` — séquence des tags, sauts, budget de temps.
  - `geometrie_tags.py` — normale du tag, pose d'approche, erreurs de visite.
  - `memoire_tags.py` — fusion des observations de tags dans `map`, lecture/écriture YAML.
  - `recherche.py` — points de recherche dans la grille SLAM.
  - `alignement.py` — commande d'alignement final et garde avant.
  - `mot_depart.py` — extraction de canal audio et décision « mot de départ ».
  - `statut.py` — adaptateur vers le statut des organisateurs.
  - `noeud_course.py` — nœud de course (machine d'états).
  - `noeud_mot_depart.py` — nœud Vosk.
  - `config/slam_toolbox.yaml`, `launch/course.launch.py`, `test/`.
- `caytu_nav_solution/caytu_nav_solution/parametres_nav2.py` — YAML Nav2 de course généré depuis la config.
- `deployer_robot.sh`, `outils/diagnostic_robot.sh`, `docs/PHASE_REELLE.md`.

**Modifiés**
- `caytu_config/caytu_config/models.py`, `loader.py`, `__init__.py`, `config/fosa.yaml`, `setup.py`, `test/test_config.py`.
- `caytu_interfaces/msg/TagDetection.msg` — poses caméra, robot et map.
- `caytu_perception/caytu_perception/detecteur_tags.py` — réécriture du nœud ; `test/test_perception.py`.
- `caytu_nav_solution/caytu_nav_solution/navigation_runtime.py` — cap imposé, interruption, rotation Nav2.
- `run.sh` — modes robot (`course`, `depart`, `ordre-test`, `sauver-carte`, `diagnostic-robot`), profils.

**Hors déploiement robot (non supprimés)** : `caytu_assistant`, `caytu_semantic_map`, `classificateur_obstacles`, `suivi_personnes`, benchmark, `task_solution.py`. Le script de déploiement ne construit que `--packages-up-to caytu_course`.

---

## Lot 1 — Configuration « robot réel » par défaut (`caytu_config`)

### Tâche 1.1 : schémas de course
**Fichiers :** créer `caytu_config/caytu_config/modeles_course.py` ; tester dans `caytu_config/test/test_config.py`.

- [ ] Écrire les tests qui échouent :
  - `load_config(CONFIG).course.tag_ids == (1, 4, 7, 9)` ; `course.visit_distance_m == 0.5` ; `course.status.visit_topic` non vide.
  - `CourseConfig.from_mapping` refuse des `tag_ids` en double et un `order_type` hors `{string, int32_multi_array}`.
  - `StartWordConfig` refuse `words: []` et `sample_width` hors `{2}`.
  - `ObstacleSourceConfig` refuse `obstacle_max_range >= raytrace_max_range`.
- [ ] Implémenter les dataclasses gelées `ObstacleSourceConfig`, `TagCameraConfig`, `StatusTopicsConfig`, `AlignmentConfig`, `CourseConfig`, `StartWordConfig` avec les helpers `_section/_check_keys/_get/_as_*` de `models.py` (clés inconnues refusées, aucune valeur par défaut dans le code).
- [ ] `pytest caytu_config/test -q` : vert.

### Tâche 1.2 : nouvelles clés des sections existantes
**Fichiers :** `models.py` (`SystemConfig`, `NavigationConfig`, `AprilTagConfig`, `FosaConfig`), `loader.py`, `__init__.py`.

- [ ] Tests : `system.deployment == "robot"`, `use_sim_time is False`, `base_frame == "base_link"`, `ros_domain_id == 0`, `navigation.odom_topic == "/botwheel_explorer/odom"`, `navigation.cmd_vel_topic == "/cmd_vel"`, `perception.apriltag.size_m == 0.165`, `perception.apriltag.ids == (1, 4, 7, 9)`, deux caméras `camera1`/`camera2`.
- [ ] `DEPLOYMENTS` += `robot` ; `NavigationConfig` += `odom_topic, odom_frame, footprint, footprint_padding, center_offset_x, max_linear_speed, max_angular_speed, laser_self_filter, obstacle_sources` ; `AprilTagConfig` += `ids, min_marker_distance_rate, max_rate_hz, detection_image_topic, tf_timeout_sec, cameras` ; `FosaConfig` += `course, start_word`.

### Tâche 1.3 : YAML par défaut et profil simulation
**Fichiers :** `config/fosa.yaml`, créer `config/profils/simulation.yaml`, `setup.py` (installer `config/profils/*.yaml`).

- [ ] Valeurs robot issues de l'inventaire du 02/11 (`/scan_filtered`, `/botwheel_explorer/odom`, `/cmd_vel`, `/camera{1,2}/camera{1,2}/color/*`, domaine 0, temps réel).
- [ ] Profil simulation : `use_sim_time: true`, `base_footprint`, `/odom`, `/robot_base_controller/cmd_vel_unstamped`, `/scan`, `laser_self_filter: true`, caméra `/top_camera_color/*`, domaine 42.
- [ ] Test : avec `FOSA_CONFIG_OVERRIDE=profils/simulation.yaml`, `use_sim_time is True` et `base_frame == "base_footprint"`.
- [ ] Corriger les 3 tests obsolètes (`category` obligatoire) ; adapter `test_la_configuration_par_defaut_est_valide`.
- [ ] Commit : `feat(config): profil robot réel par défaut et sections course/voix`.

## Lot 2 — Détection AprilTag fiable (`caytu_interfaces`, `caytu_perception`)

### Tâche 2.1 : message enrichi
- [ ] `TagDetection.msg` : `header, tag_id, pose_camera, pose_robot, has_pose_robot, pose_map, has_pose_map, distance_m, confidence`. Seul consommateur hors déploiement : `caytu_semantic_map` (lit `pose_map`, inchangé).

### Tâche 2.2 : module pur `apriltag_opencv.py`
- [ ] Tests (OpenCV réel, images synthétiques générées dans le test) :
  - IDs 1, 4, 7, 9 détectés à 1 m et 3 m (640×480, HFOV 69°), marge blanche d'un bit **sur fond gris** ;
  - un tag hors liste (ID 3) est ignoré ;
  - `estimer_pose` avec taille 0,165 m : distance à 2,00 m ± 3 cm ; axe z du tag orienté vers la caméra.
- [ ] Implémentation : `creer_detecteur(famille, taux_distance_min)` (`ArucoDetector` si OpenCV ≥ 4.7, sinon `DetectorParameters_create()` ; **jamais** `DetectorParameters()` + `detectMarkers` sur 4.6 → segfault reproduit), raffinement `CORNER_REFINE_APRILTAG`, `detecter(image, ids_autorises)`, `estimer_pose(coins, taille, K, D)` en `SOLVEPNP_IPPE_SQUARE`.

### Tâche 2.3 : module pur `transformations.py`
- [ ] Déplacer/tester `quaternion_depuis_matrice`, `multiplier_quaternions`, `tourner_vecteur`, et ajouter `composer(translation, rotation, position, orientation)`.

### Tâche 2.4 : nœud `detecteur_tags.py`
- [ ] Paramètres ROS `camera_name`, `image_topic`, `camera_info_topic` (une instance par caméra dans le launch) ; cadence max `max_rate_hz` ; `TransformListener(spin_thread=True)` + `lookup_transform(..., timeout)` ; poses publiées dans les repères caméra, `base_frame` et `map` (si TF disponible) ; image publiée sur `detection_image_topic` quand un tag attendu est vu.
- [ ] Réparer `test/test_perception.py` (plus de mock de numpy/cv2).
- [ ] Commit : `fix(perception): détecteur AprilTag compatible OpenCV 4.6 et poses exploitables`.

## Lot 3 — Cœur de course pur (nouveau paquet `caytu_course`)

### Tâche 3.1 : squelette du paquet
- [ ] `package.xml` (ament_python ; dépend de `rclpy, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, nav2_msgs, action_msgs, tf2_ros, caytu_config, caytu_interfaces, caytu_nav_solution, caytu_nav_bringup, slam_toolbox, python3-numpy, python3-yaml`), `setup.py` (entry points `course_parc`, `mot_depart`, installation `launch/` et `config/`), `resource/caytu_course`, `test/`.

### Tâche 3.2 : `ordre_tags.py`
- [ ] Tests : `lire_ordre("7,1,9,4", (1,4,7,9)) == (7,1,9,4)` ; accepte `"[7, 1, 9, 4]"`, `"7 1 9 4"`, `'{"order": [7,1,9,4]}'`, la liste `[7,1,9,4]` ; lève `OrdreInvalide` pour un tag manquant, un doublon, un ID inconnu, un texte vide.

### Tâche 3.3 : `sequence.py`
- [ ] Tests sur l'ordre `7,1,9,4` : `cible() == 7` ; `marquer_visite(7)` → `cible() == 1` ; `sauter()` → 1 dans `sautes`, `cible() == 9` ; `est_attendu(1)` faux après le saut (jamais de retour) ; `terminee()` après 4 traitements ; `budget_pour_tag(300, 2, 45) == 150`, `budget_pour_tag(60, 3, 45) == 45`, `budget_pour_tag(10, 1, 45) == 10`.

### Tâche 3.4 : `geometrie_tags.py`
- [ ] Test de régression du bug d'audit : robot en (0,0) cap 0, tag vertical en (2,0) dont la normale pointe vers −x (quaternion construit depuis la matrice colonnes `x=(0,-1,0)`, `y=(0,0,1)`, `z=(-1,0,0)`) → `cap_normale` = π ; `pose_approche(…, 0.5)` = (1,5 ; 0 ; 0).
- [ ] Tests `erreurs_visite` (tag exprimé dans le repère robot) : robot pile à 0,5 m face au tag → erreurs nulles ; tag à 0,6 m → erreur distance +0,1 ; robot tourné de 10° → erreur de cap 10° ; décalage latéral de 5 cm → erreur latérale 0,05 ; `center_offset_x` pris en compte.
- [ ] `visite_valide(erreurs, tol_distance, tol_laterale, tol_cap)`.

### Tâche 3.5 : `memoire_tags.py`
- [ ] Tests : moyenne pondérée (poids = 1/distance²) des positions, moyenne circulaire de la normale ; `connait(id)` après une observation ; observations au-delà de `distance_max` ignorées ; `exporter()/importer()` YAML aller-retour ; `oublier(id)`.

### Tâche 3.6 : `recherche.py`
- [ ] Tests sur une grille 5×5 m (résolution 0,05) avec murs en bordure : les points proposés sont à plus de `clairance` des murs, espacés de `pas`, triés par distance au robot, et excluent les zones déjà visitées ; grille inconnue → liste vide.

### Tâche 3.7 : `alignement.py`
- [ ] Tests : erreur de cap 10° → rotation seule, signe correct ; cap aligné et trop loin → avance ≤ `max_linear` ; trop près → recule ; erreur latérale > 2× tolérance → `etat == "renaviguer"` ; tout dans les tolérances → `etat == "aligne"` et vitesses nulles ; `obstacle_devant(points, x_avant, demi_largeur, marge)` bloque l'avance.
- [ ] Commit : `feat(course): logique de course pure (ordre, séquence, géométrie, recherche, alignement)`.

## Lot 4 — Navigation pilotée par la configuration

### Tâche 4.1 : `navigation_runtime.py`
- [ ] Tests : `construire_but(..., cap=1.0)` impose ce cap ; `naviguer_une_fois(..., interrompre=lambda: True)` annule le but et renvoie `"interrompu"` ; comportement inchangé sans ces arguments (tests existants verts).
- [ ] Ajouter `tourner(angle, delai, interrompre)` via l'action Nav2 `spin` (contrôle de collision par la costmap).

### Tâche 4.2 : `parametres_nav2.py`
- [ ] Tests : avec la config robot, aucune occurrence de `base_footprint` ni de `/odom` dans le YAML généré ; `robot_base_frame == "base_link"` partout ; couches d'obstacles = sources `enabled` ; `desired_linear_vel <= max_linear_speed` ; empreinte = `navigation.footprint` ; tolérances du goal checker (`xy 0.08`, `yaw 0.12`) ; `use_sim_time` appliqué partout ; pas de `static_layer` sans carte → la couche statique lit `/map` de SLAM.
- [ ] `ecrire_parametres_course(config, chemin)`.

### Tâche 4.3 : SLAM
- [ ] `caytu_course/config/slam_toolbox.yaml` (online async, `minimum_travel_distance 0.2`, `map_update_interval 2.0`, `max_laser_range 8.0`) ; repères, scan et mode injectés au lancement (`mapping`, ou `localization` + `map_file_name` + `map_start_pose [0,0,0]` si `course.slam_map_file`).
- [ ] Commit : `feat(nav): paramètres Nav2 de course générés depuis la configuration`.

## Lot 5 — Nœud de course, statut et lancement

### Tâche 5.1 : `statut.py`
- [ ] `PublicateurStatut(node, status_config)` : `signaler_depart()`, `signaler_visite(tag_id, image)`, `signaler_fin(motif, resume)`, QoS fiable + transient local. Seul fichier à adapter quand le paquet officiel sera publié.

### Tâche 5.2 : `noeud_course.py`
- [ ] Entrées : ordre (double abonnement : fiable/transient local + best effort/volatile), départ, détections, images de détection, `/map`, TF.
- [ ] États publiés sur `course.state_topic` : `ATTENTE_ORDRE`, `ATTENTE_DEPART`, `DEMARRAGE`, `RECHERCHE`, `APPROCHE`, `ALIGNEMENT`, `VISITE`, `TERMINEE`.
- [ ] Boucle : attendre ordre + départ → chrono → statut départ → Nav2 prêt → pour chaque tag : budget, recherche (rotation Nav2 puis points de recherche, interruption dès que la cible est connue), approche Nav2 (pré-approche si la meilleure observation est lointaine), alignement visuel, validation stable, statut visite + image, maintien → fin (statut fin, export mémoire des tags, vitesse nulle).
- [ ] Arrêt propre sur Ctrl-C/SIGTERM : annulation Nav2 + vitesses nulles.

### Tâche 5.3 : `launch/course.launch.py`
- [ ] Génère le YAML Nav2 ; lance `slam_toolbox`, `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` sous un seul `lifecycle_manager` ; un détecteur par caméra (respawn) ; le nœud mot de départ (si activé) ; le nœud de course (argument `mission:=false` pour cartographier sans mission) ; filtre laser seulement si `laser_self_filter`.
- [ ] Commit : `feat(course): nœud de course, statut et lancement complet`.

## Lot 6 — Départ vocal hors ligne
- [ ] `mot_depart.py` tests : `extraire_canal` (stéréo int16 → canal demandé), `est_mot_depart` (JSON Vosk : texte exact, confiance minimale, mot hors liste refusé, `[unk]` refusé).
- [ ] `noeud_mot_depart.py` : type audio résolu dynamiquement (`audio_common_msgs/msg/AudioData`), armement uniquement pendant `ATTENTE_ORDRE`/`ATTENTE_DEPART`, grammaire `["allez", "[unk]"]`, publication unique du départ (`data: "voix"`).
- [ ] Commit : `feat(course): démarrage vocal hors ligne (Vosk)`.

## Lot 7 — Déploiement robot et documentation
- [ ] `run.sh` : profils (`FOSA_PROFIL=simulation`), domaine lu via `caytu_config`, source de l'espace de travail de base (`FOSA_BASE_WS`), modes `course`, `depart`, `ordre-test`, `sauver-carte`, `diagnostic-robot`.
- [ ] `deployer_robot.sh` : aucun sudo ; vérifie Jazzy, Nav2, slam_toolbox, OpenCV ; venv dans le dossier d'équipe ; Vosk depuis un dossier de roues hors ligne ou PyPI ; build `--packages-up-to caytu_course`.
- [ ] `outils/diagnostic_robot.sh` : topics/types attendus, TF (hauteur des caméras → caméra haute), résolution caméra, type de `/cmd_vel`, débit et taille des messages audio, enregistrement rosbag si disponible.
- [ ] `docs/PHASE_REELLE.md` : installer, lancer, démarrer (commande et voix), check-lists de créneau, questions aux organisateurs.
- [ ] Commit : `chore(deploiement): scripts robot sans sudo et guide phase réelle`.

## Lot 8 — Vérification
- [ ] `colcon build --packages-up-to caytu_course` et tests de tous les paquets touchés.
- [ ] Essai en simulation (profil `simulation`) : ordre de test + départ commande → SLAM, Nav2, recherche, sauts sur budget, statut fin, arrêt propre.
- [ ] Revue de code ; corrections ; commit `test(course): validation en simulation`.

## Auto-relecture
- Couverture des exigences du guide : ordre (3.2, 5.2), départ commande (5.2, 7) et voix (6), cartographie en direct (4.3, 5.3), tags 36h11 1/4/7/9 (2), visite 0,5 m ±12° (3.4, 3.7, 5.2), obstacles fixes/mobiles (4.2), statut départ/visites/fin (5.1), 10 min sans intervention (3.3, 5.2), instructions du gel (7).
- Inconnues isolées dans la configuration ou dans `statut.py`.

---

## Journal d'exécution (03/11)

| Lot | Commit | État |
|---|---|---|
| 1 Configuration robot | `afe975d` | fait, tests verts |
| 2 Détection AprilTag | `cb1c0aa` | fait, nœud vérifié sur image synthétique (pose à 6 mm près) |
| 3 Logique pure | `7b2d643` | fait, 53 tests |
| 4 Nav2 depuis la config + SLAM | `17a16a4` | fait |
| 5 Nœud de course + lancement | `671cbf4` | fait, lancement vérifié |
| 6 Départ vocal | `6d6e4e6` | fait, vérifié avec Vosk réel et voix de synthèse |
| 7 Déploiement + guide | `126ef5b` | fait |
| 8 Validation simulation | `6fcb392` + suite | en cours (Gazebo + 4 tags 36h11) |

Écarts par rapport au plan, tous justifiés par une mesure :
- `caytu_perception/setup.cfg` ajouté : sans lui, `ros2 run`/launch ne trouvaient
  pas les exécutables (même défaut, non corrigé, dans `caytu_assistant` et
  `caytu_semantic_map`, hors déploiement).
- Affinage des coins : fenêtre d'une demi-cellule au lieu de `CORNER_REFINE_SUBPIX`
  fixe (biais de ±8 % sur la distance) ou `CORNER_REFINE_APRILTAG` (90 ms/image,
  portée < 3 m). Erreur mesurée : 0,1 % à 0,5 m, < 3 % jusqu'à 4 m.
- Mot de départ : vocabulaire libre + énoncé exact par défaut ; une grammaire
  restreinte confondait « alors », « allô », « Alex » avec « allez ».
- Clés de configuration ajoutées en cours de route : `course.map_topic`,
  `goal_xy_tolerance_m`, `goal_yaw_tolerance_deg`, `alignment.min_*`,
  `apriltag.detection_image_rate_hz`, `apriltag.tf_max_lag_sec`,
  `start_word.restricted_grammar`.
- Premier essai complet en simulation : budgets en temps réel au lieu de
  l'horloge ROS, TF map <- caméra en retard de 0,3 s, BT abandonnant les buts
  (`default_server_timeout` 20 ms), Ctrl-C transformé en `TypeError` → corrigés
  dans `6fcb392`.
