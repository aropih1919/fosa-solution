# Flux de la solution

Vue d'ensemble de ce qui tourne sur le robot pendant une course, du lancement
jusqu'au signal de fin.

## 1. Lancement

`./run.sh course` source ROS, le workspace du robot et la configuration, puis
lance `caytu_course/launch/course.launch.py`. Le launch :

1. charge `caytu_config/config/fosa.yaml` (et le profil actif) ;
2. génère un YAML Nav2 temporaire à partir de `nav2_params.yaml`,
   `controller_rpp.yaml` et du profil de navigation ;
3. démarre un seul `lifecycle_manager` pour slam_toolbox, le contrôleur, le
   planificateur, le serveur de comportements et le BT navigator ;
4. démarre un détecteur AprilTag par caméra, un nœud profondeur → scan par
   caméra, le nœud du mot de départ et le nœud de course.

En simulation seulement, `laser_filters` masque les roues du lidar et publie
`/scan_filtered`. Sur le robot, le lidar des organisateurs fournit déjà
`/scan_filtered`.

```text
run.sh course
└── course.launch.py
    ├── lifecycle_manager_course
    │   ├── slam_toolbox            → /map, TF map→odom
    │   ├── controller_server       → /cmd_vel
    │   ├── planner_server
    │   ├── behavior_server         → spin, backup, wait
    │   └── bt_navigator            → /navigate_to_pose
    ├── tag_detector_camera1/2      → /perception/tags, /perception/tags/image
    ├── scan_profondeur_camera1/2   → /fosa/scan_camera1, /fosa/scan_camera2
    ├── mot_depart                  → /fosa/course/depart
    └── course_parc                 → /fosa/course/etat, /fosa/statut/*
```

## 2. Entrées du nœud de course

`EntreesCourse` ne fait que mémoriser ; les décisions restent dans la boucle
de `NoeudCourse`.

| Entrée | Topic | Usage |
|---|---|---|
| Ordre des tags | `course.order_topic` (`/parc/tag_order`) | Séquence imposée par les organisateurs. |
| Départ | `course.start_topic` (`/fosa/course/depart`) | Commande `./run.sh depart` ou mot « allez ». |
| Détections | `/perception/tags` | Dernière pose de chaque tag dans le repère robot. |
| Images | `/camera{1,2}/.../color/image_raw` | Image de preuve associée à la visite. |
| Carte | `/map` | Grille d'occupation pour choisir les points de recherche. |
| Scan | `navigation.scan_topic` | Garde frontale pendant l'alignement. |
| TF | `map`, `odom`, `base_link` | Pose du robot. |

## 3. Machine à états

```text
ATTENTE_ORDRE / ATTENTE_DEPART
        │ départ (commande ou voix)
        ▼
DEMARRAGE  (chrono lancé, signal de départ publié)
        │ Nav2 prêt + robot localisé
        ▼
pour chaque tag de l'ordre :
        ├── RECHERCHE   tours sur place, puis points libres de la carte
        ├── APPROCHE    but Nav2 à 0,9 m (ou 0,5 m si le tag est proche)
        ├── ALIGNEMENT  asservissement visuel à 0,5 m, cap ±5°
        ├── VISITE      position stable, image + identifiant publiés
        └── saut        budget du tag épuisé, jamais revisité
        ▼
TERMINEE / TEMPS_ECOULE  (fin publiée, robot arrêté)
```

Règles de temps :

- chronomètre officiel sur l'horloge ROS, marge de sécurité déduite ;
- budget par tag calculé sur le temps restant et le nombre de tags restants ;
- un tag sauté n'est jamais remis dans la séquence ;
- à l'échéance, le but Nav2 en cours est annulé et le robot s'arrête.

## 4. Visite d'un tag

1. `_pose_tag_courante` compose la dernière détection avec la pose actuelle du
   robot. À défaut, la mémoire des tags prend le relais.
2. `pose_approche` calcule le point à 0,5 m sur la normale du tag.
3. Nav2 rejoint ce point avec le cap final imposé (le planificateur ne remplace
   pas l'orientation d'arrivée).
4. `_aligner` corrige distance, décalage latéral et cap par petites vitesses,
   avec une garde d'obstacle frontale et arrière.
5. Quand la position est stable, `_valider_visite` publie l'identifiant et
   l'image, puis le robot garde la position.

## 5. Publication du statut

`caytu_course/statut.py` est le seul point à adapter quand le paquet officiel
sera publié. En attendant :

| Événement | Topic | Message |
|---|---|---|
| Départ | `/fosa/statut/depart` | `String` JSON (ordre, source) |
| Visite | `/fosa/statut/tag` | `Int32` identifiant |
| Image | `/fosa/statut/image` | `Image` de la caméra |
| Fin | `/fosa/statut/fin` | `String` JSON (motif, résumé) |

QoS fiable et transient local : un écran lancé en retard reçoit le dernier
état.

## 6. Configuration

Tout vient de `caytu_config/config/fosa.yaml` :

- `system` : domaine ROS, repères, horloge ;
- `navigation` : topics, empreinte, vitesses, sources d'obstacles, costmaps ;
- `perception` : tags, caméras, conversion profondeur → scan ;
- `course` : ordre, départ, statut, seuils de visite, budgets, recherche ;
- `start_word` : audio, modèle Vosk, mot attendu.

Le profil `config/profils/simulation.yaml` surcharge les topics et repères pour
Gazebo. Aucun nœud ne code une valeur métier en dur.

## 7. Déploiement

`deployer_robot.sh` construit uniquement les paquets de la course
(`--packages-up-to caytu_course`) et installe Vosk dans `.fosa_python`, sans
`sudo`. Nav2 et slam_toolbox restent des paquets système fournis par les
organisateurs.
