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
    └── course_parc                 → /fosa/course/etat, /parc/start, /parc/tag_visit, /parc/finish
```

## 2. Entrées du nœud de course

`EntreesCourse` ne fait que mémoriser ; les décisions restent dans la boucle
de `NoeudCourse`.

| Entrée | Topic | Usage |
|---|---|---|
| Ordre des tags | `/parc/tag_order` | Séquence imposée par l'arbitre ; type découvert sur le graphe ROS. |
| Départ interne | `/fosa/course/depart` | Commande `./run.sh depart` ou mot « allez » (relayé à l'arbitre). |
| Départ arbitre | `/parc/start` | Un départ publié directement par l'équipe lance aussi la course. |
| Statut arbitre | `/parc/run_status` | Simple suivi dans le journal. |
| Détections | `/perception/tags` | Dernière pose de chaque tag dans le repère robot. |
| Images | `/camera{1,2}/.../color/image_raw` | Image de preuve associée à la visite. |
| Carte | `/map` | Grille d'occupation pour choisir les points de recherche. |
| Scan | `navigation.scan_topic` | Garde frontale pendant l'alignement. |
| TF | `map`, `odom`, `base_link` | Pose du robot. |

## 3. Machine à états

```text
ATTENTE_ORDRE / ATTENTE_DEPART
        │ départ (commande ou voix) → /parc/start
        ▼
DEMARRAGE  (chronomètre lancé)
        │ Nav2 prêt + robot localisé
        ▼
pour chaque tag de l'ordre :
        ├── la stratégie garde-t-elle ce tag ? sinon : saut (jamais revisité)
        ├── RECHERCHE   tours sur place, puis points libres de la carte
        ├── APPROCHE    Nav2 jusqu'à 0,9 m, face au tag
        ├── ALIGNEMENT  asservissement visuel jusqu'au point à 0,5 m, puis face au tag
        └── VISITE      position stable → /parc/tag_visit (identifiant + JPEG), maintien
        ▼
TERMINEE / TEMPS_ECOULE  → /parc/finish, robot arrêté
```

Règles de temps :

- chronomètre officiel sur l'horloge ROS, marge de sécurité déduite ;
- la stratégie est réévaluée toutes les 2 s, y compris pendant une
  navigation ou un alignement, qu'elle peut interrompre ;
- un tag sauté n'est jamais remis dans la séquence ;
- à l'échéance, le but Nav2 en cours est annulé et le robot s'arrête.

## 4. Stratégie de visite

`caytu_course/strategie.py` (pur, sans ROS) répond à la question : « faut-il
continuer sur la cible actuelle ? ».

1. `_tags_restants` décrit chaque tag restant : point d'approche s'il est en
   mémoire, nombre d'échecs, temps déjà passé dessus.
2. `planifier` évalue toutes les sous-séquences des tags restants (16 au plus) :
   - durée = trajets (distance × détour ÷ vitesse) + temps de visite, plus le
     temps de recherche pour un tag jamais vu ;
   - valeur = points par tag × probabilité de réussite (qui baisse avec les
     échecs et la recherche) ;
   - seuls les plans dont la durée majorée de la marge tient dans le temps
     restant sont retenus ; le plus rentable gagne, puis le plus court.
3. `garder_cible` : si le meilleur plan ne commence plus par la cible, elle
   est sautée. Un plan vide (plus rien ne tient) ne fait rien sauter.

Le journal affiche chaque changement de plan :
`Plan : [1, 9, 4] (durée estimée 132 s, 180 s restantes, 68 points espérés)`.
Les durées (`course.strategy`) sont à recaler sur les essais réels.

## 5. Visite d'un tag

1. `_pose_tag_courante` compose la dernière détection avec la pose actuelle du
   robot. À défaut, la mémoire des tags prend le relais.
2. Si le tag est à plus de 1,2 m, `_approcher` envoie Nav2 au point à 0,9 m
   sur la normale du tag, cap final imposé. Nav2 ne vise jamais le point à
   0,5 m : son contrôle de collision refusait d'approcher le support du tag.
3. `_aligner` applique `commande_alignement` à chaque image :
   - rouler vers le point de visite (0,5 m sur la normale), en marche arrière
     s'il est derrière, en corrigeant le cap ;
   - au point (2,5 cm), tourner sur place face au tag (2°) ;
   - garde d'obstacle avant et arrière sur le lidar ;
   - retour à Nav2 seulement si le tag est trop loin ou vu trop de biais.
4. Après plusieurs mesures stables, `_valider_visite` publie l'identifiant et
   l'image, puis le robot garde la position.

## 6. Arbitre

`caytu_course/statut.py` publie vers l'arbitre officiel (`parc_msgs`, installé
dans l'espace de travail de base du robot, sourcé par `run.sh`) :

| Événement | Topic | Message |
|---|---|---|
| Départ | `/parc/start` | `StartSignal` : `method` 0 (commande) ou 1 (voix), `heard` |
| Visite | `/parc/tag_visit` | `TagVisit` : `tag_id` + image JPEG de la caméra |
| Fin | `/parc/finish` | `Finish` : `reason` |

`caytu_course/arbitre.py` remplit ces messages d'après leur définition réelle
(`get_fields_and_field_types`) : le champ de l'image et le type de l'ordre ne
sont pas documentés, le code s'adapte à ce qui est installé sur le robot.

## 7. Configuration

Tout vient de `caytu_config/config/fosa.yaml` :

- `system` : domaine ROS, repères, horloge ;
- `navigation` : topics, empreinte, vitesses, sources d'obstacles, costmaps ;
- `perception` : tags, caméras, conversion profondeur → scan ;
- `course` : ordre, arbitre, seuils de visite, alignement, stratégie, recherche ;
- `start_word` : audio, modèle Vosk, mot attendu.

Le profil `config/profils/simulation.yaml` surcharge les topics et repères pour
Gazebo. Aucun nœud ne code une valeur métier en dur.

## 8. Déploiement

`deployer_robot.sh` construit uniquement les paquets de la course
(`--packages-up-to caytu_course`) et installe Vosk dans `.fosa_python`, sans
`sudo`. Nav2 et slam_toolbox restent des paquets système fournis par les
organisateurs.
