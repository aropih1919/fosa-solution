# Architecture Fosa — intégration du 6 novembre

[Guide opérateur](../README.md) · [Comparatif et validation](AUDIT-BRANCHES-2026-11-06.md)

## Six paquets ROS

| Paquet | Fonction |
|---|---|
| `caytu_config` | Configuration typée, profils et réglages d’exploitation |
| `caytu_interfaces` | Messages internes de détection |
| `caytu_perception` | AprilTags OpenCV et profondeur vers scans |
| `caytu_nav_bringup` | Nav2 et arbre de récupération borné |
| `caytu_nav_solution` | Client Nav2, budget et paramètres générés |
| `caytu_course` | Départ, recherche, approche, alignement, arbitre et garde |

```text
caméras → tags mesurés ──────┐
arbitre → ordre/départ/fin ──┼→ course → buts Nav2 ──┐
micro → « allez » ──────────┘       └→ alignement ──┼→ /fosa/cmd_vel
                                                 ↓
lidar + profondeur ────────────────────────→ garde_vitesse → /cmd_vel
         └→ costmaps Nav2
course → /parc/start, /parc/tag_visit (JPEG), /parc/finish
```

Réglages : `caytu_config/config/fosa.yaml`, `profils/` et `exploitation.json`.
Les interfaces ROS publiées conservent leurs noms.

## Navigation

**`carte`** : SLAM en direct ou localisation sur une préparation enregistrée.
Les tags mémorisés ne sont rechargés que si la carte correspondante existe.
Le départ doit être celui de la préparation ; sauvegarder la carte avant
d’arrêter SLAM en préparation manuelle.

**`sans_carte`** : profil appliqué après le profil robot/simulation par
`FOSA_NAVIGATION=sans_carte`. Repère global `odom`, costmaps roulantes, aucun
SLAM dans le launch ou le lifecycle manager, aucune coordonnée de tag chargée.
Le planificateur refuse l’espace inconnu. L’odométrie peut dériver ; la visite
exige toujours une mesure visuelle proche et fraîche.

Nav2 utilise Smac 2D et Regulated Pure Pursuit. La recherche alterne balayage
et trajets vers des points libres non visités, autour du départ mesuré.
La stratégie compare les sous-séquences de l’ordre selon le temps restant et
20 points par tag autonome. Un tag sauté n’est pas repris en remontant l’ordre.

## Perception et preuve

- AprilTags 36h11, IDs 1, 4, 7, 9.
- Robot : camera2 avant, camera1 basse ; simulation : inverse. Les deux caméras
  sont utilisées, chaque preuve conserve son `frame_id`.
- Carré noir : 0,165 m réel ; 0,12 m dans le modèle Gazebo étudié (8 cases noires
  dans une texture de 10 cases sur une face de 0,15 m).
- Pré-approche Nav2 à 0,9 m, puis alignement visuel à 0,5 m.
- Nouvelle vérification de distance, écart latéral, cap et fraîcheur juste avant
  le signalement.
- JPEG provenant d’une image fraîche où le **même tag** a été détecté, au même
  horodatage et dans la même caméra. Aucune image quelconque de secours.
- Sans JPEG, la visite est refusée. Après publication, maintien de 5 s réelles,
  y compris si `FINISHED` arrive immédiatement après le dernier tag.

## Garde et horloges

Toutes les commandes Fosa passent par `/fosa/cmd_vel`. La garde vérifie
l’empreinte balayée en avance, recul et rotation. Elle bloque si une source
activée manque, est périmée ou sans TF. Les scans libres à `+inf` sont acceptés
uniquement avec `inf_is_valid`. La fraîcheur combine horodatage ROS et réception
réelle ; la minuterie monotone continue même si `/clock` se fige.

En match, l’état de course, le budget et les états terminaux arbitre interdisent
également le mouvement. Le budget utilise le temps monotone comme le referee,
avec 5 s de marge. Les durées d’asservissement suivent la physique ROS simulée.
Un arrêt arbitre interrompt les actions Nav2 ; l’arrêt du processus de course
entraîne celui de son launch.

Les dimensions, TF, capteurs et distances physiques restent à mesurer sur le
robot. La garde complète les protections organisateurs.

## Arbitre, voix et exploitation

Les vrais `parc_msgs` viennent du workspace officiel. Leur absence bloque le
match. L’ordre déjà publié est récupéré en QoS `TRANSIENT_LOCAL`. Une ancienne
fin n’interrompt pas une course encore en attente d’un nouveau départ.
`tags_reported` est un accusé de signalement, pas un verdict.

Vosk reconnaît « allez » hors ligne, uniquement en attente du départ. Le
parcours final n’exige pas de conversation générale ; aucun LLM ne commande
directement un moteur.

Le lanceur détache les sessions de SSH et identifie leurs groupes de processus
par jeton. Le déploiement utilise des workspaces neufs, un manifeste SHA-256
et l’activation de `actuel` après compilation.

## Vérification

Le chemin principal est ROS 2 Jazzy natif : `./outils/verifier.sh` après
`./simulation/simulation.sh preparer`. Il exécute les tests logiciels, les tests
réseau DDS et les échanges ROS des départs, JPEG et arrêts. `--docker` reste une
option de CI isolée ; `--simulation` ajoute une répétition officielle.

Le `PosePublisher` de Gazebo Jazzy publie `/model/sitoe_robot/pose`. Le pont
officiel attendant `sitoe_robot/pose`, `simulation.sh` ajoute un pont vers
`/sim/pose_verite`. Le moniteur possède seul cette vérité terrain et exige
quatre visites valides, JPEG reconnus, immobilité, ordre et absence de contacts.
Les verdicts des juges restent distincts de ce rapport technique.

Les paquets organisateurs sont cherchés dans `FOSA_BASE_WS`, sinon `~/parc_ws`
puis `~/ros2_ws` sur le robot.
