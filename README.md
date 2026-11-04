# Équipe Fosa — PARC Engineers League 2026, phase réelle

Solution de course du robot **CAYTU Sito-É** (Jetson AGX Orin, ROS 2 Jazzy) :
cartographie SLAM en direct, navigation Nav2, détection des AprilTags 36h11,
alignement à 0,5 m face au tag et départ à la voix.

> **Règlement résumé** — ordre des tags 1, 4, 7 et 9 publié par les
> organisateurs au départ ; course de 10 minutes maximum ; départ par commande
> ou par le mot « allez » ; une visite compte lorsque le robot s'arrête à 0,5 m
> du tag, face à lui à ±12°, et garde la position jusqu'à confirmation des
> juges. Le détail officiel est dans
> [`docs/parc-phase-finale-docs/PARC2026_Phase_Reelle_Guide_FR.md`](docs/parc-phase-finale-docs/PARC2026_Phase_Reelle_Guide_FR.md).

## 1. Ce que fait le robot

1. Lit l'ordre des tags publié par l'arbitre sur `/parc/tag_order`.
2. Attend le départ : commande `./run.sh depart` ou mot « allez », signalé à
   l'arbitre sur `/parc/start`.
3. Pour chaque tag, dans l'ordre : le cherche (tours sur place et points de la
   carte SLAM), s'en approche avec Nav2 jusqu'à 0,9 m, finit à 0,5 m face à
   lui par asservissement visuel, signale la visite avec une image JPEG sur
   `/parc/tag_visit`, puis garde la position.
4. Signale la fin sur `/parc/finish` et s'arrête, au plus tard à 10 minutes.

Une stratégie replanifie en continu les tags à tenter selon le temps restant :
un tag est sauté (jamais revisité) quand le tenter coûterait plus de points
qu'il n'en rapporte. Aucun mouvement n'a lieu avant le départ.

## 2. Paquets

| Paquet | Rôle |
|---|---|
| `caytu_config` | Toutes les valeurs métier : robot, navigation, perception, course, mot de départ. |
| `caytu_course` | Machine à états de la course, stratégie, recherche, alignement, interface de l'arbitre, lancement. |
| `caytu_perception` | Détection AprilTag par OpenCV et obstacles hauts par profondeur. |
| `caytu_nav_solution` | Chronomètre de course, client Nav2 et génération du YAML Nav2. |
| `caytu_nav_bringup` | Paramètres Nav2, filtre du lidar et arbre de comportement. |
| `caytu_interfaces` | Messages `TagDetection` et `TagDetectionArray`. |

## 3. Installation sur le robot

Le robot doit déjà disposer de ROS 2 Jazzy, Nav2, slam_toolbox, OpenCV et
`cv_bridge`. Ces paquets sont système : les demander aux organisateurs.

Sur un PC connecté à Internet, préparer les roues Vosk et le modèle français :

```bash
./deployer_robot.sh --preparer
```

Copier tout le dépôt (avec `livraison/`) dans le dossier de l'équipe du robot,
puis, sur le robot :

```bash
./deployer_robot.sh
```

Le script construit les six paquets, vérifie les prérequis et installe Vosk
sans `sudo`. La voix est facultative : si elle échoue, le départ par commande
reste disponible.

## 4. Déroulement d'un créneau

```bash
cd <dossier équipe>/ws/src/fosa-solution

./run.sh diagnostic-robot   # premier créneau : topics, TF, caméras, audio
./run.sh course             # SLAM + Nav2 + tags + mot de départ + mission
./run.sh depart             # départ par commande (ou dire « allez »)
./run.sh etat               # suivre l'état de la course
./run.sh arreter            # essais uniquement
```

`course` tourne dans tmux (ou en arrière-plan avec `nohup`) : une coupure SSH
n'arrête pas la course. Le journal est écrit dans `~/.ros/fosa/`.

Autres modes utiles pendant les essais :

```bash
./run.sh cartographie       # tout sauf la mission : carte, réglages, rosbag
./run.sh sauver-carte       # sérialiser la carte SLAM
./run.sh mesure-rotation    # vérifier l'odométrie en rotation (robot dégagé)
./run.sh ordre-test "7,1,9,4"   # ordre de test, ESSAIS SEULEMENT
```

## 5. Configuration

Tout est dans `caytu_config/config/fosa.yaml` : topics, repères, empreinte,
vitesses, seuils d'alignement, mots de départ. Les valeurs par défaut sont
celles du robot réel (`ROS_DOMAIN_ID=0`, `use_sim_time=false`).

Pour Gazebo, utiliser le profil simulation :

```bash
FOSA_PROFIL=simulation ./run.sh course
```

Variables utiles : `FOSA_BASE_WS` (workspace des paquets organisateurs),
`FOSA_CONFIG_OVERRIDE` (surcharge YAML), `FOSA_DONNEES` (journaux et cartes).

## 6. Arbitre officiel

L'arbitre tourne sur le robot ; `parc_msgs` est dans l'espace de travail de
base (`~/ros2_ws`), que `run.sh` source avant le nôtre (`FOSA_BASE_WS` pour un
autre dossier). `./run.sh course` prévient si `parc_msgs` est introuvable.

| Événement | Topic |
|---|---|
| Ordre des tags (lu) | `/parc/tag_order` |
| Départ | `/parc/start` (`method` 0 commande, 1 voix, `heard`) |
| Visite | `/parc/tag_visit` (`tag_id` + image JPEG) |
| Fin | `/parc/finish` (`reason`) |

Pendant un créneau d'essai : demander à un organisateur d'armer une course,
suivre `ros2 topic echo /parc/run_status`, puis vérifier sur
`http://ROBOT_IP:8080/` ce que l'arbitre a reçu. Consignes complètes :
[`docs/parc-phase-finale-docs/informations-parc.txt`](docs/parc-phase-finale-docs/informations-parc.txt).

Restent à confirmer sur place : la valeur de `reason` pour la fin, le champ
image de `TagVisit`, le format de `/speech_audio` et l'identité de la caméra
haute. `./run.sh diagnostic-robot` enregistre les définitions de `parc_msgs`
et l'état réel des interfaces.

## 7. Tests

```bash
colcon test --packages-select caytu_config caytu_perception caytu_course caytu_nav_solution
colcon test-result --verbose
```

Les tests ne demandent ni robot ni Gazebo.

## 8. Documentation

- [`docs/CHOIX_TECHNIQUES.md`](docs/CHOIX_TECHNIQUES.md) : comment nous avons
  résolu chaque difficulté, et pourquoi (préparation de l'oral) ;
- [`docs/FLUX_DE_LA_SOLUTION.md`](docs/FLUX_DE_LA_SOLUTION.md) : déroulement
  détaillé d'une course dans le code ;
- [`docs/GUIDE_DE_TEST.md`](docs/GUIDE_DE_TEST.md) : tests, simulation et
  créneaux sur le robot.
