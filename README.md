# Fosa — finale PARC 2026

Solution ROS 2 Jazzy pour le CAYTU Sito-É : découverte des AprilTags, Nav2,
départ par commande ou « allez », visites avec JPEG et arrêt arbitre.

**Branche : `integration/finale-propre-2026-11-06`.**
Voir le [comparatif des branches et l’état de validation](docs/AUDIT-BRANCHES-2026-11-06.md),
l’[architecture](docs/README-SOLUTION.md) et l’[audit ROS](AUDIT-TOPICS-PARC.md).

## Consignes du 6 novembre

- Robot : **camera1 basse, camera2 avant** ; disposition inverse en simulation.
- Mode autonome **sans carte ni SLAM** disponible : tags découverts en direct,
  Nav2 sur des costmaps roulantes dans `odom`.
- Ordre annoncé pour le lendemain : **4,9,1,7**. Le robot lit toujours
  `/parc/tag_order` ; cet ordre sert seulement de défaut aux répétitions PC.
- Barème annoncé : **20 points par tag autonome, 10 si hardcodé**. Les autres
  points et les verdicts restent du ressort des organisateurs.
- Match : **600 secondes réelles** maximum. Visite : centre à 0,5 m, cap à ±12°,
  JPEG du tag et maintien immobile. `parc_msgs/TagVisit` exige au moins 3 s ;
  Fosa conserve **5 s**.

Documents originaux : [docs/parc-phase-finale-docs](docs/parc-phase-finale-docs/).

## 1. Préparer le PC

Placer les quatre paquets officiels à côté du dépôt. Les organisateurs les
installent désormais dans `~/parc_ws` sur le robot ; `~/ros2_ws` reste accepté :

```text
~/parc_ws/src/          (ou ~/ros2_ws/src/)
├── fosa-solution/
├── parc_msgs/
├── parc_referee/
├── parc_robot_bringup/
└── parc_robot_description/
```

### Avec Ubuntu 24.04 et ROS 2 Jazzy installés (chemin principal)

C’est le chemin à utiliser sur le PC de test et sur le Jetson. Docker n’est
pas nécessaire.

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths . ../parc_msgs ../parc_referee \
  ../parc_robot_bringup ../parc_robot_description --ignore-src -r -y
# Vérifier les paquets de simulation et de test :
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-laser-filters ros-jazzy-depthimage-to-laserscan \
  python3-pytest python3-opencv python3-yaml
./simulation/simulation.sh preparer
./outils/verifier.sh
```

Les installations système concernent le PC. Sur le robot partagé, demander
les dépendances manquantes aux organisateurs. Le robot n’a pas besoin de
Gazebo ; `ros-jazzy-ros-gz` ne sert qu’au PC de simulation.

### Option Docker (CI isolée, non requise)

Le même dépôt peut être validé dans l’image Jazzy, notamment pour les tests
réseau DDS :

```bash
./outils/verifier.sh --docker
./outils/verifier.sh --docker --simulation
```

Le premier build télécharge plusieurs Go. L’exécution est isolée du réseau
extérieur ; les sources hôtes sont en lecture seule. Les résultats restent
sous `results/`. [Options Docker](docker/simulation/README.md) : image réutilisée,
emplacement des sources PARC et workspace de sortie. Docker reste un banc de
validation, pas le mode de déploiement du robot.

## 2. Simulation PC

```bash
# Sans préparation : ordre puis graine du monde officiel.
./simulation/simulation.sh course-sans-carte '4,9,1,7' 7
FOSA_SIM_GUI=0 ./simulation/simulation.sh course-sans-carte '4,9,1,7' 7

# Avec SLAM : préparation puis match, même graine, monde réinitialisé.
./simulation/simulation.sh essai '4,9,1,7' 7

# Arrêt dans un second terminal.
./simulation/simulation.sh arreter
```

Le script lance monde, pont de pose, arbitre officiel, moniteur et solution.
Le `PosePublisher` de Gazebo Jazzy publie `/model/sitoe_robot/pose` alors que
le pont officiel attend `sitoe_robot/pose` ; `simulation.sh` ajoute donc un
pont vers `/sim/pose_verite`, utilisé uniquement par le moniteur. Il arme
l’arbitre simulé puis donne le départ. Le moniteur est prêt avant le départ ;
seul ce moniteur utilise la vérité terrain.

Journaux : `~/.fosa_simulation/journaux/…/course/`. `rapport_course.json` exige
quatre visites dans l’ordre, géométrie correcte, JPEG reconnu, maintien valide,
absence de faux départ et de contacts détectés. Un parcours incomplet ou un
délai dépassé retourne un code non nul.

Le budget reste **600 s réelles** : sur un PC lent, cela représente moins de
temps physique simulé. Le délai global du script couvre aussi son démarrage.

Départ vocal sur un PC disposant d’un micro :

```bash
./simulation/simulation.sh voix-preparer
FOSA_SIM_VOIX=1 ./simulation/simulation.sh course-sans-carte '4,9,1,7' 7
```

Attendre `ATTENTE_DEPART`, puis dire « allez ». `FOSA_SIM_MICRO` choisit le
périphérique. Le parcours Docker sans périphérique audio utilise la commande.

## 3. Livrer au robot

Depuis le PC, avec l’adresse communiquée par les organisateurs :

```bash
# Facultatif, avec Internet : Vosk ARM64/Python 3.12 et modèle français.
./deployer_robot.sh --preparer
# Produire une archive sans contacter le robot.
./deployer_robot.sh --archive ../../fosa.zip
# Transférer, vérifier et compiler sur le Jetson.
./deployer_robot.sh --envoyer ROBOT_IP
```

L’archive contient les six paquets Fosa, les outils et la documentation avec
manifeste SHA-256. Chaque livraison compile dans un workspace neuf sous
`~/teams/fosa/livraisons/`. `~/teams/fosa/actuel` est activé après compilation ;
`precedent` conserve l’ancienne livraison. Les données de terrain restent sous
`~/teams/fosa/donnees`. Le déploiement ne relance pas une course déjà active.

Installation manuelle sur le Jetson : `./deployer_robot.sh --installer`.
Le script source `/opt/ros/jazzy`, puis `~/parc_ws` (ou `~/ros2_ws`) pour les
paquets organisateurs. `FOSA_BASE_WS` adapte ce dernier chemin. La voix préparée
est installée hors ligne.

## 4. Créneau d’essai réel

**Réserver un créneau** sur le site communiqué par les organisateurs. Ne pas
doubler une réservation active. Si le créneau est passé sans essai, en réserver
un nouveau ; une ancienne annonce d’horaire libre ne constitue pas une réservation.

```bash
ssh sitoe@ROBOT_IP
cd ~/teams/fosa/actuel/src/fosa-solution
./run.sh diagnostic-dds
./run.sh diagnostic-robot
```

Vérifier odométrie, lidar, caméras, TF et messages arbitre. Mesurer sur place la
taille noire des tags (profil réel : 0,165 m), l’empreinte et le centre du robot.
Les réglages sont dans `caytu_config/config/fosa.yaml`.

Contrôles issus du retour robot du 06/11 :

- `diagnostic_dds.py` doit être exécutable dans la livraison. En cas de
  `Permission denied`, lancer `python3 outils/diagnostic_dds.py` ou
  `chmod +x outils/diagnostic_dds.py` ; la nouvelle archive conserve le mode.
- Le repère réel est `base_frame: base_link`. Remplacer le texte
  `FRAME_ID_AFFICHÉ` des notes par le `frame_id` réellement affiché, par exemple :
  `ros2 run tf2_ros tf2_echo base_link camera1_depth_optical_frame`.
  Si `base_link` n’existe pas, le bringup/`robot_state_publisher` n’est pas lancé.
- Une ancienne session tmux `fosa_course` ou un nœud `course_parc` encore actif
  explique `duplicate session`. `./run.sh arreter` arrête maintenant l’ancienne
  session tmux ; vérifier ensuite `ros2 node list | grep -E 'course_parc|garde_vitesse'`.
- Sur `/cmd_vel`, seule `garde_vitesse` doit publier côté Fosa. Nav2 et
  l’alignement passent par `/fosa/cmd_vel`. Si `course_parc`, `controller_server`,
  `behavior_server` ou `teleop_twist_keyboard` publient encore directement,
  arrêter l’ancienne session avant le match.
- Si le robot expose `/start_motor`, l’appeler une fois, robot posé et dégagé,
  avant le premier déplacement :
  `ros2 service call /start_motor std_srvs/srv/Empty`.

Demander à un organisateur d’armer une **course d’essai**, puis :

```bash
./run.sh course-sans-carte
# Dans un second terminal SSH préparé de la même manière :
./run.sh suivi
# Une fois ATTENTE_DEPART affiché, arbitre armé et terrain disponible :
./run.sh depart
# Ou dire « allez » si la voix a été installée et vérifiée.
```

Le robot reste immobile avant le départ. Vérifier une visite dans le tableau de
bord : identifiant, JPEG, caméra, distance, cap, maintien. `tags_reported` confirme
la réception du signalement, **pas le verdict du juge**. Tester l’arrêt arbitre.

```bash
./run.sh arreter
```

Les sessions sont détachées de SSH. L’arrêt cible uniquement les groupes de
processus Fosa identifiés par jeton. Journaux et états de session sont conservés
dans `~/teams/fosa/donnees/`.

### Préparation SLAM

```bash
./run.sh preparation-manuelle
# Piloter via /fosa/cmd_vel pour bénéficier de la garde Fosa.
./run.sh sauver-carte
./run.sh arreter
# Replacer le robot exactement au départ de la préparation.
./run.sh course
```

`./run.sh preparation` réalise une exploration autonome (720 s maximum).
La préparation démarre les déplacements sans attendre le départ d’un match.
Carte et tags vont dans `~/teams/fosa/donnees/preparation/`.
Sans préparation valide, `./run.sh course` bascule automatiquement en mode
sans carte (odom) pour ne pas gaspiller le match à construire une carte.
`course-sans-carte` force ce mode ; `FOSA_NAVIGATION=carte ./run.sh course`
conserve explicitement l’ancien comportement SLAM en direct.

## 5. Ranking match

1. Garder la livraison et la configuration retenues au créneau. Choisir le mode
   réellement essayé : sans carte ou carte préparée.
2. Après redémarrage du robot, se reconnecter en SSH et revenir dans
   `~/teams/fosa/actuel/src/fosa-solution`. Les processus doivent être relancés.
3. Lancer `./run.sh course-sans-carte` (ou `course`), puis `./run.sh suivi`.
   Vérifier qu’aucun `course_parc` de l’ancienne version ne publie sur `/cmd_vel`
   et que seule `garde_vitesse` y écrit.
4. Faire armer le match. Contrôler l’ordre reçu ; l’annonce `4,9,1,7` ne remplace
   pas le message officiel.
5. Au signal autorisé, donner **un seul départ** : « allez » ou `./run.sh depart`.
6. Laisser la solution exécuter le parcours, contrôler le verdict des juges puis
   terminer la session avec `./run.sh arreter`.

Une coupure SSH laisse la course active : après reconnexion, utiliser `suivi`
plutôt que lancer une deuxième course. Après redémarrage du robot, demander un
nouvel armement selon les instructions du juge. Se **déconnecter du Wi-Fi
lorsqu’on ne l’utilise plus**.

## 6. Dépannage

- **CycloneDDS saturé** : conserver le réseau des organisateurs, puis essayer
  `FOSA_DDS_ETENDU=1 ./run.sh diagnostic-dds`. Si cela réussit, lancer la course
  avec le même préfixe. La surcharge étend les indices sans effacer les peers
  et interfaces hérités ; elle exige `rmw_cyclonedds_cpp`.
- **Garde bloquée** : vérifier les trois scans activés, leurs TF et leur fraîcheur.
  Une caméra absente bloque le mouvement tant que sa source reste activée.
- **Nav2 indisponible** : consulter le journal de lancement ; sans carte, il faut
  `odom -> base_link`, pas `map -> odom`.
- **Voix indisponible** : le nœud indique le modèle ou le format audio manquant.
  Le départ par commande reste disponible.
- **`Permission denied` sur un outil** : utiliser `python3 outils/…` ou
  `chmod +x`; la livraison préserve désormais les modes exécutables.
- **`duplicate session` / `fosa_course`** : ancienne session tmux de la version
  précédente. `./run.sh arreter` la ferme ; vérifier `ros2 node list` avant de
  relancer. Après une simple coupure SSH, ne pas relancer : utiliser `suivi`.
- **TF `base_link` ou caméra absente** : démarrer le bringup, puis
  `./run.sh diagnostic-robot`; il affiche les `frame_id` réels et les TF
  manquantes. Ne pas recopier un texte comme `FRAME_ID_AFFICHÉ`.
- **Le robot ne bouge pas** : vérifier `/start_motor` si le service existe,
  l’état de la garde, la fraîcheur des scans et l’absence d’un ancien
  `course_parc`/téléop sur `/cmd_vel`.
- **Ancienne livraison** : arrêter Fosa puis lancer depuis
  `~/teams/fosa/precedent/src/fosa-solution`. Vérifier la compatibilité des données
  persistantes avec cette version.
