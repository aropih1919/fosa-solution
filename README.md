# Équipe Fosa — PARC Engineers League 2026

Solution de course du robot **CAYTU Sito-É** (Jetson AGX Orin, ROS 2 Jazzy).

Pour l’architecture et les choix techniques, lire le [README de la solution](docs/README-SOLUTION.md). Les documents des organisateurs sont dans [`docs/parc-phase-finale-docs/`](docs/parc-phase-finale-docs/). L’[audit des interfaces ROS](AUDIT-TOPICS-PARC.md) recense les vérifications propres au robot.

---

# Guide d’utilisation

Ce guide donne les commandes et procédures dans l’ordre : préparation du PC, simulation, installation du robot, essai réel, puis match. L’architecture est expliquée dans le [README technique](docs/README-SOLUTION.md).

| Besoin | Section |
|---|---|
| Préparer le PC une fois | A. Préparer mon PC |
| Simuler sur le PC | B. Simulation sur PC |
| Installer et vérifier le robot | C. Préparer le robot |
| S’entraîner sur le robot | D. Essai réel |
| Participer au match noté | E. Ranking match |
| Résoudre un problème | F. Dépannage |

Où sont les fichiers :

```text
fosa-solution/
├── README.md              ← ce fichier
├── run.sh                 ← toutes les commandes de la course (robot et simulation)
├── deployer_robot.sh      ← transfère src/ et compile avec --envoyer ROBOT_IP
├── caytu_*/               ← les six paquets ROS 2
├── caytu_config/config/fosa.yaml   ← TOUS les réglages
├── outils/                ← diagnostic du robot, mesure de rotation
├── simulation/            ← Gazebo, sur le PC uniquement (jamais envoyé au robot)
├── docs/README-SOLUTION.md       ← architecture et fonctionnement de la solution
└── docs/parc-phase-finale-docs/  ← documents officiels des organisateurs
```

---

## A. Préparer mon PC (une seule fois)

But : pouvoir construire la solution, lancer les tests et la simulation.

1. **Installer ROS 2 Jazzy, Nav2, slam_toolbox et Gazebo** :

   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-slam-toolbox \
       ros-jazzy-laser-filters ros-jazzy-ros-gz ros-jazzy-depthimage-to-laserscan \
       ros-jazzy-audio-common-msgs ros-jazzy-teleop-twist-keyboard python3-opencv zip
   ```

2. **Placer les dépôts** dans `~/ros2_ws/src` : notre dépôt
   (`fosa-solution`) et le dépôt officiel PARC 2026
   (`parc_robot_bringup`, `parc_robot_description`, `parc_msgs`,
   `parc_referee`). Le dépôt officiel contient maintenant le simulateur,
   l'arbitre et les messages : ne pas garder une ancienne copie de `parc_msgs`
   dans un autre espace de travail.

   Garder une seule copie de `fosa-solution` sous `~/ros2_ws/src` pendant le
   build. Un second worktree contenant les mêmes paquets ROS fait échouer
   `colcon` avec des noms de paquets en double. Pour comparer des branches,
   placer le worktree hors de `src` ou lui donner un workspace séparé.

3. **Construire** :

   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   ```

4. **Préparer la simulation** (construit les paquets officiels et vérifie les
   dépendances) :

   ```bash
   cd ~/ros2_ws/src/fosa-solution
   simulation/simulation.sh preparer
   ```

5. **Vérifier avec les tests** (1 minute, sans simulateur) :

   ```bash
   cd ~/ros2_ws
   colcon test --packages-select caytu_config caytu_perception caytu_course caytu_nav_solution
   colcon test-result --verbose          # vérifier que 0 test échoue
   ```

**Après chaque modification du code** : `cd ~/ros2_ws && colcon build
--packages-up-to caytu_course`, puis relancer les tests.

`run.sh` charge toujours la configuration de ce checkout et choisit le domaine
ROS d'après le profil actif. Il ignore les anciennes surcharges de profil
restées dans le terminal : profil normal `robot`, profil de simulation
`FOSA_PROFIL=simulation`.

À ne pas faire sur le PC :

- **ne jamais lancer `./deployer_robot.sh` sans option sur le PC** : sans
  argument, il construit et installe les dépendances sur le robot ARM. Depuis le
  PC, utiliser `./deployer_robot.sh --envoyer ROBOT_IP` pour déployer ;
- **ne jamais lancer `./run.sh ...` sans `FOSA_PROFIL=simulation`** : sans lui,
  c'est le profil du robot réel (domaine ROS 0), qui peut perturber le robot
  si le PC est sur son Wi-Fi.

---

## B. Simulation sur PC

But : voir toute la solution tourner dans le simulateur officiel PARC 2026,
sans robot. Le PC doit avoir été préparé (section A).

Le simulateur publie les mêmes noms et types d'interface que le robot réel
(`/cmd_vel`, `/botwheel_explorer/odom`, `/scan_filtered`, `camera1`/`camera2`,
`/parc/*`, micro ReSpeaker). Le chemin de sécurité des commandes diffère : le
robot réel passe `/cmd_vel` par `obstacle_avoidance`, tandis que Gazebo le
reçoit par son pont. Un succès en simulation valide les interfaces et aide à
repérer les erreurs, mais ne prouve pas le comportement exact du filtre
matériel. Les topics `/sim/*` sont réservés aux tests et ne sont jamais
utilisés par la solution.

Le script donne à chaque lancement Gazebo une partition dédiée
(`fosa_simulation_<UID>_<PID>`) pour éviter qu'une autre instance Gazebo du PC
détourne les topics `/odom`, `/clock` ou les services du monde. Pour la
personnaliser :
`FOSA_SIM_GZ_PARTITION=mon_essai simulation/simulation.sh essai`.

### B.1 La répétition complète, automatique

Cette commande lance deux simulations successives du même monde : d'abord la
préparation autonome, puis le match. Tu n'as rien à taper pendant le départ
par défaut : le script arme l'arbitre et envoie lui-même le départ par
commande. **Ne dis pas « allez »** dans ce mode.

```bash
cd ~/ros2_ws/src/fosa-solution
simulation/simulation.sh essai
```

- **Gazebo** ouvre l'arène (graine 7 par défaut), démarre le robot et affiche
  ses déplacements.
- Le terminal affiche le récit de la solution, les messages de l'arbitre et
  les mesures des juges simulés.
- Le tableau de bord de l'arbitre est sur `http://localhost:8080/`; la vue des
  juges est `http://localhost:8080/judge?token=parc2026`.
- La préparation et le match utilisent deux lancements neufs du monde, avec
  la même graine et donc le même mobilier et les mêmes tags. Au début du match,
  le robot repart de la position initiale et utilise la préparation enregistrée.

La préparation autonome peut prendre jusqu'à 12 minutes de temps ROS; le match
dure au plus 10 minutes de temps ROS. Gazebo tourne plus ou moins lentement
selon le PC : le temps réel peut être beaucoup plus long. Par défaut, chaque
phase a une limite de deux heures réelles. `FOSA_SIM_GUI=0` masque la fenêtre
Gazebo; cela ne garantit pas une accélération.

Pour lancer sans fenêtre :

```bash
FOSA_SIM_GUI=0 simulation/simulation.sh essai
```

Pour fixer l'ordre et répéter exactement le même monde :

```bash
simulation/simulation.sh essai 7,1,9,4 12
```

Sans ordre imposé, l'arbitre en choisit un au hasard.

Ce que je regarde dans le terminal :

- le récit (voir le [README de la solution](docs/README-SOLUTION.md)) et, en plus discret, la surveillance technique ; une
  ligne rouge `[BLOQUÉ]` ou `[CONTACT]` signale un problème ;
- `=== FIN ===` puis une ligne par tag : visité, ou **pourquoi** il ne l'a pas
  été ;
- `[juge simulé] arbitre : état RUNNING/FINISHED/...` ;
- `[juge simulé] VISITE tag N mesurée dans Gazebo : centre à 0.5xx m, erreur
  de cap ±x° -> VALIDE` ;
- `[juge simulé] arbitre : visite du tag N reçue, photo jpeg (... octets)`.

### B.2 Choisir et reproduire une arène

Chaque graine produit toujours la même arène (mêmes tags, mêmes obstacles) :

```bash
simulation/simulation.sh preparation 12    # graine 12
simulation/simulation.sh course 7,1,9,4 12
```

Le générateur officiel écrit la vérité terrain à côté du monde :
`~/.fosa_simulation/mondes/parc_random_seed12.json`. Le moniteur la lit pour
mesurer les visites ; elle n'est jamais donnée à la solution.

`essai` génère le monde deux fois avec cette même graine : le match commence
donc dans la même disposition que la préparation, à la position de départ.

### B.3 Le départ à la voix dans la simulation

Par défaut, `essai` envoie le départ par commande et il ne faut pas parler.
Pour tester le bonus vocal, le simulateur publie le micro du PC sur les topics
ReSpeaker (`/speech_audio`, `/is_speeching`).

Une seule fois :

```bash
simulation/simulation.sh voix-preparer        # Vosk + modèle français, sans sudo
```

Puis lance la répétition complète avec départ vocal :

```bash
FOSA_SIM_VOIX=1 simulation/simulation.sh essai 7,1,9,4 7
```

Ne parle pas au lancement : attends le message `>>> Départ À LA VOIX : dites
« allez » distinctement dans le micro.` qui apparaît après l'armement de
l'arbitre. Dis alors **« allez »** clairement. Le récit confirme ce que le
micro a reconnu; le départ vocal est envoyé à l'arbitre. `arecord -L` liste
les micros; `FOSA_SIM_MICRO=<nom>` sélectionne un autre micro.

### B.4 Une seule phase

```bash
simulation/simulation.sh preparation 7        # carte + tags de la graine 7
simulation/simulation.sh course 9,4,1,7 7     # match seul sur cette arène
```

La première commande construit et enregistre carte et tags. La deuxième
démarre un nouveau monde avec la même graine; le match seul utilise la
préparation dans `~/.fosa_simulation/preparation`, séparée de celle du robot.
Le script refuse une graine différente. Pour essayer sans préparation, garder
la préparation de côté puis lancer :

```bash
mv ~/.fosa_simulation/preparation ~/.fosa_simulation/preparation_gardee
simulation/simulation.sh course 9,4,1,7 7
```

### B.5 Tout arrêter

Après un Ctrl-C, un terminal fermé, ou le message « Une course ou Gazebo
tourne déjà » :

```bash
simulation/simulation.sh arreter
```

Ne jamais lancer deux simulations à la fois : elles piloteraient le même
robot simulé et les journaux se mélangeraient.

### B.6 Guider le robot, enregistrer la carte, puis tester le match

Cette méthode vérifie le scénario que nous utiliserons pendant les essais :
tu guides le robot pour la préparation, les caméras enregistrent les tags,
puis un monde neuf reprend la même graine et utilise la carte sauvegardée.
Lance les étapes dans l'ordre et laisse chaque terminal ouvert.

1. **Terminal 1 — monde Gazebo** :

   ```bash
   cd ~/ros2_ws/src/fosa-solution
   simulation/simulation.sh monde 7
   ```

   Attendre que le robot démarre.

2. **Terminal 2 — enregistrement guidé** :

   ```bash
   cd ~/ros2_ws/src/fosa-solution
   FOSA_PROFIL=simulation ./run.sh preparation-manuelle
   ```

   Attendre le message « Cartographie guidée ». Ce mode enregistre carte et
   tags et n'envoie aucun but autonome. Dans un troisième terminal, lancer
   `FOSA_PROFIL=simulation ./run.sh suivi` : il annonce une première fois chaque
   ID et indique que sa position est mémorisée. Les observations suivantes sont
   fusionnées et sauvegardées en continu, sans message répété pour chaque image ;
   une nouvelle préparation archive l’ancienne et repart de zéro. Les cartes
   de plusieurs séances ne sont pas fusionnées : repérer tous les tags voulus
   pendant cette préparation.

3. **Terminal 4 — clavier téléop** :

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
   ```

   Parcourir les couloirs sans approcher le robot des meubles. Les caméras
   enregistrent les tags seulement lorsqu'elles les détectent.

4. Quand la carte est suffisante, **arrêter d'abord le téléop** avec Ctrl-C.
   Dans un autre terminal, lancer `FOSA_PROFIL=simulation ./run.sh sauver-carte`.
   Vérifier que la commande confirme `carte.posegraph` et `carte.data`, puis
   affiche les tags mémorisés. Ensuite arrêter la préparation avec
   `FOSA_PROFIL=simulation ./run.sh arreter`.

5. **Terminal 1** : arrêter le monde Gazebo avec Ctrl-C. Le match doit
   redémarrer sur le même monde et à la position initiale.

6. **Terminal 1** : lancer le match avec la carte guidée :

   ```bash
   simulation/simulation.sh course 7,1,9,4 7
   ```

   Le script démarre Gazebo et l'arbitre. Par défaut, il envoie le départ par
   commande : **ne dis pas « allez »**. Le tableau de bord est sur
   `http://localhost:8080/`.

Pour tester le départ vocal, suivre B.3 et lancer `course` avec
`FOSA_SIM_VOIX=1`. Les journaux restent dans `~/.fosa_simulation/journaux/`.

---

## C. Déployer et vérifier le robot

Le transfert et la compilation sont automatisés par `deployer_robot.sh`. Le script suit la consigne des organisateurs : équipe `fosa`, compte `sitoe`, archive contenant seulement `src`, destination `~/teams/fosa`, compilation sur le Jetson. Remplacer `ROBOT_IP` par l’adresse donnée par les organisateurs.

### C.1 Depuis le PC : envoyer et compiler

Depuis un terminal PC, aller dans le dépôt, puis lancer une seule commande :

```bash
cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --envoyer ROBOT_IP
```

Cette seule commande enchaîne les étapes organisateurs :
1. crée `~/ros2_ws/fosa.zip` à partir du seul dossier `src/` ;
2. copie l’archive vers `sitoe@ROBOT_IP:~/` ;
3. remplace `~/teams/fosa/src` sur le robot ;
4. extrait le ZIP, charge `~/ros2_ws/install/setup.bash`, lance `colcon build`, puis charge `install/setup.bash`.

Attendre `Déploiement terminé`. Si le script s’arrête sur une erreur, ne pas passer à la préparation : lire le message affiché. Aucun dossier `build/` ou `install/` du PC n’est envoyé.

**Départ vocal facultatif :** avant l’envoi, lancer `./deployer_robot.sh --preparer` sur le PC, puis refaire `./deployer_robot.sh --envoyer ROBOT_IP`. Après la connexion SSH de C.2 et le `cd` dans le dépôt, lancer `./deployer_robot.sh` pour installer Vosk et son modèle. Si tu sautes ces étapes, le départ par commande reste disponible.

### C.2 Depuis le robot : vérifier les interfaces

Après le message de déploiement réussi, ouvrir un terminal PC vers le robot :

```bash
ssh sitoe@ROBOT_IP
cd ~/teams/fosa/src/fosa-solution
```

Si le départ vocal a été préparé, lancer maintenant `./deployer_robot.sh`.
Puis vérifier les périphériques :

```bash
./run.sh diagnostic-robot
```

Il vérifie les topics, les caméras, le lidar, les repères, l'audio, et note la
définition exacte des messages de l'arbitre. L'audit des interfaces signale
que l'inventaire des TF de caméra pourrait être incomplet. Compare aussi le
`header.frame_id` d'une image à sa transformation vers `base_link` :

```bash
ros2 topic echo --once /camera1/camera1/depth/image_rect_raw --field header.frame_id
ros2 run tf2_ros tf2_echo base_link FRAME_ID_AFFICHÉ
```

Remplacer `FRAME_ID_AFFICHÉ` par la valeur affichée et refaire la vérification
pour camera2. Pour rapatrier le rapport, ouvrir un terminal sur le PC et lancer :

```bash
# sur le PC
scp sitoe@ROBOT_IP:'~/.ros/fosa/diagnostic_*.txt' .
```

Après le `scp`, revenir au terminal SSH du robot pour les mesures ci-dessous.
Si un nom de topic ou de caméra diffère, le corriger dans
`caytu_config/config/fosa.yaml` sur le PC, relancer C.1, puis reprendre la
vérification C.2.

**La mesure de distance** (5 minutes, très important). La visite se joue à
quelques centimètres : un tag imprimé un peu plus grand ou plus petit que
16,5 cm décale le point d'arrêt.

1. `./run.sh cartographie` (caméras et carte, sans mission).
2. Poser un tag face à une caméra, à exactement 1,00 m de l'objectif, au
   mètre ruban.
3. Dans un autre terminal :
   `source ~/ros2_ws/install/setup.bash && ros2 topic echo /perception/tags --field detections`.
4. Lire `distance_m`. Si l'écart dépasse 2 cm, corriger dans `fosa.yaml` :
   `size_m = 0,165 × 1,00 ÷ distance lue` (exemple : 0,97 lu → `size_m: 0.170`).
5. `./run.sh arreter`.

**La rotation** (1 minute, robot dégagé, rien d'autre lancé) :
`./run.sh mesure-rotation`.

---

## D. Essai réel sur le robot (créneaux d’essai, avant le ranking match)

Ce parcours suppose C.1 et C.2 terminés. Garde le terminal SSH de C.2 ouvert : il devient le **terminal 1** du robot. Le robot doit repartir du même point que celui où la préparation commence.

### D.1 Enregistrer la carte et les tags

1. Dégager les couloirs et poser le robot au point de départ du match.
2. Dans le **terminal 1**, choisir un seul mode :
   - Guidage manuel : `./run.sh preparation-manuelle`. Le nœud ne commande
     pas le robot ; utiliser le téléop autorisé.
   - Exploration autonome : `./run.sh preparation`. Le robot explore lui-même.

   Si la préparation s’ouvre dans tmux, `Ctrl-b`, puis `d`, détache l’affichage
   sans arrêter le robot.

3. Dans un **terminal 2** du PC, démarrer le suivi :

   ```bash
   ssh sitoe@ROBOT_IP
   cd ~/teams/fosa/src/fosa-solution
   ./run.sh suivi
   ```

   Le suivi annonce chaque ID la première fois qu’il est repéré et dit que sa
   position est mémorisée. Les observations suivantes sont fusionnées et
   sauvegardées sans répéter le message pour chaque image. En guidage manuel,
   balaie les murs et les couloirs, approche les tags jusqu’à ce qu’une caméra
   les détecte et évite les passages étroits. Une nouvelle préparation archive
   l’ancienne ; les cartes de plusieurs séances ne sont pas fusionnées.

4. Quand la carte est suffisante, arrêter d’abord le téléop. Tant que la
   préparation et SLAM tournent, lancer dans un **terminal 3** :

   ```bash
   ssh sitoe@ROBOT_IP
   cd ~/teams/fosa/src/fosa-solution
   ./run.sh sauver-carte
   ```

   Vérifier la confirmation de `carte.posegraph`, `carte.data` et les IDs de
   tags mémorisés. Dans le même terminal 3, lancer ensuite `./run.sh arreter` ;
   le terminal reste disponible pour la mission.
   Le mode autonome sauvegarde lui-même à la fin de ses 12 minutes.
5. Replacer le robot au point de départ avant le test autonome.

### D.2 La mission sur la carte enregistrée

1. Dans le **terminal 3** conservé depuis D.1, replacer le robot au point de
   départ puis lancer `./run.sh course`. Vérifier « Préparation utilisée ».
   Si `parc_msgs` est introuvable, prévenir un organisateur : les visites ne
   seront pas envoyées à l’arbitre.
2. Demander à un organisateur d’armer une course d’essai et de publier l’ordre.
3. Le **terminal 2** de suivi, démarré en D.1, affiche l’ordre et le récit.
4. Quand l’ordre est reçu, donner le départ : dire « allez » près du robot,
   ou lancer `./run.sh depart` dans un **terminal 4**.
5. Observer le récit et la page `http://IP_DU_ROBOT:8080/` depuis le PC.
6. Après le signal de fin, dans le terminal 4, lancer `./run.sh arreter` si la
   course est encore active.
7. Dans le terminal 2, `Ctrl-C` arrête seulement le suivi.

### D.3 Après le créneau, sur le PC

Rapatrier les journaux et la carte pour les étudier :

```bash
scp sitoe@ROBOT_IP:'~/.ros/fosa/*.log' .
scp -r sitoe@ROBOT_IP:'~/.ros/fosa/preparation' .   # carte.pgm à regarder
```

Si les durées annoncées par les plans sont loin de la réalité, je recale
`course.strategy` dans `fosa.yaml`, puis je refais C.1 au créneau
suivant.

Outils utiles pendant un créneau :

| Commande | Usage |
|---|---|
| `./run.sh cartographie` | Afficher SLAM, Nav2 et les capteurs sans enregistrer une préparation complète. |
| `./run.sh sauver-carte` | Pendant une préparation active, sauvegarder la carte et afficher les tags mémorisés; exécuter avant `./run.sh arreter`. |
| `./run.sh etat` | État brut de la course. |
| `./run.sh ordre-test "7,1,9,4"` | Publier un faux ordre si aucun organisateur ne peut armer. **Essais seulement.** |

---

## E. Le ranking match (le match noté devant le jury)

Même robot, même point de départ et même carte que pendant la préparation. Les 15 minutes de préparation et les 10 minutes de match sont distinctes.

**Préparation (15 minutes)**

1. Suivre D.1, étapes 1 à 4, en choisissant `./run.sh preparation-manuelle`.
2. Avant d’arrêter la préparation, vérifier la carte et les IDs avec
   `./run.sh sauver-carte`. Ne pas relancer `preparation-manuelle` après cela :
   une nouvelle préparation archive la précédente et repart de zéro.
3. Laisser le robot au point de départ utilisé pour cette carte.

**Match (10 minutes)**

1. Dans le terminal 1, lancer `./run.sh course`. Vérifier « Préparation utilisée ».
2. Garder le terminal 2 ouvert avec `./run.sh suivi`. S’il a été fermé, en ouvrir
   un nouveau et exécuter :

   ```bash
   ssh sitoe@ROBOT_IP
   cd ~/teams/fosa/src/fosa-solution
   ./run.sh suivi
   ```

3. Attendre que les organisateurs publient l’ordre. Au signal, dire « allez »
   ou lancer `./run.sh depart` dans un troisième terminal.
4. Dès le départ, ne plus toucher le robot, le clavier ou la session SSH.
   Le robot s’arrête et signale seul la fin à l’arbitre.

Ne jamais lancer `./run.sh ordre-test` ou `./run.sh arreter` pendant le match.

## F. Si quelque chose ne va pas

| Symptôme | Que faire |
|---|---|
| « failed to create symbolic link » au build sur le PC | Supprimer `~/ros2_ws/build/caytu_*` et `~/ros2_ws/install/caytu_*`, puis `colcon build`. |
| « Une course ou Gazebo tourne déjà » (simulation) | `simulation/simulation.sh arreter`. |
| « Le robot simulé n'a pas démarré » alors que Gazebo s'ouvre | Le script attend la découverte du topic `/botwheel_explorer/odom`, puis un message d'odométrie. Si le délai expire, lire `~/.fosa_simulation/journaux/monde_*.log`. |
| « Paquets ROS absents » au déploiement | Demander aux organisateurs d'installer Nav2 et slam_toolbox. |
| `Failed to find a free participant index for domain 0` puis `rclpy...RCLError` | CycloneDDS ne peut pas créer de participant : suivre F.1 ci-dessous. Les imports `rclpy` ont réussi. |
| « parc_msgs introuvable » au lancement | Le dépôt officiel n'est pas construit : `cd ~/ros2_ws && colcon build --symlink-install`. |
| Rien sur la page de l'arbitre | L'arbitre n'a pas démarré : lire `~/.fosa_simulation/journaux/arbitre_*.log` (port 8080 occupé ?). |
| « No image » sur la page de l'arbitre | Vérifier que la caméra a détecté le tag à la visite et que la JPEG n'est pas vide dans le journal de la course. |
| « Aucune préparation » au lancement du match | La carte et les tags n'ont pas été enregistrés : relancer `./run.sh preparation-manuelle` pour guider le robot, ou `./run.sh preparation` pour l'exploration autonome. |
| Le robot se croit ailleurs au début du match | Il n'est pas reparti de la position de départ de la préparation : le reposer et relancer `./run.sh course`. |
| Le robot s'immobilise avec `collision ahead` répété | La protection Nav2 détecte un obstacle sur la trajectoire. Ne pas désactiver la protection : arrêter l'essai, reconstruire le code à jour, puis consulter `course_*.log` et `monde_*.log` si le blocage revient. |
| Aucun tag détecté | Topics des caméras dans `fosa.yaml`, éclairage, distance. |
| Le tag est visible mais aucune pose n'est mémorisée | Afficher `header.frame_id` de l'image et vérifier `tf2_echo base_link FRAME_ID`; la chaîne TF des caméras reste à confirmer sur le robot. |
| Le robot ne bouge pas après le départ | Lire le journal : Nav2 pas prêt, carte absente, topic `/cmd_vel` différent. |
| Le départ vocal ne marche pas | voix absente (vérifier la préparation de Vosk), format audio, seuil `min_confidence`. Départ par commande en attendant. |
| Le robot dérive en tournant | `./run.sh mesure-rotation`. |
| Tags sautés trop tôt ou trop tard | Recaler `course.strategy` dans `fosa.yaml` à partir des plans du récit. |

**Encore à confirmer sur le robot** (le diagnostic C.2 y répond) : les TF
complètes des caméras, le format du son de `/speech_audio`, et quelle caméra
est la caméra haute. Les messages
officiels fixent déjà `StartSignal.method` (0 = commande, 1 = voix),
`TagVisit.image` (`sensor_msgs/CompressedImage`) et `Finish.reason`
(0 = terminé, 1 = abandon).

### F.1 CycloneDDS : aucun indice de participant libre

Depuis le dossier de la solution **sur le robot**, enregistrer d'abord le
diagnostic. Il importe `rclpy`, crée un nœud temporaire puis le détruit :

```bash
./run.sh diagnostic-dds 2>&1 | tee diagnostic-dds.log
```

Si le journal contient `Failed to find a free participant index`, essayer :

```bash
FOSA_DDS_ETENDU=1 ./run.sh diagnostic-dds 2>&1 | tee diagnostic-dds-etendu.log
```

Cette option ajoute `caytu_config/config/cyclonedds.xml` après le contenu
existant de `CYCLONEDDS_URI`. Elle sélectionne automatiquement un indice libre
entre **0 et 119**, en conservant les interfaces réseau, peers et réglages
multicast hérités. Elle s'applique uniquement à CycloneDDS et ne change pas
le domaine ROS. Avec les ports DDS standards, dépasser 119 ferait empiéter
les ports unicast sur le domaine suivant. Une configuration de ports
personnalisée doit être examinée séparément.

Si le diagnostic affiche **`[OK] Nœud ROS créé`**, reprendre les commandes
habituelles avec cette option dans **chaque terminal SSH**, par exemple :

```bash
export FOSA_DDS_ETENDU=1
./run.sh suivi
```

Dans le terminal de lancement, exporter également cette variable avant
`./run.sh course` ou `./run.sh preparation-manuelle`. Le lancement détaché
transmet explicitement la configuration DDS à ses processus. Les processus
déjà démarrés conservent leur ancienne configuration. Le diagnostic valide
la création d'un nœud, pas la réception des capteurs : vérifier ensuite
`./run.sh diagnostic-robot` et le suivi.

Si l'échec persiste, relever les processus et ports **sans arrêter les services
des organisateurs** :

```bash
ps -eo user,pid,ppid,args > processus-dds.txt
ss -lunp > ports-dds.txt
printf '%s\n' "${CYCLONEDDS_URI:-}" > configuration-dds.txt
```

Transmettre ces fichiers et les deux diagnostics. Il faut alors identifier
les processus accumulés, les ports occupés ou une configuration réseau
incompatible ; élargir une plage déjà entièrement occupée ne suffit pas.
Pour désactiver la surcharge dans un terminal : `unset FOSA_DDS_ETENDU`.

---

## Référence rapide des commandes

Les commandes `./run.sh` se lancent depuis `~/teams/fosa/src/fosa-solution` sur le robot. Pour les lancer sur Gazebo, préfixer par `FOSA_PROFIL=simulation`. `./run.sh help` affiche l’aide intégrée.

### Commandes Fosa

| Commande | Usage |
|---|---|
| `./run.sh preparation` | Exploration autonome : crée une nouvelle préparation, explore et enregistre carte et tags. |
| `./run.sh preparation-manuelle` | Lance SLAM et la mémorisation des tags pendant que l’équipe pilote le robot. |
| `./run.sh suivi` | Affiche en direct les tags repérés et l’état de la course ; lancer dans un autre terminal. |
| `./run.sh sauver-carte` | Sauvegarde la carte SLAM et affiche les IDs mémorisés ; exécuter avant d’arrêter la préparation. |
| `./run.sh course` | Charge la dernière préparation, attend l’ordre de l’arbitre puis le départ. |
| `./run.sh depart` | Donne le départ par commande quand la course l’attend. |
| `./run.sh arreter` | Arrête la préparation ou la course. |
| `./run.sh cartographie` | Lance SLAM, Nav2 et la perception sans démarrer une préparation complète. |
| `./run.sh diagnostic-robot` | Vérifie les interfaces et périphériques au premier créneau. |
| `./run.sh diagnostic-dds` | Vérifie les imports et crée un nœud temporaire pour diagnostiquer DDS. |
| `./run.sh mesure-rotation` | Mesure la rotation du robot ; course arrêtée et robot dégagé. |
| `./run.sh etat` | Affiche le topic d’état brut de la course. |
| `./run.sh ordre-test "7,1,9,4"` | Publie un faux ordre pour les essais uniquement ; jamais au ranking match. |

### Commandes de simulation

| Commande | Usage |
|---|---|
| `simulation/simulation.sh preparer` | Construit les paquets de simulation et vérifie les dépendances. |
| `simulation/simulation.sh essai [ordre] [graine]` | Préparation autonome puis match sur deux mondes neufs de même graine. |
| `simulation/simulation.sh preparation [graine]` | Enregistre carte et tags pour la graine choisie. |
| `simulation/simulation.sh course [ordre] [graine]` | Lance le match sur la dernière préparation de cette graine. |
| `simulation/simulation.sh monde [graine]` | Lance seulement Gazebo et le robot, pour un guidage manuel. |
| `simulation/simulation.sh arreter` | Arrête la simulation. |
| `simulation/simulation.sh voix-preparer` | Prépare Vosk et le modèle vocal sur le PC. |
| `simulation/simulation.sh moniteur [graine]` | Affiche le récit et les juges pendant un monde déjà lancé. |
| `simulation/simulation.sh micro` | Publie le microphone du PC vers les topics audio de simulation. |

Sans ordre fourni, l’arbitre tire l’ordre au hasard. La graine par défaut est 7. Options courantes : `FOSA_SIM_GUI=0` masque Gazebo ; `FOSA_SIM_VOIX=1` teste le départ vocal ; `FOSA_SIM_DUREE_MAX` règle la limite de temps réel d’une phase (7 200 s par défaut).
