# Équipe Fosa — PARC Engineers League 2026, phase réelle

Solution de course du robot **CAYTU Sito-É** (Jetson AGX Orin, ROS 2 Jazzy).

Ce fichier est le seul à lire. Il a deux parties :

- **Partie 1 — Comment marche notre solution** : toute la solution expliquée
  en phrases, sans lire le code. C'est aussi ce qu'il faut savoir pour
  répondre au jury.
- **Partie 2 — Ce que je dois faire** : tester en simulation sur le PC, puis
  déployer et tester sur le robot réel, étape par étape.

Les documents officiels des organisateurs sont dans
[`docs/parc-phase-finale-docs/`](docs/parc-phase-finale-docs/) (règlement,
liste des topics du robot, consignes de l'arbitre). On n'y touche pas.

---

# Partie 1 — Comment marche notre solution

## 1.1 Ce que le robot doit faire

Le jour de la course, les organisateurs publient un ordre tiré au sort pour
les quatre AprilTags 1, 4, 7 et 9, par exemple 7, 1, 9, 4. Le robot ne
connaît ni la carte de l'arène (5 m × 5 m, des tables, des chaises, parfois
des personnes) ni la position des tags.

Pour chaque tag, dans l'ordre, il doit le trouver, puis s'arrêter avec son
centre à **0,5 m** du tag, face à lui à **±12°** près, et garder cette position
le temps que les juges valident. À chaque visite, il envoie à l'arbitre le
numéro du tag et une photo de la caméra.

Le reste du règlement :

- la course dure **10 minutes** au plus, à partir du départ ;
- le départ se donne par une commande ou par le mot « allez » (5 points de
  bonus pour la voix) ;
- chaque tag visité rapporte **25 points** : 20 pour la visite et 5 de
  navigation sûre, dont on retire les pénalités de contact ;
- on peut sauter un tag, mais on ne peut jamais y revenir ;
- personne ne touche le robot après le départ.

## 1.2 Vue d'ensemble

La solution tient en six paquets ROS 2 :

| Paquet | Rôle, en une phrase |
|---|---|
| `caytu_config` | Toutes les valeurs réglables (vitesses, distances, topics, temps) dans un seul fichier : `caytu_config/config/fosa.yaml`. |
| `caytu_perception` | Voir les tags avec les caméras, et voir les obstacles hauts (plateaux de table) avec la profondeur. |
| `caytu_nav_bringup` | Les réglages de Nav2, le logiciel qui déplace le robot. |
| `caytu_nav_solution` | Le chronomètre de la course et le client qui envoie les destinations à Nav2. |
| `caytu_course` | Le « cerveau » de la course : ordre, départ, stratégie, recherche, approche, visites, arbitre. |
| `caytu_interfaces` | Le format du message « tag détecté » partagé entre la perception et la course. |

Une commande, `./run.sh course`, lance tout en même temps :

```text
                      ┌──────────────── caytu_course (le cerveau) ─────────────────┐
  arbitre ── ordre ──►│ attend l'ordre et le départ                                │
  micro ── « allez » ►│ stratégie : quels tags tenter ?                            │
                      │ pour chaque tag : chercher → approcher → s'aligner → visite│──► arbitre
                      └──────▲─────────────────────▲───────────────────┬───────────┘    (départ,
                             │ tags vus            │ carte, position   │ destinations   visites, fin)
  caméras ──► détection AprilTag            lidar ──► SLAM          Nav2 (planner + controller)
  caméras ──► profondeur → obstacles hauts ───────────────────────────►│ cartes de coût
                                                                        └──► moteurs (/cmd_vel)
```

## 1.3 Savoir où l'on est : la carte faite en direct (SLAM)

Le robot n'a pas de carte au départ. **slam_toolbox** en construit une
pendant la course, à partir du lidar. Le lidar mesure la distance aux murs
et aux meubles tout autour du robot. En comparant chaque nouvelle mesure à
la carte déjà construite, slam_toolbox fait deux choses à la fois :

- il **agrandit la carte** au fur et à mesure que le robot découvre l'arène ;
- il **corrige la position du robot** sur cette carte. L'odométrie des roues
  dérive avec le temps ; le SLAM la recale sur les murs.

La carte est une grille de cases de 5 cm : libre, occupée ou inconnue. Elle
sert à la fois à Nav2 (pour planifier) et à notre recherche de tags.

## 1.4 Se déplacer sans rien heurter : Nav2

Nav2 est le logiciel standard de ROS 2 pour déplacer un robot. On lui donne
une destination (une position et une orientation) et il s'occupe du reste.
Ses pièces travaillent ensemble :

- **Les cartes de coût.** Nav2 transforme la carte en « carte de coût » : les
  obstacles sont interdits, et une marge de sécurité autour d'eux coûte cher,
  pour que le robot préfère passer loin. Il y en a deux :
  - la **carte globale** couvre toute l'arène et sert à planifier le trajet ;
  - la **carte locale** est une fenêtre de 6 m autour du robot, mise à jour
    10 fois par seconde avec les obstacles vus à l'instant (une personne qui
    passe, une chaise déplacée).

  Le lidar alimente ces cartes, et nos « faux lidars » issus de la
  profondeur des caméras aussi (voir 1.5).
- **Le planner (planificateur)** cherche le chemin. À partir de la carte
  globale, le planificateur **Smac 2D** calcule le chemin le plus court et le
  plus sûr jusqu'à la destination, en contournant les obstacles et leur marge.
- **Le controller (contrôleur)** suit ce chemin. Le contrôleur **Regulated Pure
  Pursuit (RPP)** vise un point un peu plus loin sur le chemin, comme un
  conducteur regarde la route devant lui, et calcule à chaque instant la
  vitesse d'avance et de rotation à envoyer aux moteurs. Il ralentit dans les
  virages serrés et près des obstacles, tourne sur place si le chemin part
  trop sur le côté, et s'arrête s'il prévoit une collision dans la seconde.
- **Le chef d'orchestre (BT navigator)** enchaîne le tout avec un « arbre de
  comportement ». Il fait replanifier le chemin deux fois par seconde : si un
  obstacle apparaît, le chemin le contourne. Si le robot est bloqué, il tente
  des manœuvres de secours (effacer les obstacles fantômes, tourner sur
  place, reculer, attendre), avec un nombre d'essais borné : jamais de boucle
  infinie.
- **La vitesse** est plafonnée à 0,3 m/s : l'arène est petite et des
  personnes peuvent s'y trouver.

## 1.5 Voir : les tags et les obstacles hauts

Le robot a deux caméras RealSense D435i, qui donnent chacune une image
couleur et une image de profondeur.

**Les tags.** Sur chaque image couleur, OpenCV cherche les AprilTags de la
famille 36h11. Pour un tag trouvé, on connaît la position de ses quatre coins
dans l'image et sa taille réelle (16,5 cm). OpenCV en déduit où est le tag en
3D par rapport à la caméra, avec la méthode `IPPE_SQUARE`, faite pour les
carrés plats. On en tire la distance au tag et la direction de sa face. Cette
position est ensuite convertie dans le repère du robot, puis dans celui de
la carte.

Un réglage a compté : avec les paramètres par défaut, OpenCV ratait les tags
imprimés entourés d'une fine marge blanche sur un fond gris. Le détecteur est
réglé pour ce cas et vérifié sur des images de synthèse.

**Les obstacles hauts.** Le lidar est posé bas. Il voit les pieds d'une table,
mais pas son plateau, contre lequel le robot pourrait taper. On transforme
donc l'image de profondeur de chaque caméra en « faux lidar » : on garde
seulement les points situés entre deux hauteurs utiles, et on les envoie à
Nav2 comme un lidar de plus. Le plateau de table apparaît alors dans les
cartes de coût.

## 1.6 Se souvenir des tags

Chaque fois qu'un tag est vu, même si ce n'est pas celui qu'on cherche, sa
position est mémorisée sur la carte. Les vues de près comptent plus que les
vues de loin, car une vue lointaine est imprécise : en simulation, une vue à
1,5 m pouvait décaler la position de 60 cm. Une vue nettement plus proche
remplace les anciennes.

Ainsi, en cherchant le tag 7, le robot apprend souvent où sont le 9 ou le 4.
Plus tard, il ira directement vers eux sans chercher.

## 1.7 Chercher un tag qu'on n'a jamais vu

Si la cible n'est pas en mémoire, le robot :

1. fait un tour lent sur lui-même, pour que les caméras balaient les murs ;
2. va vers le point libre de la carte le plus proche qu'il n'a pas encore
   visité, loin des obstacles ;
3. recommence.

Dès que la cible apparaît sur une image, la recherche s'arrête.

## 1.8 Se placer à 0,5 m face au tag

C'est l'étape qui nous a demandé le plus de travail. Elle se fait en deux
étages.

1. **Nav2 amène le robot à 0,9 m du tag**, face à lui. On ne demande pas à
   Nav2 d'aller à 0,5 m : en simulation, son contrôleur voyait le support du
   tag juste devant, croyait à une collision et refusait d'avancer. Deux
   tags sur quatre étaient perdus ainsi.
2. **Un asservissement visuel finit le trajet.** À chaque image, on calcule le
   point exact où doit se trouver le centre du robot : à 0,5 m, sur l'axe qui
   sort de la face du tag. Le robot :
   - tourne vers ce point, puis roule vers lui en corrigeant sa direction à
     chaque image (en marche arrière si le point est derrière) ;
   - une fois au point (à 2,5 cm près), tourne sur place pour faire face au
     tag (à 2° près).

   Pendant ces manœuvres, une garde lit le lidar et bloque l'avance ou le
   recul si un obstacle est trop près.

Nous visons 2,5 cm et 2°, bien plus strict que les ±12° du règlement, pour
garder de la marge face aux juges. La visite n'est validée qu'après plusieurs
mesures stables d'affilée. Le robot envoie alors le numéro et la photo à
l'arbitre, et garde la position.

Si le tag est vu trop de biais ou de trop loin, l'asservissement rend la main
à Nav2, qui ramène le robot à 0,9 m, bien en face.

## 1.9 La stratégie : quels tags tenter, et quand en sauter un

Aller toujours vers le tag suivant n'est pas forcément le meilleur choix. Un
tag introuvable, ou très loin des autres, peut coûter plus de temps qu'il ne
rapporte de points.

**Ce que dit le règlement :** l'ordre est imposé, on peut sauter un tag sans
jamais y revenir, et les tags suivants rapportent toujours leurs points.

**L'idée clé :** comme l'ordre est imposé, les seuls choix possibles sont de
garder ou de sauter chaque tag restant. Pour 4 tags, cela fait au plus 16
plans. On peut donc **tous les comparer** à chaque instant : la réponse est
exacte et instantanée.

**Pour chaque plan, le robot estime :**

- **sa durée**. Pour un tag déjà repéré : distance à parcourir × 1,4 (on ne
  va jamais en ligne droite à cause des meubles), divisée par la vitesse
  réelle, plus le temps de la visite. Pour un tag jamais vu, on ajoute un
  temps de recherche ;
- **ses points**. 25 points par tag, multipliés par une chance de réussite.
  Cette chance baisse à chaque échec sur le tag, et quand la recherche d'un
  tag dure plus longtemps que prévu.

**La décision :**

- on écarte les plans qui ne tiennent pas dans le temps restant, avec 20 % de
  marge de sécurité ;
- parmi les autres, on garde celui qui rapporte le plus, puis le plus court ;
- le robot saute la cible actuelle seulement si le meilleur plan sans elle
  rapporte au moins 5 points de plus que le meilleur plan avec elle.

Sans ce seuil, en simulation, le robot sautait le premier tag au bout de
2 secondes, à cause d'une différence infime entre des plans équivalents.

**Le calcul est refait toutes les 2 secondes**, y compris pendant un trajet,
avec la vraie position du robot et les tags découverts entre-temps : c'est
un « horizon glissant ». Le temps déjà perdu sur un tag n'entre jamais en
compte ; seul compte ce qui peut encore être gagné.

**Exemple.** L'ordre est 7, 1, 9, 4. Le 7 est loin au nord-ouest, les trois
autres près du départ, et il reste 2 minutes.

- Les quatre tags demandent 181 s : impossible.
- Tout plan à trois tags qui garde le 7 dépasse 139 s : impossible aussi.
- Le plan 1-9-4 tient en 106 s.

Garder le 7, c'est deux tags (50 points) ; le sauter, c'est trois tags
(75 points). Le robot saute donc le 7 tout de suite. Avec 3 minutes 10, les
quatre tiennent, et il les fait tous.

**Pourquoi pas un algorithme plus compliqué ?**

- Le problème du voyageur de commerce ne sert à rien : changer l'ordre ne
  fait que perdre des points.
- Un graphe et une recherche de chemin maison feraient doublon : Nav2 calcule
  déjà les chemins sur la vraie carte.
- L'apprentissage automatique demanderait des données que nous n'avons pas
  sur l'arène réelle, et une décision explicable vaut mieux devant un jury.

Le journal affiche chaque nouveau plan, par exemple :
`Plan : [1, 9, 4] (durée estimée 106 s, 120 s restantes, 68 points espérés)`.

## 1.10 Le départ : par commande ou à la voix

- **Par commande** : `./run.sh depart`.
- **À la voix** : le micro du robot publie le son ; la reconnaissance vocale
  **Vosk** tourne sur le robot, sans Internet. Le robot n'écoute que pendant
  l'attente du départ, et n'accepte que le mot exact « allez », avec une
  confiance minimale.

Pourquoi cette prudence : si un bruit du public lance la course trop tôt, la
course est recommencée et le bonus est perdu. Une reconnaissance limitée à
quelques mots confondait « alors », « allô » ou « Alex » avec « allez ». Nous
reconnaissons donc la phrase librement, puis nous exigeons exactement
« allez ».

Aucun mouvement n'a lieu avant le départ.

## 1.11 Parler à l'arbitre

L'arbitre officiel tourne sur le robot. Nous échangeons avec lui par des
topics ROS :

| Quand | Topic | Contenu |
|---|---|---|
| Avant le départ | `/parc/tag_order` (lu) | L'ordre des tags. |
| Au départ | `/parc/start` | `method` 0 = commande, 1 = voix ; `heard` = mot entendu. |
| À chaque visite | `/parc/tag_visit` | Numéro du tag + photo JPEG de la caméra. |
| À la fin | `/parc/finish` | `reason`. |

La définition exacte de certains messages ne nous a pas été transmise (type de
l'ordre, champ de la photo). Le code lit donc la définition installée sur le
robot au démarrage, au lieu de la deviner.

L'arbitre affiche ce qu'il reçoit sur `http://IP_DU_ROBOT:8080/`.

## 1.12 Le temps, la sûreté et la robustesse

- Le chronomètre démarre au départ, sur l'horloge ROS. À la fin des
  10 minutes (moins une marge de 5 s), la destination en cours est annulée,
  le robot s'arrête et la fin est signalée.
- La course tourne détachée de la session SSH (dans `tmux`) : une coupure
  Wi-Fi ne l'arrête pas.
- Les nœuds de perception redémarrent seuls s'ils plantent.
- Aucune valeur n'est codée en dur : tout se règle dans
  `caytu_config/config/fosa.yaml`, vérifié au démarrage (une valeur
  incohérente bloque le lancement avec un message clair).

## 1.13 Comment nous l'avons vérifié

- **188 tests automatiques**, sans robot ni simulateur : géométrie,
  alignement, stratégie, lecture de l'ordre, messages de l'arbitre,
  configuration, détection sur images synthétiques.
- **Simulation Gazebo** : le monde officiel PARC, avec quatre tags ajoutés, et
  des courses complètes de bout en bout. Un moniteur mesure chaque visite
  avec la vraie position du robot dans le simulateur.
- **Une mesure qui a compté** : l'odométrie du modèle Gazebo surestimait les
  rotations de 15 %, à cause d'un mauvais écartement des roues dans le
  modèle. Sur le vrai robot, `./run.sh mesure-rotation` compare l'odométrie,
  le gyroscope et le lidar.

## 1.14 Questions probables du jury

**Pourquoi ne pas aller droit au tag suivant ?** Un tag introuvable ou mal
placé peut coûter plus de points qu'il n'en rapporte. La stratégie compare
tous les plans possibles et ne saute un tag que si cela rapporte plus.

**Que se passe-t-il si un tag n'est jamais trouvé ?** Plus la recherche dure
au-delà du temps prévu, plus sa chance de réussite baisse. Dès qu'un plan
sans lui rapporte nettement plus, il est sauté.

**Comment évitez-vous les personnes ?** Le lidar et la profondeur des
caméras alimentent la carte de coût en continu. Nav2 replanifie deux fois par
seconde et ralentit près des obstacles, et la vitesse est plafonnée.

**Comment savez-vous que vous êtes à 0,5 m ?** La caméra mesure la position du
tag à partir de sa taille réelle connue, à chaque image. Le robot vise 2,5 cm
de précision, et la visite n'est validée qu'après plusieurs mesures stables.

**Et si la voix échoue ?** Le départ par commande reste toujours possible ; la
voix n'est qu'un bonus.

**Pourquoi OpenCV plutôt qu'un paquet AprilTag ROS ?** OpenCV est déjà sur le
robot, sans installation système (interdite sans les organisateurs), et nous
maîtrisons chaque réglage.

---

# Partie 2 — Ce que je dois faire

## 2.1 Où sont les choses

```text
fosa-solution/
├── README.md              ← ce fichier
├── run.sh                 ← lancer la course et les outils (sur le robot ou en simulation)
├── deployer_robot.sh      ← préparer, envoyer et installer la solution sur le robot
├── caytu_*/               ← les six paquets ROS 2
├── caytu_config/config/fosa.yaml   ← TOUS les réglages
├── outils/                ← diagnostic du robot, mesure de rotation
├── simulation/            ← Gazebo sur le PC uniquement (jamais envoyé au robot)
└── docs/parc-phase-finale-docs/    ← documents officiels des organisateurs
```

Le dépôt doit se trouver dans `<espace de travail>/src/fosa-solution`, par
exemple `~/ros2_ws/src/fosa-solution` sur le PC.

## 2.2 Tester en simulation sur mon PC

### Une seule fois : installer

Il faut, sur le PC :

- ROS 2 Jazzy, Nav2, slam_toolbox, `laser_filters` et Gazebo (`ros_gz`) :

  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-slam-toolbox \
      ros-jazzy-laser-filters ros-jazzy-ros-gz python3-opencv
  ```

- le dépôt de simulation PARC (`parc_robot_bringup`,
  `parc_robot_description`) dans `~/ros2_ws/src`, comme pour la phase 1.

Ensuite :

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build                                   # nos paquets + ceux de PARC
cd src/fosa-solution
simulation/simulation.sh preparer              # copie de parc_msgs + modèles des tags
```

`preparer` crée `~/.fosa_simulation/` : une copie de simulation du paquet de
l'arbitre (`parc_msgs` n'existe que sur le robot) et les modèles 3D des
quatre tags.

### Après chaque modification du code

```bash
cd ~/ros2_ws && colcon build --packages-up-to caytu_course
cd src/fosa-solution
```

Tests automatiques (une minute, sans simulateur) :

```bash
cd ~/ros2_ws
colcon test --packages-select caytu_config caytu_perception caytu_course caytu_nav_solution
colcon test-result --verbose          # attendu : 0 errors, 0 failures
```

### Lancer une course complète automatique

```bash
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh essai 7,1,9,4
```

Le script lance Gazebo avec les tags, puis la course. Il publie l'ordre,
donne le départ, attend la fin et affiche un résumé. Gazebo tourne environ
3 fois moins vite que le temps réel : comptez 30 à 45 minutes. Suivi en
direct :

```bash
tail -f ~/.fosa_simulation/journaux/moniteur.log
```

Dans le résumé, je regarde :

- `ARBITRE départ méthode 0` : le départ a été signalé ;
- `Plan : [...]` : les décisions de la stratégie ;
- `VISITE tag N : distance réelle 0.5xx m, erreur de cap ±x° -> VALIDE` : la
  visite, mesurée par la vraie position dans Gazebo ;
- `ARBITRE visite tag N : image jpeg (... octets)` : la photo est partie ;
- `TERMINEE` à la fin.

Journaux complets : `~/.fosa_simulation/journaux/` (`course.log`,
`moniteur.log`, `monde.log`).

### Lancer à la main, pas à pas

Dans quatre terminaux, depuis `~/ros2_ws/src/fosa-solution` :

```bash
# Terminal 1 : le monde, le robot et les tags
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh monde
# Terminal 2 : la course
FOSA_PROFIL=simulation FOSA_BASE_WS=~/.fosa_simulation/parc_ws ./run.sh course
# Terminal 3 : l'arbitre et les juges simulés
simulation/simulation.sh moniteur
# Terminal 4 : l'ordre puis le départ
export FOSA_PROFIL=simulation FOSA_BASE_WS=~/.fosa_simulation/parc_ws
./run.sh ordre-test "7,1,9,4"
./run.sh depart
```

`FOSA_SIM_GUI=1` devant `simulation/simulation.sh monde` affiche Gazebo (plus
lent). `FOSA_SIM_ODOM_CORRIGEE=1` corrige l'erreur d'odométrie du modèle
officiel (voir 1.13) : je le garde toujours.

En simulation, le profil `caytu_config/config/profils/simulation.yaml`
remplace automatiquement les topics du robot réel par ceux de Gazebo, sur le
domaine ROS 42. Le mot de départ vocal y est désactivé (pas de micro).

## 2.3 Tester sur le robot réel

### Ce qu'il faut savoir avant

- **Aucune adresse IP à modifier dans les fichiers.** Notre code tourne sur le
  robot lui-même et parle aux capteurs en local (domaine ROS 0). L'adresse
  IP ne sert qu'à se connecter en SSH et à copier le code ; on la donne en
  argument de commande.
- **Il me faut des organisateurs** : l'adresse IP du robot, le nom
  d'utilisateur SSH (et son mot de passe), et le dossier réservé à l'équipe
  sur le robot. Dans la suite : `UTILISATEUR`, `IP_DU_ROBOT`, `DOSSIER_EQUIPE`.
- **Interdit sur le robot** : `sudo`, installer des paquets système, publier
  sur `/parc/*` hors de notre créneau, publier sur l'odométrie du robot.
- **Le robot redémarre entre deux créneaux** : la solution reste installée
  dans notre dossier, mais il faut relancer la course à chaque créneau.

### Étape 1 — Sur le PC, avant d'aller au robot (une fois, avec Internet)

```bash
cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --preparer
```

Cette commande télécharge dans `livraison/` ce que le robot ne peut pas
télécharger seul : Vosk pour le processeur du robot (aarch64) et le modèle
vocal français. Elle est à refaire seulement si `livraison/` a été supprimé.

### Étape 2 — Envoyer la solution au robot (après chaque modification)

Le PC doit être sur le même réseau Wi-Fi que le robot.

```bash
./deployer_robot.sh --envoyer UTILISATEUR@IP_DU_ROBOT DOSSIER_EQUIPE
# exemple : ./deployer_robot.sh --envoyer parc@192.168.1.50 /home/parc/fosa
```

Le code est copié par SSH dans `DOSSIER_EQUIPE/ws/src/fosa-solution`, avec
seulement ce qui sert au robot : les paquets, `run.sh`,
`deployer_robot.sh`, `outils/`, `livraison/` et ce README. La simulation,
`docs/` et l'historique git restent sur le PC.

### Étape 3 — Installer sur le robot (après chaque envoi)

```bash
ssh UTILISATEUR@IP_DU_ROBOT
cd DOSSIER_EQUIPE/ws/src/fosa-solution
./deployer_robot.sh
```

Le script :

1. vérifie que Nav2, slam_toolbox et OpenCV sont installés ;
2. construit nos six paquets dans `DOSSIER_EQUIPE/ws` ;
3. installe Vosk et le modèle vocal dans notre dossier, sans `sudo`.

S'il annonce « Paquets ROS absents », je le signale aux organisateurs : nous
n'avons pas le droit de les installer nous-mêmes. Si la voix échoue, la
course reste possible avec le départ par commande.

### Étape 4 — Premier créneau : le diagnostic

```bash
./run.sh diagnostic-robot
```

Il vérifie environ une minute : les topics attendus, les caméras, le lidar,
les repères (TF), l'audio, et les définitions exactes des messages de
l'arbitre. Le rapport est enregistré dans `~/.ros/fosa/diagnostic_*.txt`.
Je le rapatrie sur le PC pour l'étudier au calme :

```bash
# sur le PC
scp UTILISATEUR@IP_DU_ROBOT:~/.ros/fosa/diagnostic_*.txt .
```

Si un nom de topic ou de caméra diffère, je le corrige dans
`caytu_config/config/fosa.yaml` sur le PC, puis je refais les étapes 2 et 3.

Avec `./run.sh diagnostic-robot --enregistrer 120`, il enregistre aussi
2 minutes de données capteurs (rosbag) pour régler la solution hors du robot.

### Étape 5 — Un créneau d'essai avec l'arbitre

1. **Terminal 1** (SSH) : lancer la solution.

   ```bash
   cd DOSSIER_EQUIPE/ws/src/fosa-solution
   ./run.sh course
   ```

   La course démarre dans `tmux`, détachée de la session SSH. `Ctrl-b` puis
   `d` quitte l'affichage sans arrêter la course. Si `run.sh` affiche
   « ATTENTION : parc_msgs introuvable », les visites ne seront pas
   comptées : prévenir un organisateur.

2. **Demander à un organisateur d'armer une course d'essai.** L'ordre des
   tags est alors publié, et le robot passe en `ATTENTE_DEPART`.

3. **Terminal 2** (SSH) : suivre l'état de l'arbitre.

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic echo /parc/run_status
   ```

4. **Donner le départ** : dire « allez » près du robot, ou taper
   `./run.sh depart` dans un troisième terminal.

5. **Suivre** : `./run.sh etat` affiche l'état de notre course. La page
   `http://IP_DU_ROBOT:8080/`, ouverte dans un navigateur du PC, montre ce
   que l'arbitre a reçu : départ, visites avec photo, fin.

6. **Arrêter** (essai seulement) : `./run.sh arreter`.

7. **Rapatrier les journaux** sur le PC :

   ```bash
   scp UTILISATEUR@IP_DU_ROBOT:~/.ros/fosa/course_*.log .
   ```

   Dans le journal, les lignes `Plan : ...` montrent les décisions et les
   durées estimées par la stratégie. Si elles sont loin de la réalité, je
   recale `course.strategy` dans `fosa.yaml` (vitesse réelle, temps de
   visite, temps de recherche).

Autres outils utiles pendant les essais :

| Commande | Usage |
|---|---|
| `./run.sh cartographie` | Tout sauf la course : pour voir la carte et régler. |
| `./run.sh sauver-carte` | Enregistrer la carte SLAM, utile seulement si les cartes enregistrées sont autorisées. |
| `./run.sh mesure-rotation` | Robot dégagé, course arrêtée : vérifier l'odométrie en rotation. |
| `./run.sh ordre-test "7,1,9,4"` | Publier un faux ordre. **Essais seulement**, jamais pendant une course notée. |

### Étape 6 — Le jour de la course notée

1. Avant le gel du code : soumettre la version finale, celle qui est installée
   sur le robot.
2. Au début de la course : `./run.sh course`, puis attendre que les
   organisateurs publient l'ordre.
3. Au signal : dire « allez » (bonus de 5 points), ou `./run.sh depart`.
4. **Ne plus rien toucher** : ni le robot, ni le clavier, ni la session SSH.
   Le robot s'arrête seul à la fin.

### Si quelque chose ne va pas

| Symptôme | Que faire |
|---|---|
| « Paquets ROS absents » au déploiement | Demander aux organisateurs d'installer Nav2 et slam_toolbox. |
| « parc_msgs introuvable » au lancement | L'espace de base `~/ros2_ws` du robot n'existe pas ou a changé : `FOSA_BASE_WS=<dossier> ./run.sh course`. |
| Rien sur la page de l'arbitre | Même cause que ci-dessus. |
| « No image » sur la page de l'arbitre | Le champ photo de `TagVisit` a un autre format : lire le rapport du diagnostic et adapter `caytu_course/caytu_course/arbitre.py`. |
| Aucun tag détecté | Topics des caméras dans `fosa.yaml`, éclairage, distance. |
| Le robot ne bouge pas après le départ | Lire le journal : Nav2 pas prêt, carte absente, topic `/cmd_vel` différent. |
| Le départ vocal ne marche pas | Vosk absent (relancer `./deployer_robot.sh`), format audio, seuil `min_confidence`. Départ par commande en attendant. |
| Le robot dérive en tournant | `./run.sh mesure-rotation`. |
| Tags sautés trop tôt ou trop tard | Recaler `course.strategy` dans `fosa.yaml` à partir des lignes `Plan : ...` du journal. |

### Points encore à confirmer sur place

Le diagnostic du premier créneau répond à ces questions :

- la valeur de `reason` attendue pour la fin de course ;
- le champ photo exact de `TagVisit` ;
- le format du son de `/speech_audio` ;
- quelle caméra est la caméra haute.

Il faut aussi demander aux organisateurs si une carte enregistrée pendant les
essais est autorisée le jour de la course.
