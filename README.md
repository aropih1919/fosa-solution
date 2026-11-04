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

Juste avant le match, les organisateurs accordent **15 minutes** pour
cartographier l'arène et repérer les tags : le match de 10 minutes peut
ensuite utiliser ces données. Le mobilier peut changer d'un jour à l'autre :
il faut donc refaire la carte à chaque fois, pendant ces 15 minutes.

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

Le jour J se déroule en deux temps, une commande chacun :

1. `./run.sh preparation` (pendant les 15 minutes) : le robot explore seul,
   construit la carte, va mesurer chaque tag de près, et enregistre la carte
   et la position des tags (voir 1.6) ;
2. `./run.sh course` (le match) : le robot se repère sur cette carte, connaît
   déjà la position des tags, et les visite dans l'ordre imposé.

`./run.sh course` lance tout en même temps :

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

## 1.3 Savoir où l'on est : la carte (SLAM)

**slam_toolbox** construit la carte à partir du lidar. Le lidar mesure la
distance aux murs et aux meubles tout autour du robot. En comparant chaque
nouvelle mesure à la carte déjà construite, slam_toolbox fait deux choses à
la fois :

- il **agrandit la carte** au fur et à mesure que le robot découvre l'arène ;
- il **corrige la position du robot** sur cette carte. L'odométrie des roues
  dérive avec le temps ; le SLAM la recale sur les murs.

La carte est une grille de cases de 5 cm : libre, occupée ou inconnue. Elle
sert à la fois à Nav2 (pour planifier) et à notre recherche de tags.

- **Pendant la préparation**, slam_toolbox construit une carte neuve et elle
  est enregistrée à la fin.
- **Pendant le match**, slam_toolbox recharge cette carte et ne fait plus que
  s'y **localiser** : il retrouve la position du robot en comparant le lidar
  à la carte enregistrée. Pour cela, le match doit partir de l'endroit où la
  préparation a commencé (la position de départ officielle) : c'est l'origine
  de la carte.
- **Sans préparation** (par exemple si elle a échoué), le match construit la
  carte en direct, comme un robot qui découvre l'arène.

Si des meubles ont bougé depuis la préparation, le lidar les voit quand même :
ils apparaissent dans les cartes de coût de Nav2 (voir 1.4) et le robot les
contourne.

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

## 1.6 La préparation : carte et tags enregistrés avant le match

Pendant les 15 minutes de préparation, `./run.sh preparation` fait tout seul :

1. il met de côté la préparation précédente (renommée avec la date) et
   repart de zéro : la carte de la veille peut être fausse ;
2. il explore l'arène avec la recherche décrite en 1.8 : tours sur place et
   points libres de la carte, jusqu'à l'avoir entièrement parcourue ;
3. chaque tag aperçu de loin (à plus de 1,3 m) est mesuré de près : le robot
   va se placer à 0,9 m devant lui, car une vue proche est bien plus précise ;
4. il s'arrête quand toute l'arène est parcourue et les 4 tags mesurés, ou au
   bout de 12 minutes (il reste 3 minutes de marge) ;
5. il enregistre la carte et la position des tags dans
   `~/.ros/fosa/preparation/` : `carte.posegraph` et `carte.data` pour le
   match, `carte.pgm` pour la voir, `tags.yaml` pour les tags.

Les tags sont réenregistrés à chaque étape : si la préparation est arrêtée
avant la fin, ce qui a été trouvé n'est pas perdu, et la carte est enregistrée
à l'arrêt.

## 1.7 Se souvenir des tags

Chaque fois qu'un tag est vu, même si ce n'est pas celui qu'on cherche, sa
position est mémorisée sur la carte. Les vues de près comptent plus que les
vues de loin, car une vue lointaine est imprécise : en simulation, une vue à
1,5 m pouvait décaler la position de 60 cm. Une vue nettement plus proche
remplace les anciennes.

Au début du match, la mémoire contient déjà les tags de la préparation : le
robot va droit vers eux. Pendant le match, il continue de mémoriser ce qu'il
voit : en cherchant le tag 7, il apprend souvent où sont le 9 ou le 4. Si un
tag n'est pas à sa position mémorisée (déplacé, mal mesuré), il est oublié et
cherché à nouveau.

## 1.8 Chercher un tag qu'on n'a jamais vu

Si la cible n'est pas en mémoire, le robot :

1. fait un tour lent sur lui-même, pour que les caméras balaient les murs ;
2. va vers le point libre de la carte le plus proche qu'il n'a pas encore
   visité, loin des obstacles ;
3. recommence.

Dès que la cible apparaît sur une image, la recherche s'arrête.

## 1.9 Se placer à 0,5 m face au tag

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
   - une fois au point (à 1,2 cm près), tourne sur place pour faire face au
     tag (à 2° près).

   Pendant ces manœuvres, une garde lit le lidar et bloque l'avance ou le
   recul si un obstacle est trop près.

Nous visons 1,2 cm et 2°, bien plus strict que les ±12° du règlement, pour
garder de la marge face aux juges. La visite n'est validée qu'après plusieurs
mesures stables d'affilée. Le robot envoie alors le numéro et la photo à
l'arbitre, et garde la position.

Si le tag est vu trop de biais ou de trop loin, l'asservissement rend la main
à Nav2, qui ramène le robot à 0,9 m, bien en face.

## 1.10 La stratégie : quels tags tenter, et quand en sauter un

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

## 1.11 Le départ : par commande ou à la voix

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

## 1.12 Parler à l'arbitre

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

## 1.13 Le temps, la sûreté et la robustesse

- Le chronomètre démarre au départ, sur l'horloge ROS. À la fin des
  10 minutes (moins une marge de 5 s), la destination en cours est annulée,
  le robot s'arrête et la fin est signalée.
- La course tourne détachée de la session SSH (dans `tmux`) : une coupure
  Wi-Fi ne l'arrête pas.
- Les nœuds de perception redémarrent seuls s'ils plantent.
- Aucune valeur n'est codée en dur : tout se règle dans
  `caytu_config/config/fosa.yaml`, vérifié au démarrage (une valeur
  incohérente bloque le lancement avec un message clair).

## 1.14 Comprendre ce que fait le robot : le récit en direct

Chaque décision du robot est publiée en phrase française, et `./run.sh suivi`
l'affiche dans un terminal, avec le temps écoulé et le temps restant. Exemple :

```text
[00:00 | reste 09:55] Carte de la préparation chargée ; tags déjà localisés : 1, 4, 7, 9.
[00:00 | reste 09:55] Départ donné à la voix (« allez ») : chronomètre lancé.
[00:00 | reste 09:55] Ordre des tags reçu de l'arbitre : 7 → 1 → 9 → 4.
[00:01 | reste 09:54] Nouvelle cible : tag 7, déjà localisé (vu à 0.9 m).
[00:01 | reste 09:54] Plan : 7 → 1 → 9 → 4, environ 290 s de travail pour 594 s restantes (90 points espérés).
[00:24 | reste 09:31] Tag 7 VISITÉ (1/4) : centre à 0.50 m, face au tag à 1.2° près. Photo envoyée à l'arbitre.
[03:12 | reste 06:43] Visite du tag 1 ratée (1e échec) : Nav2 n'a pas pu atteindre le point d'approche (obstacle ou chemin bloqué). On réessaie.
[08:20 | reste 01:35] Tag 9 sauté : il faudrait encore environ 160 s pour l'atteindre et le visiter, et il reste 95 s. ...
[10:00 | reste 00:00] === FIN ===
    Tag 7 : visité (1e visite).
    Tag 9 : non visité — sauté par la stratégie (≈ 160 s nécessaires, 95 s restantes ; [4] rapportait plus).
    Bilan : ...
```

Le récit couvre : les données chargées, le départ, l'ordre, chaque tag repéré
(même s'il n'est pas la cible), chaque nouveau plan, chaque saut avec ses
chiffres, chaque échec avec sa raison, chaque visite avec la distance et
l'angle mesurés, et à la fin, pour chaque tag non visité, **pourquoi**.

## 1.15 Comment nous l'avons vérifié

- **185 tests automatiques**, sans robot ni simulateur : géométrie,
  alignement, stratégie, lecture de l'ordre, messages de l'arbitre,
  configuration, détection sur images synthétiques.
- **Simulation Gazebo** : le monde officiel PARC, avec quatre tags ajoutés, et
  des courses complètes de bout en bout. Un moniteur mesure chaque visite
  avec la vraie position du robot dans le simulateur.
- **Résultat** : en simulation, les quatre tags ont été visités dans l'ordre
  7, 1, 9, 4 en 370 s sur 600, chacun signalé à l'arbitre avec sa photo, face
  au tag à moins de 1,5°. Une ancienne course tournait peut-être en même temps
  sur le même robot simulé : ce chiffre est à reconfirmer avec
  `simulation/simulation.sh essai`.
- **Deux mesures qui ont compté :**
  - l'odométrie du modèle Gazebo surestimait les rotations de 15 %, à cause
    d'un mauvais écartement des roues dans le modèle. Sur le vrai robot,
    `./run.sh mesure-rotation` compare l'odométrie, le gyroscope et le lidar ;
  - dans le rendu Gazebo, le tag paraît 3,8 % plus grand qu'en réalité, et le
    robot s'arrêtait à 0,55 m au lieu de 0,50. Nous l'avons mesuré avec un
    tag posé à distance connue, puis corrigé. Sur le vrai robot, la même
    vérification se fait au mètre ruban (voir 2.3, étape 4).

## 1.16 Questions probables du jury

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
tag à partir de sa taille réelle connue, à chaque image. Le robot vise 1,2 cm
de précision, et la visite n'est validée qu'après plusieurs mesures stables.

**Pourquoi une préparation, et pas une carte faite à l'avance ?** Le mobilier
peut changer entre les essais et le match. La préparation refait la carte et
remesure les tags juste avant le match, dans les 15 minutes accordées. Si
elle échoue, le match sait aussi cartographier et chercher en direct.

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

**Important** : construire toujours avec `colcon build` simple sur le PC.
Ne pas lancer `./deployer_robot.sh` sur le PC : il construit autrement
(`--symlink-install`) et le mélange des deux casse le build. En cas d'erreur
« failed to create symbolic link », supprimer `~/ros2_ws/build/caytu_*` et
`~/ros2_ws/install/caytu_*`, puis reconstruire.

### Répéter le jour J : préparation puis match

```bash
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh essai 7,1,9,4
```

- **La fenêtre Gazebo s'ouvre** : on voit le robot rouler, tourner sur
  lui-même et s'arrêter devant les tags.
- **Le terminal affiche le récit**, en phrases (voir 1.14), plus ce que reçoit
  l'arbitre et le verdict des « juges simulés » : la vraie distance et le
  vrai angle mesurés dans Gazebo.

Le script enchaîne deux phases, chacune dans un monde neuf, robot à la
position de départ :

1. **la préparation** : exploration, carte, tags mesurés et enregistrés ;
2. **le match** : publie l'ordre 7, 1, 9, 4, donne le départ, et le robot
   court sur la carte de la préparation.

Il s'arrête de lui-même à la fin de chaque phase. Gazebo tourne environ
3 fois moins vite que le temps réel : comptez environ une heure pour les
deux. `FOSA_SIM_GUI=0` devant la commande masque Gazebo et va plus vite.

Une phase seule :

```bash
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh preparation     # carte + tags
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh course 9,4,1,7  # match (avec la préparation si elle existe)
```

Pour tester le match sans préparation (carte en direct), supprimer d'abord
`~/.fosa_simulation/preparation`. (En simulation, la préparation est rangée
là, et pas dans `~/.ros/fosa`.)

Ce que je regarde dans le terminal :

- `=== FIN ===` puis une ligne par tag : visité, ou **pourquoi** il ne l'a pas
  été ;
- `[juge simulé] VISITE tag N mesurée dans Gazebo : centre à 0.5xx m, erreur
  de cap ±x° -> VALIDE` ;
- `[juge simulé] arbitre : visite du tag N reçue, photo jpeg (... octets)`.

Journaux : `~/.fosa_simulation/journaux/` (`course_*.log` : tout le détail
technique ; `moniteur_*.log` : le récit).

### Tout arrêter

Si un essai est interrompu (Ctrl-C, terminal fermé) ou si le script répond
« Une course ou Gazebo tourne déjà » :

```bash
simulation/simulation.sh arreter
```

Ne jamais lancer deux essais à la fois : deux courses piloteraient le même
robot simulé et les journaux se mélangeraient.

### Lancer à la main, pas à pas

Dans quatre terminaux, depuis `~/ros2_ws/src/fosa-solution` :

```bash
# Terminal 1 : le monde, le robot et les tags (fenêtre Gazebo)
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh monde
# Terminal 2 : la préparation OU le match
export FOSA_PROFIL=simulation FOSA_BASE_WS=~/.fosa_simulation/parc_ws
./run.sh preparation              # ou : ./run.sh course
# Terminal 3 : le récit et les juges simulés
simulation/simulation.sh moniteur
# Terminal 4 (match seulement) : l'ordre puis le départ
export FOSA_PROFIL=simulation FOSA_BASE_WS=~/.fosa_simulation/parc_ws
./run.sh ordre-test "7,1,9,4"
./run.sh depart
```

Le profil `caytu_config/config/profils/simulation.yaml` remplace
automatiquement les topics du robot réel par ceux de Gazebo, sur le domaine
ROS 42. Le mot de départ vocal y est désactivé (pas de micro).
**Sans `FOSA_PROFIL=simulation`, `run.sh` utilise le profil du robot réel
(domaine ROS 0)** : sur le Wi-Fi du robot, cela peut le perturber.

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

**Vérifier la mesure de distance (5 minutes, très important).** La visite
se joue à quelques centimètres près. Une erreur d'échelle de la caméra ou un
tag imprimé un peu plus grand ou plus petit que 16,5 cm décale le point
d'arrêt. En simulation, nous avons mesuré une telle erreur de 3 cm, que nous
avons corrigée.

1. Lancer `./run.sh cartographie`.
2. Poser un tag face à une caméra, à exactement 1,00 m de l'objectif, au
   mètre ruban.
3. Dans un autre terminal :
   `source ~/ros2_ws/install/setup.bash && ros2 topic echo /perception/tags --field detections`.
4. Lire `distance_m`. Si l'écart dépasse 2 cm, corriger dans `fosa.yaml` :
   `size_m = 0,165 × 1,00 ÷ distance lue` (exemple : 0,97 lu → `size_m: 0.170`).

### Étape 5 — Un créneau d'essai : préparation puis match

C'est la répétition exacte du jour J.

**A. La préparation (le robot explore seul)**

1. Poser le robot à la **position de départ** du match, arène dégagée.
2. **Terminal 1** (SSH) :

   ```bash
   cd DOSSIER_EQUIPE/ws/src/fosa-solution
   ./run.sh preparation
   ```

   La préparation démarre dans `tmux`, détachée de la session SSH :
   `Ctrl-b` puis `d` quitte l'affichage sans l'arrêter.
3. **Terminal 2** (SSH) : `./run.sh suivi` raconte tout en direct : tags
   repérés, tags mesurés de près, carte enregistrée.
4. Attendre `=== FIN ===` dans le suivi (12 minutes au plus). Il indique où
   sont enregistrées la carte et les tags, et quels tags manquent
   éventuellement.
5. `./run.sh arreter`, puis **reposer le robot à la position de départ**. Le
   match se repère sur la carte à partir de cette position.

**B. Le match**

1. **Terminal 1** : `./run.sh course`. Il annonce « Préparation utilisée :
   ... ». Si `run.sh` affiche « ATTENTION : parc_msgs introuvable », les
   visites ne seront pas comptées : prévenir un organisateur.
2. **Demander à un organisateur d'armer une course d'essai.** L'ordre des
   tags est alors publié.
3. **Terminal 2** : `./run.sh suivi` (le récit, voir 1.14). Éventuellement,
   un terminal 3 pour l'arbitre :

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic echo /parc/run_status
   ```

4. **Donner le départ** : dire « allez » près du robot, ou taper
   `./run.sh depart` dans un autre terminal.
5. **Suivre** dans le récit. La page `http://IP_DU_ROBOT:8080/`, ouverte dans
   un navigateur du PC, montre ce que l'arbitre a reçu : départ, visites avec
   photo, fin.
6. **Arrêter** (essai seulement) : `./run.sh arreter`.

**C. Rapatrier les journaux** sur le PC pour les étudier :

```bash
scp UTILISATEUR@IP_DU_ROBOT:'~/.ros/fosa/*.log' .
scp -r UTILISATEUR@IP_DU_ROBOT:'~/.ros/fosa/preparation' .   # carte.pgm à regarder
```

Si les durées annoncées par les plans sont loin de la réalité, je recale
`course.strategy` dans `fosa.yaml` (vitesse réelle, temps de visite, temps de
recherche).

Autres outils utiles pendant les essais :

| Commande | Usage |
|---|---|
| `./run.sh cartographie` | SLAM, Nav2 et caméras sans mission : pour voir et régler. |
| `./run.sh sauver-carte` | Enregistrer tout de suite la carte dans la préparation. |
| `./run.sh mesure-rotation` | Robot dégagé, rien d'autre lancé : vérifier l'odométrie en rotation. |
| `./run.sh ordre-test "7,1,9,4"` | Publier un faux ordre. **Essais seulement**, jamais pendant une course notée. |

### Étape 6 — Le jour de la course notée

1. Avant le gel du code : soumettre la version finale, celle qui est installée
   sur le robot.
2. **Les 15 minutes de préparation** : robot à la position de départ,
   `./run.sh preparation`, `./run.sh suivi`, attendre `=== FIN ===`, puis
   `./run.sh arreter`. Si la préparation a raté (carte non enregistrée), la
   relancer tant qu'il reste du temps.
3. Reposer le robot à la position de départ, puis `./run.sh course` et
   attendre que les organisateurs publient l'ordre.
4. Au signal : dire « allez » (bonus de 5 points), ou `./run.sh depart`.
5. **Ne plus rien toucher** : ni le robot, ni le clavier, ni la session SSH.
   Le robot s'arrête seul à la fin. `./run.sh suivi` peut rester ouvert pour
   regarder.

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
| Tags sautés trop tôt ou trop tard | Recaler `course.strategy` dans `fosa.yaml` à partir des plans du récit. |
| « Aucune préparation » au lancement du match | La préparation n'a pas enregistré la carte : relancer `./run.sh preparation`. |
| Le robot se croit ailleurs au début du match | Il n'est pas reparti de la position de départ de la préparation : le reposer et relancer `./run.sh course`. |

### Points encore à confirmer sur place

Le diagnostic du premier créneau répond à ces questions :

- la valeur de `reason` attendue pour la fin de course ;
- le champ photo exact de `TagVisit` ;
- le format du son de `/speech_audio` ;
- quelle caméra est la caméra haute.

Confirmé par les organisateurs : 15 minutes de préparation (carte et tags)
avant chaque match, avec les données utilisables pendant le match.
