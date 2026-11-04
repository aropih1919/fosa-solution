# Équipe Fosa — PARC Engineers League 2026, phase réelle

Solution de course du robot **CAYTU Sito-É** (Jetson AGX Orin, ROS 2 Jazzy).

Ce fichier est le seul à lire. Il a deux parties :

- **Partie 1 — Comment marche notre solution** : toute la solution expliquée
  en phrases, sans lire le code. C'est aussi ce qu'il faut savoir pour
  répondre au jury.
- **Partie 2 — Ce que je dois faire** : six sections séparées, à lire selon
  la situation (préparer le PC, préparer le robot, simulation, essai réel,
  ranking match, problèmes).

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
    vérification se fait au mètre ruban (partie 2, section B.5).

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

Six situations, chacune dans sa section. Je lis seulement celle dont j'ai
besoin.

| Je veux… | Où | Section |
|---|---|---|
| installer ce qu'il faut sur mon PC (une fois) | PC | **A. Préparer mon PC** |
| installer la solution sur le robot | PC + robot | **B. Préparer le robot** |
| essayer la solution sans robot | PC | **C. Simulation sur mon PC** |
| m'entraîner sur le robot pendant un créneau d'essai | robot | **D. Essai réel sur le robot** |
| faire le match noté devant le jury | robot | **E. Le ranking match** |
| comprendre un problème | — | **F. Si quelque chose ne va pas** |

Où sont les fichiers :

```text
fosa-solution/
├── README.md              ← ce fichier
├── run.sh                 ← toutes les commandes de la course (robot et simulation)
├── deployer_robot.sh      ← envoyer et installer la solution sur le robot
├── caytu_*/               ← les six paquets ROS 2
├── caytu_config/config/fosa.yaml   ← TOUS les réglages
├── outils/                ← diagnostic du robot, mesure de rotation
├── simulation/            ← Gazebo, sur le PC uniquement (jamais envoyé au robot)
└── docs/parc-phase-finale-docs/    ← documents officiels des organisateurs
```

---

## A. Préparer mon PC (une seule fois)

But : pouvoir construire la solution, lancer les tests et la simulation.

1. **Installer ROS 2 Jazzy, Nav2, slam_toolbox et Gazebo** :

   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-slam-toolbox \
       ros-jazzy-laser-filters ros-jazzy-ros-gz python3-opencv
   ```

2. **Placer les dépôts** dans `~/ros2_ws/src` : notre dépôt
   (`fosa-solution`) et le dépôt de simulation PARC
   (`parc_robot_bringup`, `parc_robot_description`), comme pour la phase 1.

3. **Construire** :

   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build
   ```

4. **Préparer la simulation** (copie de `parc_msgs`, modèles des tags) :

   ```bash
   cd ~/ros2_ws/src/fosa-solution
   simulation/simulation.sh preparer
   ```

5. **Vérifier avec les tests** (1 minute, sans simulateur) :

   ```bash
   cd ~/ros2_ws
   colcon test --packages-select caytu_config caytu_perception caytu_course caytu_nav_solution
   colcon test-result --verbose          # attendu : 185 tests, 0 errors, 0 failures
   ```

**Après chaque modification du code** : `cd ~/ros2_ws && colcon build
--packages-up-to caytu_course`, puis relancer les tests.

À ne pas faire sur le PC :

- **ne jamais lancer `./deployer_robot.sh` sans option** : il est fait pour le
  robot. Il refuse de toute façon de s'exécuter sur le PC ;
- **ne jamais lancer `./run.sh ...` sans `FOSA_PROFIL=simulation`** : sans lui,
  c'est le profil du robot réel (domaine ROS 0), qui peut perturber le robot
  si le PC est sur son Wi-Fi.

---

## B. Préparer le robot (installation de la solution)

But : copier la solution sur le robot et l'installer. Aucune adresse IP n'est
à écrire dans les fichiers : notre code tourne sur le robot lui-même. L'IP
sert seulement à s'y connecter, et on la tape dans la commande.

### B.1 Ce que je demande aux organisateurs

- l'**adresse IP du robot** ;
- le **nom d'utilisateur SSH** et son mot de passe ;
- le **dossier réservé à l'équipe** sur le robot.

Dans la suite : `UTILISATEUR`, `IP_DU_ROBOT`, `DOSSIER_EQUIPE`. Exemple :
`parc`, `192.168.1.50`, `/home/parc/fosa`.

Interdit sur le robot : `sudo`, installer des paquets système, publier sur
`/parc/*` en dehors de notre créneau, publier sur l'odométrie du robot.

### B.2 Sur le PC : télécharger la voix (une fois, avec Internet)

```bash
cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --preparer
```

Télécharge dans `livraison/` Vosk pour le processeur du robot et le modèle
vocal français : le robot n'a pas forcément Internet.

### B.3 Sur le PC : envoyer la solution (après chaque modification)

Le PC doit être sur le même Wi-Fi que le robot.

```bash
./deployer_robot.sh --envoyer UTILISATEUR@IP_DU_ROBOT DOSSIER_EQUIPE
```

Seul ce qui sert au robot est copié, dans
`DOSSIER_EQUIPE/ws/src/fosa-solution` : les paquets, `run.sh`,
`deployer_robot.sh`, `outils/`, `livraison/` et ce README.

### B.4 Sur le robot : installer (après chaque envoi)

```bash
ssh UTILISATEUR@IP_DU_ROBOT
cd DOSSIER_EQUIPE/ws/src/fosa-solution
./deployer_robot.sh
```

Le script vérifie que Nav2, slam_toolbox et OpenCV sont présents, construit
nos six paquets, et installe la voix dans notre dossier, sans `sudo`.

- « Paquets ROS absents » : le signaler aux organisateurs (nous n'avons pas le
  droit de les installer).
- Échec de la voix : pas grave, le départ par commande reste possible.

### B.5 Sur le robot, au premier créneau : vérifier le robot

**Le diagnostic** (1 minute) :

```bash
./run.sh diagnostic-robot
```

Il vérifie les topics, les caméras, le lidar, les repères, l'audio, et note la
définition exacte des messages de l'arbitre. Je rapatrie le rapport sur le
PC :

```bash
# sur le PC
scp UTILISATEUR@IP_DU_ROBOT:'~/.ros/fosa/diagnostic_*.txt' .
```

Si un nom de topic ou de caméra diffère, je le corrige dans
`caytu_config/config/fosa.yaml` sur le PC, puis je refais B.3 et B.4.

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

## C. Simulation sur mon PC

But : voir toute la solution tourner dans Gazebo, sans robot. Le PC doit avoir
été préparé (section A).

### C.1 La répétition complète, automatique

```bash
cd ~/ros2_ws/src/fosa-solution
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh essai 7,1,9,4
```

- **Une fenêtre Gazebo s'ouvre** : on voit le robot rouler, tourner sur
  lui-même et s'arrêter devant les tags.
- **Le terminal raconte tout**, en phrases (voir 1.14), plus ce que reçoit
  l'arbitre et le verdict de « juges simulés », qui mesurent la vraie distance
  et le vrai angle dans Gazebo.

Le script enchaîne deux phases, comme le jour J :

1. **la préparation** : le robot explore, construit la carte, mesure les tags
   et enregistre tout ;
2. **le match** : dans un monde neuf, robot à la position de départ, l'ordre
   7, 1, 9, 4 est publié, le départ est donné, et le robot court sur la carte
   enregistrée.

Comptez environ une heure (Gazebo tourne environ 3 fois moins vite que le
temps réel). Ajouter `FOSA_SIM_GUI=0` devant la commande masque Gazebo et va
plus vite.

Ce que je regarde dans le terminal :

- `=== FIN ===` puis une ligne par tag : visité, ou **pourquoi** il ne l'a pas
  été ;
- `[juge simulé] VISITE tag N mesurée dans Gazebo : centre à 0.5xx m, erreur
  de cap ±x° -> VALIDE` ;
- `[juge simulé] arbitre : visite du tag N reçue, photo jpeg (... octets)`.

### C.2 Une seule phase

```bash
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh preparation     # carte + tags
FOSA_SIM_ODOM_CORRIGEE=1 simulation/simulation.sh course 9,4,1,7  # match seul
```

Le match seul utilise la dernière préparation de simulation, rangée dans
`~/.fosa_simulation/preparation` (jamais mélangée avec celle du robot). Pour
tester un match sans préparation (carte en direct), supprimer ce dossier.

### C.3 Tout arrêter

Après un Ctrl-C, un terminal fermé, ou le message « Une course ou Gazebo
tourne déjà » :

```bash
simulation/simulation.sh arreter
```

Ne jamais lancer deux simulations à la fois : elles piloteraient le même
robot simulé et les journaux se mélangeraient.

### C.4 À la main, terminal par terminal (pour déboguer)

```bash
# Terminal 1 : le monde, le robot et les tags
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

Journaux : `~/.fosa_simulation/journaux/` (`course_*.log` : détail
technique ; `moniteur_*.log` : le récit).

---

## D. Essai réel sur le robot (créneaux d'essai, avant le ranking match)

But : répéter exactement le ranking match, sans points en jeu. Un créneau se
fait en deux temps : **D.1 enregistrer la carte et les tags**, puis **D.2 la
mission sur cette carte**. Le robot doit avoir été préparé (section B).

Avant de commencer : ouvrir **deux terminaux SSH** sur le robot, et dans
chacun :

```bash
ssh UTILISATEUR@IP_DU_ROBOT
cd DOSSIER_EQUIPE/ws/src/fosa-solution
```

### D.1 Enregistrer la carte et les tags (préparation)

1. Poser le robot à la **position de départ** du match, arène dégagée.
2. **Terminal 1** : `./run.sh preparation`.
   Le robot explore seul. `Ctrl-b` puis `d` quitte l'affichage sans
   l'arrêter (il tourne dans `tmux`).
3. **Terminal 2** : `./run.sh suivi`. Il raconte : tags repérés, tags mesurés
   de près, carte enregistrée.
4. Attendre `=== FIN ===` (12 minutes au plus). Le suivi indique où sont
   la carte et les tags, et quels tags manquent éventuellement.
5. **Terminal 1** : `./run.sh arreter`.

### D.2 La mission sur la carte enregistrée

1. **Reposer le robot à la position de départ** : la carte a été faite à
   partir de là.
2. **Terminal 1** : `./run.sh course`.
   Je vérifie qu'il annonce « Préparation utilisée : ... ». S'il affiche
   « ATTENTION : parc_msgs introuvable », les visites ne seront pas comptées :
   prévenir un organisateur.
3. **Demander à un organisateur d'armer une course d'essai** : l'ordre des
   tags est publié.
4. **Terminal 2** : `./run.sh suivi`.
5. **Donner le départ** : dire « allez » près du robot, ou `./run.sh depart`
   dans un troisième terminal.
6. **Regarder** le récit, et la page `http://IP_DU_ROBOT:8080/` dans un
   navigateur du PC : ce que l'arbitre a reçu (départ, visites avec photo,
   fin).
7. **Arrêter** : `./run.sh arreter`.

### D.3 Après le créneau, sur le PC

Rapatrier les journaux et la carte pour les étudier :

```bash
scp UTILISATEUR@IP_DU_ROBOT:'~/.ros/fosa/*.log' .
scp -r UTILISATEUR@IP_DU_ROBOT:'~/.ros/fosa/preparation' .   # carte.pgm à regarder
```

Si les durées annoncées par les plans sont loin de la réalité, je recale
`course.strategy` dans `fosa.yaml`, puis je refais B.3 et B.4 au créneau
suivant.

Outils utiles pendant un créneau :

| Commande | Usage |
|---|---|
| `./run.sh cartographie` | Caméras et carte sans mission : pour voir et régler. |
| `./run.sh sauver-carte` | Enregistrer tout de suite la carte dans la préparation. |
| `./run.sh etat` | État brut de la course. |
| `./run.sh ordre-test "7,1,9,4"` | Publier un faux ordre si aucun organisateur ne peut armer. **Essais seulement.** |

---

## E. Le ranking match (le match noté devant le jury)

Même déroulement que D, mais pour de vrai : les points comptent et personne
ne touche le robot après le départ.

**Avant le jour J**

- Au gel du code : la version installée sur le robot est la version soumise.
  Ne plus rien modifier ensuite.

**Les 15 minutes de préparation**

1. Robot à la **position de départ**, arène telle qu'elle sera pour le match.
2. **Terminal 1** : `./run.sh preparation`. **Terminal 2** : `./run.sh suivi`.
3. Attendre `=== FIN ===`, vérifier « Carte enregistrée » et la liste des
   tags trouvés.
4. Si la carte n'est pas enregistrée, ou s'il manque des tags et qu'il reste
   du temps : relancer `./run.sh preparation`.
5. `./run.sh arreter`.

**Le match (10 minutes)**

1. Reposer le robot à la **position de départ**.
2. **Terminal 1** : `./run.sh course`, vérifier « Préparation utilisée ».
3. **Terminal 2** : `./run.sh suivi` (peut rester ouvert pour regarder).
4. Attendre que les organisateurs publient l'ordre.
5. Au signal : dire **« allez »** (bonus de 5 points), ou `./run.sh depart`
   si la voix échoue.
6. **Ne plus rien toucher** : ni le robot, ni le clavier, ni la session SSH.
   Le robot s'arrête seul et signale la fin à l'arbitre.

**Ce qu'on ne fait jamais en match** : `./run.sh ordre-test`, `./run.sh
arreter`, toucher le robot, modifier un fichier.

---

## F. Si quelque chose ne va pas

| Symptôme | Que faire |
|---|---|
| « failed to create symbolic link » au build sur le PC | Supprimer `~/ros2_ws/build/caytu_*` et `~/ros2_ws/install/caytu_*`, puis `colcon build`. |
| « Une course ou Gazebo tourne déjà » (simulation) | `simulation/simulation.sh arreter`. |
| « Paquets ROS absents » au déploiement | Demander aux organisateurs d'installer Nav2 et slam_toolbox. |
| « parc_msgs introuvable » au lancement | L'espace de base `~/ros2_ws` du robot a changé : `FOSA_BASE_WS=<dossier> ./run.sh course`. |
| Rien sur la page de l'arbitre | Même cause que ci-dessus. |
| « No image » sur la page de l'arbitre | Le champ photo de `TagVisit` a un autre format : lire le rapport du diagnostic et adapter `caytu_course/caytu_course/arbitre.py`. |
| « Aucune préparation » au lancement du match | La préparation n'a pas enregistré la carte : relancer `./run.sh preparation`. |
| Le robot se croit ailleurs au début du match | Il n'est pas reparti de la position de départ de la préparation : le reposer et relancer `./run.sh course`. |
| Aucun tag détecté | Topics des caméras dans `fosa.yaml`, éclairage, distance. |
| Le robot ne bouge pas après le départ | Lire le journal : Nav2 pas prêt, carte absente, topic `/cmd_vel` différent. |
| Le départ vocal ne marche pas | Vosk absent (refaire B.4), format audio, seuil `min_confidence`. Départ par commande en attendant. |
| Le robot dérive en tournant | `./run.sh mesure-rotation`. |
| Tags sautés trop tôt ou trop tard | Recaler `course.strategy` dans `fosa.yaml` à partir des plans du récit. |

**Encore à confirmer sur le robot** (le diagnostic B.5 y répond) : la valeur de
`reason` pour la fin de course, le champ photo exact de `TagVisit`, le format
du son de `/speech_audio`, et quelle caméra est la caméra haute.
