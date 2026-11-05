# Comprendre la solution Fosa

Ce document explique l’architecture, les décisions de conception et le fonctionnement interne de la solution CAYTU Sito-É pour la PARC Engineers League 2026. Il s’adresse à l’équipe et au jury.

Pour les procédures pas à pas et les commandes à exécuter, utiliser le [guide d’utilisation](../README.md).

---

## 1.1 Ce que le robot doit faire

Le jour de la course, les organisateurs publient un ordre tiré au sort pour
les quatre AprilTags 1, 4, 7 et 9, par exemple 7, 1, 9, 4. Le robot ne
connaît ni la carte de l'arène (5 m × 5 m, des tables, des chaises, parfois
des personnes) ni la position des tags.

Juste avant le match, les organisateurs accordent **15 minutes de préparation**
(consigne communiquée oralement à l'équipe) pour cartographier l'arène et
repérer les tags. Le robot peut ensuite utiliser ces données pendant le match
de 10 minutes. Le mobilier peut changer : refaire la carte pendant chaque
préparation.

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

Le jour J se déroule en deux temps :

1. **Préparation (15 minutes)** : on peut guider le robot pour choisir un
   parcours efficace. SLAM construit la carte et les caméras enregistrent les
   tags vus. `./run.sh preparation-manuelle` enregistre ces données sans
   envoyer de buts de navigation. `./run.sh preparation` offre l'autre choix :
   le robot explore seul.
2. **Match (10 minutes au plus)** : `./run.sh course` charge la carte et les
   tags enregistrés, attend l'ordre puis le départ. À partir du départ, le
   robot agit seul et l'équipe ne le touche plus.

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

Pendant les 15 minutes de préparation, deux modes sont possibles :

- `./run.sh preparation-manuelle` : l'équipe guide le robot avec le moyen de
  pilotage autorisé. Le nœud ne commande aucun déplacement; SLAM fait la carte
  et les caméras mémorisent automatiquement les tags détectés.
- `./run.sh preparation` : le robot explore et approche les tags de façon
  autonome. Utiliser ce mode pour vérifier qu'il sait refaire le parcours
  sans aide.

Dans les deux modes, les données sont sauvegardées dans le même dossier; au
début du match, `./run.sh course` les charge automatiquement. En guidage
manuel, les tags sont écrits pendant que le nœud tourne. Avant d'arrêter la
préparation, lancer `./run.sh sauver-carte` dans un autre terminal : cette
commande enregistre la carte SLAM active et affiche les tags mémorisés. Ensuite
seulement, arrêter avec `./run.sh arreter`. Pour éviter deux sources de
commandes, ne jamais lancer les deux modes en même temps. À la limite de
12 minutes, le nœud enregistre lui-même avant de quitter.

Le mode autonome suit ce déroulement :

1. il met de côté la préparation précédente (renommée avec la date) et
   repart de zéro : la carte de la veille peut être fausse ;
2. il explore l'arène avec la recherche décrite en 1.8 : tours sur place et
   points libres de la carte ;
3. chaque tag aperçu de loin (à plus de 1,3 m) est mesuré de près : le robot
   va se placer à 0,9 m devant lui, car une vue proche est bien plus précise,
   puis recule de 30 cm pour quitter les abords du tag ;
4. il s'arrête dès que les 4 tags sont mesurés de près : explorer davantage
   n'apporte rien au match et risque de le coincer contre un meuble. S'il ne
   les trouve pas tous, il continue jusqu'à avoir parcouru deux fois toute
   l'arène, ou au bout de 12 minutes (il reste 3 minutes de marge) ;
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
   visité, à au moins 50 cm des obstacles pour laisser passer toute l'empreinte
   du robot ;
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
l'angle mesurés, et à la fin, pour chaque tag non visité, **pourquoi**. Le
micro raconte aussi ce qu'il entend (`[micro] Entendu « alors » : pas le mot
de départ`).

Sous le récit, en plus discret, la **surveillance technique** montre la
mécanique (dans l'esprit du moniteur de la branche benchmark) :

```text
        [NAV2] trajet en cours
        [CHEMIN] trajet planifié : 2,31 m
        [CHEMIN] détour : chemin rallongé de 0,80 m (3,11 m), obstacle sur l'ancien chemin ?
        [ROTATION] rotation sur place de 9 s
        [ARRÊT] immobile depuis 6 s pendant un trajet Nav2
        [BLOQUÉ] commandes envoyées (v=-0.08, w=+0.00) mais le robot ne bouge pas depuis 5 s
        [OSCILLATION] le robot tourne à gauche puis à droite sans progresser (3 changements de sens en 5 s)
        [NAV2] ÉCHEC : Nav2 a abandonné le trajet
        [ÉTAT] position (1,23 ; -0,50), cap +90° | v=+0.20 w=+0.00 | Nav2 actif | chemins 7 | contacts 0
```

Les alertes (`[BLOQUÉ]`, `[OSCILLATION]`, `[CONTACT]`, échec de Nav2) sont en
rouge.
`./run.sh suivi --sans-technique` n'affiche que le récit.

## 1.15 Se dégager quand le robot est coincé

En simulation, le robot est resté coincé entre un tag et un meuble : Nav2
n'arrivait plus à repartir. Le robot recule donc de 30 cm, lentement et avec
une garde lidar à l'arrière :

- après avoir mesuré un tag pendant la préparation ;
- après chaque visite pendant le match, avant de partir vers le tag suivant ;
- après deux trajets de recherche ratés de suite.

## 1.16 Comment nous l'avons vérifié

- **des tests automatiques**, sans robot ni simulateur : géométrie,
  alignement, stratégie, lecture de l'ordre, messages de l'arbitre,
  configuration, détection sur images synthétiques.
- **Simulation Gazebo** : le simulateur officiel PARC 2026 (arène générée par
  graine, arbitre et messages officiels), avec des courses complètes de bout
  en bout. Un moniteur mesure chaque visite avec la vraie position du robot
  dans le simulateur.
- **Résultat** : sur l'ancien monde (tags ajoutés à la main), les quatre tags
  ont été visités dans l'ordre 7, 1, 9, 4 en 370 s sur 600, chacun signalé à
  l'arbitre avec sa photo, face au tag à moins de 1,5°. Ce chiffre est à
  reconfirmer sur l'arène officielle seedée avec
  `simulation/simulation.sh essai`.
- **Deux mesures historiques, propres à l'ancien simulateur :**
  - l'odométrie du modèle Gazebo surestimait les rotations de 15 %, à cause
    d'un mauvais écartement des roues dans le modèle. Le modèle officiel a été
    repris ; sur le vrai robot, `./run.sh mesure-rotation` compare l'odométrie,
    le gyroscope et le lidar ;
  - dans l'ancien rendu, le tag paraissait 3,8 % plus grand qu'en réalité, et
    le robot s'arrêtait à 0,55 m au lieu de 0,50. Les tags officiels ont une
    géométrie différente (carré noir de 0,12 m dans une face de 0,15 m) : la
    taille est réglée dans `profils/simulation.yaml` et reste à remesurer.

## 1.17 Questions probables du jury

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

**Pourquoi une préparation ?** D'après les consignes communiquées oralement
par les organisateurs, les 15 minutes avant le match servent à refaire la
carte et repérer les tags. Le match peut ainsi commencer avec ces données.
Même avec une carte enregistrée, le lidar et les caméras continuent de voir
les obstacles mobiles.

**Et si la voix échoue ?** Le départ par commande reste toujours possible ; la
voix n'est qu'un bonus.

**Pourquoi OpenCV plutôt qu'un paquet AprilTag ROS ?** OpenCV est déjà sur le
robot, sans installation système (interdite sans les organisateurs), et nous
maîtrisons chaque réglage.

---
