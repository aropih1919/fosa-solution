# Solution Fosa — architecture et points de vigilance

Ce document rassemble les choix techniques et les garde-fous de la solution
CAYTU Sito-É pour la PARC Engineers League 2026. Les procédures et commandes
détaillées sont dans le [guide d’utilisation](../README.md).

## Contraintes du match

- Visiter les AprilTags **1, 4, 7 et 9** dans l’ordre communiqué par l’arbitre.
  Un tag peut être sauté, sans retour possible.
- Arrêter le centre du robot à **0,5 m** du tag, face à lui à **±12°** près,
  maintenir la position et transmettre son numéro avec une photo.
- Disposer de **10 minutes**, sans intervention physique après le départ.
  Chaque tag rapporte jusqu’à **25 points**, sous réserve des pénalités de
  contact ; le départ à la voix (« allez ») rapporte **5 points** de bonus.
- Utiliser les **15 minutes de préparation** annoncées oralement par les
  organisateurs pour refaire la carte et repérer les tags. L’arène de
  5 m × 5 m et son mobilier peuvent changer entre deux matchs.

## Architecture

| Paquet ROS 2 | Responsabilité |
|---|---|
| `caytu_config` | Configuration centralisée et validation des réglages. |
| `caytu_perception` | Détection des tags et obstacles hauts issus de la profondeur. |
| `caytu_nav_bringup` | Configuration de Nav2. |
| `caytu_nav_solution` | Chronomètre et client de navigation. |
| `caytu_course` | Préparation, départ, stratégie, recherche, approche et échanges avec l’arbitre. |
| `caytu_interfaces` | Message de détection partagé entre perception et course. |

```text
Caméras → tags mémorisés ────────────┐
Lidar → slam_toolbox → carte, pose ─┼→ caytu_course → destinations Nav2
Arbitre → ordre ; micro → départ ──┘       │                  │
                                         └→ visites, fin     └→ /cmd_vel
Lidar + profondeur → obstacles → cartes de coût Nav2
```

Les valeurs réglables sont centralisées dans `caytu_config`, notamment
`caytu_config/config/fosa.yaml` et les profils. Les valeurs citées ici
expliquent le comportement ; la configuration reste la référence.

## Préparation et mémoire des tags

Deux modes alimentent les mêmes données :

- **Manuel** (`preparation-manuelle`) : l’équipe guide le robot ; le nœud
  enregistre la carte et les détections sans envoyer de buts de navigation.
- **Autonome** (`preparation`) : le robot explore et mesure les tags de près.
  Il archive la préparation précédente et repart d’une carte neuve.

En autonome, un tag aperçu à plus de **1,3 m** est approché à **0,9 m** pour
affiner sa position. La préparation s’arrête dès que les quatre tags sont
mesurés de près, après deux parcours complets de l’arène, ou à **12 minutes**
pour conserver trois minutes de marge.

Les données sont enregistrées dans `~/.ros/fosa/preparation/` :
`carte.posegraph` et `carte.data` pour recharger le SLAM, `carte.pgm` pour
visualiser la carte, `tags.yaml` pour les tags. Les tags sont sauvegardés au
fil de la préparation. En mode manuel, lancer `./run.sh sauver-carte` avant
`./run.sh arreter` ; ne pas lancer les deux modes simultanément.

La mémoire retient tous les tags vus, même hors cible. Les observations
proches pèsent davantage ; une vue nettement plus proche remplace les
anciennes. Un tag absent de sa position mémorisée est oublié puis recherché.

Au match, `slam_toolbox` recharge la carte en mode localisation. **Le robot
doit repartir de la position où la préparation a commencé**, à l’origine de
la carte. Sans préparation utilisable, il construit la carte pendant le
match. Les obstacles mobiles restent pris en compte par les cartes de coût.

### Conditions de démarrage à préserver

- **Découvrir le topic avant d’attendre un message.** `ros2 topic echo --once`
  peut quitter en erreur si le topic n’est pas encore découvert. Sonder
  `ros2 topic list`, puis vérifier la réception effective d’un message avant
  de déclarer le lancement prêt. L’absence temporaire du topic ne suffit pas
  à conclure que le robot simulé n’a pas démarré.
- **Recevoir la carte déjà publiée.** L’abonnement de `course_parc` à `/map`
  doit conserver le QoS `TRANSIENT_LOCAL` : en `VOLATILE`, il peut manquer
  la carte de `slam_toolbox` et attendre sans jamais planifier.
- **Attendre les serveurs Nav2 requis** avant la première action de recherche.

## Perception et navigation

Les deux RealSense D435i fournissent les images couleur et de profondeur.
OpenCV détecte les AprilTags **36h11** et estime leur pose par `IPPE_SQUARE`,
puis la transforme dans les repères du robot et de la carte. La taille réelle
du tag doit correspondre au profil utilisé : elle conditionne directement
la distance estimée. Le réglage du détecteur couvre les impressions avec
une fine marge blanche sur fond gris, vérifiées sur images synthétiques.

La profondeur est filtrée par hauteur puis convertie en scans pour Nav2 :
elle complète le lidar bas, qui voit les pieds d’une table mais peut manquer
son plateau.

Nav2 utilise **Smac 2D** pour planifier et **Regulated Pure Pursuit** pour
suivre les chemins. Le BT navigator replanifie à **2 Hz** et borne les
tentatives de récupération. Le lidar et la profondeur alimentent les cartes
de coût ; la vitesse est plafonnée à **0,3 m/s**.

### Recherche dans une arène encombrée

Si la cible n’est pas mémorisée, le robot alterne un balayage lent sur place
et un trajet vers le point libre non visité le plus proche. La recherche
s’arrête dès que la cible est détectée.

Les essais ont montré deux pièges à éviter : éliminer tous les candidats
avec des marges trop restrictives, ou accepter un passage où le robot ne
tient pas. Les règles à conserver sont donc :

- échantillonner plus finement près des frontières de carte si nécessaire ;
- distinguer les obstacles connus de la frontière inconnue : seule la marge
  côté inconnu peut être relâchée ;
- garder **0,50 m autour des obstacles** et laisser Nav2 valider l’empreinte
  complète. Une marge de secours de 0,35 m était inférieure au rayon
  d’encombrement d’environ 0,43 m avec padding : le contrôleur restait bloqué
  sur `collision ahead` ;
- rejeter en configuration une marge de secours inférieure à la marge normale ;
- arrêter avec un diagnostic après **deux balayages consécutifs sans point
  de navigation**, plutôt que tourner jusqu’à expiration de la préparation.

### Approche finale et dégagement

L’approche se fait en deux étapes :

1. **Nav2 place le robot à 0,9 m**, face au tag. Lui demander directement
   0,5 m provoquait des refus d’avancer à proximité du support du tag.
2. **L’asservissement visuel vise le point à 0,5 m sur la normale au tag**,
   corrige la translation, puis l’orientation. Les tolérances internes sont
   **1,2 cm et 2°** ; la visite exige plusieurs mesures stables consécutives.
   Une garde lidar bloque l’avance ou le recul si un obstacle est trop proche.

Une vue trop lointaine ou trop oblique rend la main à Nav2 pour se replacer.
Après validation, le robot transmet le numéro et la photo, puis maintient
sa position pour les juges.

Pour quitter les abords d’un tag, le robot recule de **30 cm**, lentement et
sous garde lidar arrière : après une mesure en préparation, après une visite
avant de repartir, et après deux trajets de recherche ratés consécutifs.

## Stratégie de course

L’ordre étant imposé, chaque tag restant est soit conservé, soit sauté :
**au plus 16 plans** pour quatre tags. Le robot les compare tous, sans
ajouter un planificateur de chemins qui ferait doublon avec Nav2.

Pour chaque plan, il estime :

- la durée : distance × **1,4** pour les détours, divisée par la vitesse
  réelle, plus le temps de visite et, si nécessaire, de recherche ;
- les points espérés : **25 par tag**, pondérés par une probabilité de
  réussite qui baisse avec les échecs et les recherches prolongées.

Il écarte les plans incompatibles avec le temps restant, avec **20 % de
marge**, puis maximise les points espérés et départage par la durée.
La cible actuelle n’est sautée que si le meilleur plan sans elle rapporte
**au moins 5 points de plus** que le meilleur plan avec elle. Ce seuil évite
les abandons prématurés dus à des différences d’estimation minimes.

Le calcul est refait toutes les **2 secondes**, même en navigation, à partir
de la position et des détections actualisées. Il considère les gains encore
possibles, pas le temps déjà dépensé.

## Départ, arbitre et fin de course

Le départ se donne par `./run.sh depart` ou par **Vosk**, hors ligne.
L’écoute n’est active qu’en attente du départ. La reconnaissance transcrit
librement la phrase, puis exige exactement **« allez »** avec une confiance
minimale : une grammaire limitée confondait notamment « alors », « allô » et
« Alex » avec le mot de départ. Aucun mouvement de course ne précède le départ.

| Événement | Topic | Contenu |
|---|---|---|
| Ordre reçu | `/parc/tag_order` | Ordre des tags. |
| Départ | `/parc/start` | `method` : 0 = commande, 1 = voix ; `heard` : mot entendu. |
| Visite | `/parc/tag_visit` | Numéro du tag et photo JPEG. |
| Fin | `/parc/finish` | `reason`. |

Le code lit les définitions de messages installées au démarrage pour adapter
la lecture de l’ordre et le champ de la photo. Les vérifications propres au
robot sont détaillées dans l’[audit des interfaces ROS](../AUDIT-TOPICS-PARC.md).

Le chronomètre utilise l’horloge ROS à partir du départ. À **10 minutes moins
5 secondes**, le but actif est annulé, le robot s’arrête et signale la fin.
La course tourne dans `tmux` pour survivre à une coupure SSH ; les nœuds de
perception sont relancés automatiquement en cas de plantage.

## Diagnostic et validation restante

`./run.sh suivi` affiche les décisions en français, le temps restant, les
plans et leurs estimations, les visites et les raisons d’échec ou de saut.
La surveillance technique signale notamment les blocages malgré des
commandes moteur, les oscillations, les contacts et les échecs Nav2.
`--sans-technique` limite l’affichage au récit de course.

Les tests automatiques couvrent la géométrie, l’alignement, la stratégie,
la configuration, les interfaces de l’arbitre et la détection synthétique.
Les essais de bout en bout utilisent Gazebo PARC 2026 et un moniteur qui
mesure les visites à partir de la position réelle dans le simulateur.

**À confirmer sur les conditions actuelles :**

- Revalider une course complète sur l’arène officielle générée par graine
  avec `simulation/simulation.sh essai`. La réussite sur l’ancien monde
  ne constitue pas une validation du simulateur officiel.
- Remesurer la taille utile des tags officiels dans le profil de simulation
  (`profils/simulation.yaml`) : carré noir de **0,12 m** dans une face de
  **0,15 m**. Ne pas confondre la taille de la face avec celle utilisée pour
  estimer la pose.
- Sur le robot réel, comparer odométrie, gyroscope et lidar avec
  `./run.sh mesure-rotation`, puis vérifier la distance d’arrêt au tag.
