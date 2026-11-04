# Comment nous avons résolu le problème

Document de préparation à l'oral : pour chaque difficulté de la course, le
problème, notre solution, pourquoi ce choix, et comment nous l'avons vérifié.
Le détail du code est dans [`FLUX_DE_LA_SOLUTION.md`](FLUX_DE_LA_SOLUTION.md).

## 1. Le problème en une phrase

Conduire le CAYTU Sito-É, sans carte ni coordonnées, jusqu'à quatre AprilTags
(1, 4, 7, 9) dans un ordre tiré au sort au départ, s'arrêter à 0,5 m face à
chacun (±12°), sans toucher le mobilier ni les personnes, en 10 minutes au
plus, et signaler chaque visite à l'arbitre avec une image.

Contraintes qui ont guidé nos choix :

| Contrainte | Conséquence pour nous |
|---|---|
| Aucune carte, tags à découvrir | Cartographie en direct et recherche active des tags. |
| Ordre imposé, saut permis mais sans retour | Une stratégie qui décide quand sauter. |
| 10 minutes, chronomètre lancé au départ | Chaque décision tient compte du temps restant. |
| Personnes et mobilier déplacé | Obstacles détectés en direct, vitesse limitée. |
| Visite = 0,5 m, ±12°, position tenue | Placement final précis, plus strict que le règlement. |
| Aucun contact humain après le départ | Robot 100 % autonome, course détachée de la session SSH. |

## 2. L'architecture en une image

```text
 caméras ──► détection AprilTag ──► mémoire des tags ──┐
 caméras ──► profondeur → obstacles hauts ──┐          │
 lidar ────► SLAM (carte + position) ───────┼──► Nav2 ◄─┤
                                            │          │
 arbitre ──► ordre des tags ────────────────┼──► stratégie ──► recherche
 micro ───► mot « allez » ──► départ        │          │        approche
                                            │          │        alignement
                                            └──────────┴──► arbitre (visites, fin)
```

Six paquets ROS 2, une seule source de configuration (`caytu_config`) : aucun
seuil n'est codé en dur dans les nœuds.

## 3. Les sous-problèmes et nos réponses

### 3.1 Savoir où l'on est sans carte

- **Solution** : slam_toolbox construit la carte en direct pendant la course,
  à partir du lidar, et fournit la position du robot.
- **Pourquoi** : le règlement peut interdire les cartes pré-construites ; une
  carte faite pendant la course est toujours à jour (mobilier déplacé).
- **Bonus** : si les organisateurs l'autorisent, une carte enregistrée pendant
  les essais peut être rechargée (`course.slam_map_file`).

### 3.2 Se déplacer sans rien heurter

- **Solution** : Nav2, avec le planificateur Smac 2D (chemin sur la carte) et
  le contrôleur Regulated Pure Pursuit (suivi du chemin, ralentit près des
  obstacles). Vitesse plafonnée à 0,3 m/s.
- **Obstacles hauts** : un plateau de table est invisible pour le lidar, posé
  bas. Les images de profondeur des deux caméras sont converties en « faux
  lidar » d'obstacles entre deux hauteurs et ajoutées aux cartes de coût.
- **Pourquoi** : Nav2 est la référence de ROS 2 et recommandé par les
  organisateurs ; RPP est simple, prévisible et facile à régler.
- **Sécurité** : empreinte réelle du robot plus une marge, arbre de
  comportement aux reprises bornées (jamais de boucle infinie), garde avant
  et arrière pendant les manœuvres finales.

### 3.3 Voir et localiser les tags

- **Solution** : détection AprilTag 36h11 par OpenCV sur les deux caméras,
  pose estimée avec la taille réelle du tag (16,5 cm) par la méthode
  `IPPE_SQUARE`, prévue pour les carrés plans.
- **Réglage mesuré** : avec les paramètres par défaut, OpenCV ratait les tags
  imprimés avec une marge blanche sur un fond gris. Les paramètres du
  détecteur ont été réglés et vérifiés sur des images de synthèse.
- **Mémoire des tags** : chaque tag vu est mémorisé dans la carte, même s'il
  n'est pas la cible. Les vues proches comptent plus que les lointaines (une
  vue à 1,5 m dispersait la position de 60 cm en simulation).

### 3.4 Trouver un tag jamais vu

- **Solution** : le robot fait un tour lent sur lui-même, puis va vers les
  points libres de la carte les plus proches et recommence. Les caméras
  balaient ainsi les murs de l'arène.
- **Pourquoi** : l'arène ne fait que 5 m × 5 m ; une exploration simple et
  systématique suffit, et la carte SLAM indique où le robot peut aller.

### 3.5 Se placer à 0,5 m face au tag

C'est le point qui nous a demandé le plus de travail.

- **Constat en simulation** : Nav2 n'arrivait pas à amener le robot à 0,5 m
  d'un tag sur son support. Son contrôle de collision voyait un obstacle juste
  devant et refusait d'avancer (plus de 14 000 alertes en une course). Deux
  tags sur quatre étaient perdus.
- **Solution en deux étages** :
  1. Nav2 amène le robot à 0,9 m, face au tag (zone dégagée, rapide) ;
  2. un asservissement visuel finit : le robot roule vers le point situé à
     0,5 m sur l'axe du tag, en corrigeant son cap à chaque image, puis tourne
     sur place pour faire face au tag.
- **Précision** : nous visons 2,5 cm et 2°, bien plus strict que les ±12° du
  règlement, pour garder de la marge face aux juges.
- **Validation** : la visite n'est signalée qu'après plusieurs mesures stables
  consécutives ; le robot garde ensuite la position.

### 3.6 La stratégie : quels tags tenter, et quand sauter

Question : avec un ordre imposé, des tags dispersés et 10 minutes, faut-il
toujours viser le tag suivant ?

- **Ce que dit le règlement** : on peut sauter un tag, jamais y revenir, et
  les tags suivants rapportent toujours leurs points. Chaque tag vaut 25
  points (20 de visite + 5 de navigation sûre).
- **Observation clé** : comme l'ordre est imposé, il n'y a que 16 plans
  possibles pour 4 tags (garder ou sauter chacun). On peut donc **tous les
  évaluer** à chaque instant : la solution est exacte et instantanée.
- **Pour chaque plan, on estime** :
  - sa durée : trajets (distance × 1,4 pour le mobilier ÷ vitesse réelle),
    temps de visite, temps de recherche si le tag n'a jamais été vu ;
  - sa valeur : 25 points × probabilité de réussite, qui baisse à chaque
    échec sur ce tag et avec le temps passé à chercher un tag introuvable.
- **Décision** : parmi les plans qui tiennent dans le temps restant (avec 20 %
  de marge), on garde le plus rentable, puis le plus court. Si ce plan ne
  contient plus la cible actuelle, le robot la saute immédiatement.
- **Horizon glissant** : le calcul est refait toutes les deux secondes, avec
  la position réelle du robot et les tags découverts entre-temps. Le temps
  déjà perdu sur une cible n'entre jamais en compte : seul compte ce qui peut
  encore être gagné.

**Exemple** (testé dans `test_strategie.py`) : ordre 7, 1, 9, 4 ; le 7 et le 9
sont au nord, le 1 et le 4 près du départ ; il reste 3 minutes. Faire les
quatre demande 201 s : impossible. Parmi les plans à trois tags, 1-9-4 est le
plus court (132 s). Le robot saute donc le 7 tout de suite et marque 75
points au lieu de s'épuiser en allers-retours.

**Pourquoi pas un algorithme plus lourd ?**

- *Voyageur de commerce* : inutile, changer l'ordre ne fait que perdre des
  points.
- *Graphe et recherche de chemin maison* : Nav2 calcule déjà les chemins sur
  la carte réelle ; nous n'avons besoin que d'une estimation de durée.
- *Apprentissage par renforcement* : pas de données d'entraînement sur
  l'arène réelle, et une décision explicable vaut mieux devant un jury.

### 3.7 Le départ, par commande ou à la voix

- **Commande** : `./run.sh depart`.
- **Voix** (bonus de 5 points) : reconnaissance hors ligne avec Vosk sur le
  micro du robot, sans Internet. Le robot n'écoute que lorsqu'il attend le
  départ, et n'accepte que le mot exact « allez » avec une confiance minimale.
- **Pourquoi** : une grammaire restreinte confondait « alors », « allô » ou
  « Alex » avec « allez » ; le vocabulaire libre suivi d'une comparaison
  exacte évite qu'un bruit du public lance la course (course recommencée et
  bonus perdu).

### 3.8 L'arbitre officiel

- Le robot lit l'ordre sur `/parc/tag_order`, signale le départ sur
  `/parc/start` (méthode commande ou voix, mot entendu), chaque visite sur
  `/parc/tag_visit` avec une image JPEG de la caméra, et la fin sur
  `/parc/finish`.
- La définition exacte de certains messages ne nous a pas été transmise : le
  code lit la définition installée sur le robot au démarrage (type de l'ordre,
  champ de l'image) au lieu de la deviner.

### 3.9 Robustesse et sûreté

- Aucun mouvement avant le départ ; arrêt garanti à la fin du chronomètre
  (horloge ROS, marge de sécurité).
- La course tourne détachée de la session SSH : une coupure Wi-Fi ne l'arrête
  pas.
- Les nœuds de perception redémarrent seuls en cas de panne.
- Le premier créneau d'essai lance un diagnostic qui vérifie tous les topics,
  repères, caméras, l'audio et enregistre les définitions de l'arbitre.

## 4. Comment nous l'avons vérifié

- **Tests automatiques** : plus de 170 tests sans robot ni simulateur
  (géométrie, alignement, stratégie, lecture de l'ordre, messages de
  l'arbitre, configuration, détection sur images synthétiques).
- **Simulation Gazebo** : monde officiel PARC, quatre tags ajoutés, course
  complète de bout en bout avec mesure de chaque visite par la position réelle
  du simulateur (distance et angle).
- **Mesures** : l'odométrie du modèle Gazebo surestimait les rotations de 15 %
  (écartement des roues erroné) ; un outil (`./run.sh mesure-rotation`)
  compare odométrie, gyromètre et lidar sur le vrai robot.

## 5. Questions probables du jury

**Pourquoi ne pas aller droit au tag suivant ?** Parce qu'un tag introuvable
ou mal placé peut coûter plus de points qu'il n'en rapporte. La stratégie
compare tous les plans possibles et saute un tag seulement si cela rapporte
plus.

**Que se passe-t-il si un tag n'est jamais trouvé ?** Plus la recherche dure,
plus sa probabilité de réussite baisse ; dès qu'un plan sans lui rapporte
plus, il est sauté, et le robot passe au suivant.

**Comment évitez-vous les personnes ?** Lidar et caméras de profondeur
alimentent la carte de coût en continu ; Nav2 replanifie deux fois par
seconde et ralentit près des obstacles ; la vitesse est plafonnée.

**Comment êtes-vous sûrs d'être à 0,5 m ?** La distance vient de la pose du tag
mesurée par la caméra (taille réelle connue), recalculée à chaque image, et
la visite n'est validée qu'après plusieurs mesures stables, avec une précision
visée de 2,5 cm.

**Et si le démarrage vocal échoue ?** Le départ par commande reste toujours
disponible ; la voix n'est qu'un bonus.

**Pourquoi OpenCV plutôt qu'un paquet AprilTag ROS ?** OpenCV est déjà sur le
robot, sans installation système (interdite sans les organisateurs), et nous
maîtrisons chaque paramètre.

## 6. Limites connues

- Les durées de la stratégie sont des estimations prudentes, à recaler sur les
  essais réels (le journal affiche chaque plan et sa durée estimée).
- Certaines valeurs de l'arbitre restent à confirmer sur le robot (valeur de
  `reason` pour la fin, champ image de `TagVisit`) : le diagnostic du premier
  créneau les enregistre.
