# Exemple de solution cible — PARC 2026, phase réelle

Ce document propose une synthèse des points forts de `feat/phase-reelle` et
`liantsoa/essaie`, en les rapportant aux exigences du guide officiel PARC 2026
fourni en pièce jointe (`PARC2026_Phase_Reelle_Guide_FR.pdf`).
La proposition a été mise en œuvre partiellement dans la branche
`integration/phase-reelle-securisee`, créée depuis `feat/phase-reelle`.
148 tests de configuration, course, perception et BT passent (un test dépendant
de ROS est ignoré); la compilation ROS, la simulation et les essais robot restent
à faire.

## Éléments à reprendre

| Source | Point fort à conserver | Rapport avec la tâche |
|---|---|---|
| `feat/phase-reelle` | Orchestration de course, lecture de l'ordre publié, stratégie de visite dans l'ordre, limite de temps, signalement à l'arbitre et démarrage vocal hors ligne. | Répond au déroulement autonome de 10 minutes, à l'ordre aléatoire, aux visites et au bonus vocal. |
| `feat/phase-reelle` | Détection AprilTag dédiée, estimation géométrique de pose, approche suivie d'un alignement visuel et tests synthétiques de distance/orientation. | Cible les tags 36h11 inconnus et la visite à 0,5 m face au tag. |
| `feat/phase-reelle` | Conversion de la profondeur des caméras en observations d'obstacles pour Nav2, en complément du LiDAR. | Aide à voir les meubles que le LiDAR bas pourrait manquer. |
| `liantsoa/essaie` | Rejet des observations périmées, contrôle des horodatages et des TF, et suivi des objets dans `odom`. | Réduit les décisions fondées sur des mesures anciennes ou faussées par le mouvement du robot. |
| `liantsoa/essaie` | Conservation du but pendant un blocage, observation après absence de progrès et absence de recul/pivot aveugle dans le BT. | Favorise la reprise autonome sans intervention manuelle et limite le risque de contact. |
| `liantsoa/essaie` | Garde-fou de recul exigeant une couverture arrière suffisante; recul désactivé par défaut. | Une marche arrière ne doit être permise qu'après validation de la zone arrière. |

## Architecture recommandée

1. **Préparer la course.** Démarrer les capteurs, la localisation/cartographie
   et Nav2 sur le robot. Privilégier la cartographie en direct tant que
   l'autorisation des cartes préconstruites n'est pas confirmée. Charger le
   modèle vocal et les autres dépendances avant le jour de la course.
2. **Attendre le départ.** Lire l'ordre officiel des tags depuis le topic des
   organisateurs. Ne démarrer le chronomètre et le mouvement qu'à la commande
   ou à la reconnaissance hors ligne du mot choisi; la voix ne sert qu'au
   départ.
3. **Trouver et visiter les tags.** Utiliser la détection 36h11 et les IDs
   `1, 4, 7, 9`. Construire la pose depuis l'image, la calibration et la TF;
   approcher avec Nav2, puis finir par asservissement visuel. Valider le centre
   à `0,5 m`, l'orientation dans la tolérance officielle de `±12°`, et
   l'immobilité requise. Publier l'ID et l'image de preuve sur l'interface de
   statut officielle. La durée de maintien ou le signal de confirmation doit
   suivre le protocole définitif des organisateurs.
4. **Éviter les obstacles.** Utiliser le LiDAR et les scans issus de la
   profondeur RGB-D comme sources de navigation pour Nav2. Traiter toute
   personne comme un obstacle, sans comportement de salutation ni de suivi.
   La classification YOLO peut servir au diagnostic, mais ne doit ni envoyer
   de commande moteur ni remplacer les données de collision des costmaps.
5. **Gérer un blocage.** Garder la cible de tag courante, replannifier sur des
   observations fraîches et attendre immobile si aucun trajet sûr n'est
   disponible. Ne pas effacer les obstacles observés pour forcer un passage.
   Ne reprendre ou sauter une cible qu'en respectant l'ordre et les règles de
   la course. Un recul éventuel doit être court, lent, surveillé et interdit
   sans couverture arrière démontrée; le laisser désactivé jusque-là.
6. **Terminer proprement.** Signaler les visites et la fin de course, arrêter
   le robot à la limite de temps ou en cas de condition d'arrêt immédiat. Aucun
   mécanisme ne doit nécessiter une intervention de l'équipe après le départ.

## État de l'intégration

La branche dédiée part de `feat/phase-reelle`; les branches sources restent
intactes. La course PARC, l'ordre des AprilTags, l'alignement visuel et les
preuves arbitre sont donc conservés. Les garde-fous ajoutés sont :

- Le BT de navigation replannifie, mais n'efface aucune costmap et ne lance ni
  recul ni pivot de récupération. Après un échec, `NoeudCourse` s'immobilise,
  observe pendant `course.blocked_observation_sec` et reprend le même but. La
  stratégie ne peut sauter la cible qu'après `course.same_target_retries`.
- L'alignement s'arrête si le scan est absent, périmé, sans mesure valide, sans
  TF au timestamp exact ou sans couverture circulaire complète. Un obstacle
  dans le couloir ou dans le disque balayé par le robot bloque toute commande;
  la perte du tag entraîne une attente immobile. Le recul reste désactivé par
  `course.alignment.allow_reverse: false`.
- Les scans profondeur périmés sont rejetés. La TF est demandée au timestamp de
  la mesure avec un délai borné; aucun repli vers la TF « la plus récente »
  n'est appliqué. Les limites de fraîcheur et délais sont configurables.
- L'évitement de collision reste fondé sur les couches LiDAR/profondeur de
  Nav2. YOLO n'est pas une autorité de collision; aucune salutation ni poursuite
  de personne n'est ajoutée au nœud de course.

## Validations restantes

1. Avec l'environnement ROS installé, compiler le workspace et exécuter aussi
   les tests Nav2 nécessitant `action_msgs` et `rclpy`.
2. En simulation, vérifier les horodatages TF, l'ordre aléatoire, le tag perdu,
   un obstacle mobile, l'observation immobile, les reprises bornées et la limite
   de temps.
3. Sur le robot, confirmer les topics/frame_id réels, la couverture angulaire
   arrière, les distances d'arrêt et les interfaces `parc_msgs`; effectuer les
   essais encadrés avant toute activation du recul. Les noms de caméra du guide
   et ceux de l'inventaire ROS fourni diffèrent encore.

## Choix à ne pas reprendre sans adaptation

- Ne pas faire dépendre l'évitement d'une classe YOLO `personne` : la règle
  demande d'éviter toute personne, et une détection sémantique peut manquer.
  Les scans doivent continuer à représenter les obstacles pour Nav2.
- Ne pas reprendre tels quels les effacements de costmap, rotations et reculs
  génériques du BT de `feat/phase-reelle` pour gérer un obstacle mobile. Les
  remplacer par la politique conservatrice de maintien du but et d'observation;
  toute récupération en mouvement doit être bornée et surveillée.
- Ne pas déclarer une visite à partir de la seule détection du tag : la
  distance, l'alignement, l'arrêt, l'image et le signalement officiel comptent.
- Ne pas supposer que les topics ou types de statut provisoires sont définitifs;
  les vérifier avec les organisateurs avant le gel du code.

## Vérifications avant le gel

- Tester l'ordre arbitraire, les tags manquants/sautés et la limite de 10 min.
- Mesurer sur le robot l'erreur de distance et d'orientation à la visite, puis
  vérifier l'image et le signalement de l'arbitre.
- Déplacer un obstacle absent de la carte et vérifier arrêt, replannification
  ou attente, puis reprise autonome de la même cible sans contact.
- Tester les personnes comme obstacles, la fraîcheur des scans, les distances
  d'arrêt et, si un recul est envisagé, toute sa couverture arrière.
- Vérifier que la reconnaissance vocale est hors ligne, n'accepte que le mot
  choisi et ne fait pas bouger le robot avant le départ.
- Répéter les scénarios en simulation puis sur le robot réel; une réussite de
  tests unitaires ne remplace pas ces essais de sûreté.

## Références de code

- `feat/phase-reelle` : `caytu_course/`, `apriltag_opencv.py`,
  `scan_profondeur.py`, `noeud_scan_profondeur.py`.
- `liantsoa/essaie` : `classificateur_obstacles.py`,
  `navigate_bounded_recovery.xml` et la politique de reprise de mission.
