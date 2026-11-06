# Audit des branches — finale PARC, 6 novembre 2026

## Décision

**Base retenue : `fix/diagnostic-cyclonedds` (`925ca34`).** Elle contient déjà
`feat/phase-reelle` (`a69ca41`). La branche produite est
`integration/finale-propre-2026-11-06`, avec reprise sélective des contrôles de
fraîcheur de `integration/phase-reelle-securisee` (`b4fae57`).

« Stable » désigne ici une base cohérente et testable, pas une course physique
réussie. Aucun essai sur le robot partagé n’a été effectué pendant cet audit.

## Comparaison

Références contrôlées après `git fetch origin`. Les dates ci-dessous sont celles
des derniers commits ; Git ne conserve pas une date de création fiable des branches.

| Branche / révision | Dernier commit | Constat | Décision |
|---|---|---|---|
| `feat/phase-reelle` / `a69ca41` | 06/11, 02:01 +03 | Six paquets uniques, moteur de course complet, SLAM, voix Vosk, arbitre, préparation manuelle et autonome | Base fonctionnelle, déjà incluse dans le correctif DDS |
| `fix/diagnostic-cyclonedds` / `925ca34` | 06/11, 17:28 +03 | Même moteur, diagnostic de création des nœuds et surcharge DDS préservant le réseau hérité | Meilleure base retenue |
| `origin/feat/to-fix-simulate` / `5bb09ef` | 06/11, 02:45 +03 | Sept paquets ; autre moteur, architecture concurrente | Pas de fusion globale : ordre de secours inventé et signalement possible après alignement insuffisant |
| `origin/feat/solution-voix` / `1cc5ad2` | 06/11, 19:01 +03 | Huit noms de paquets, dont cinq en double à cause de `fosa-solution.18/` ; reprend le moteur précédent | Non compilable globalement telle quelle ; pas une base stable malgré sa date récente |
| `integration/phase-reelle-securisee` / `b4fae57` | 05/11, 15:44 +03 | Contrôles utiles de fraîcheur, modifications de navigation sur une autre intégration | Reprise ciblée des contrôles, adaptée à la configuration courante |
| `integration/phase-finale` / `cff9736` | 03/11 | Architecture plus ancienne, sept paquets | Écartée au profit de la lignée phase réelle |

La copie `origin/feat/solution-voix:fosa-solution.18` est **identique** à l’arbre
`925ca34`. La fusionner ne rapporte aucun correctif supplémentaire et introduit
les doublons `caytu_config`, `caytu_interfaces`, `caytu_nav_bringup`,
`caytu_nav_solution`, `caytu_perception`.

Vérifications reproductibles :

```bash
git merge-base --is-ancestor a69ca41 925ca34
git diff --exit-code 925ca34 origin/feat/solution-voix:fosa-solution.18
git log --graph --oneline --decorate --all
```

## Conformité recherchée

Sources : guide de phase réelle, inventaire ROS du 02/11, consignes arbitre dans
`docs/parc-phase-finale-docs/`, annonces transmises du 06/11 et sources officielles
voisines à la révision `4ac1876` (ajout des paquets referee).

| Exigence | Intégration produite |
|---|---|
| ROS 2 Jazzy et interfaces du robot | Six paquets uniques, vrais `parc_msgs`, profil robot en domaine 0 |
| Positions de tags à découvrir | Détection des deux caméras ; sans carte, aucun chargement de poses préalables |
| Caméras inversées robot/simulation | camera2 avant sur robot, camera1 avant en simulation, preuves avec leur propre `frame_id` |
| Solution sans carte recommandée | Profil `sans_carte`, Nav2 en `odom`, costmaps roulantes, SLAM absent |
| Ordre officiel et absence de faux départ | Réception `/parc/tag_order`, attente du départ ; `4,9,1,7` réservé au défaut du banc simulé |
| Visite à 0,5 m, cap ±12°, JPEG | Recontrôle géométrique et image fraîche associée au même tag ; visite sans JPEG refusée |
| Maintien après signalement | 5 s réelles, minimum de configuration 3 s, maintien sur le dernier `FINISHED` |
| Limite de 10 minutes | Budget monotone réel comme l’arbitre, marge de 5 s, interruption Nav2 et garde |
| Obstacles et personnes | Nav2 et garde sur lidar/profondeur ; dimensions et efficacité physique à valider au créneau |
| Barème annoncé le 06/11 | Stratégie à 20 points/tag autonome ; 10 si solution hardcodée selon les juges |
| Départ vocal | Vosk hors ligne, mot exact, publication `StartSignal` avec méthode et mot entendu |
| Installation dans le dossier équipe | Archive Fosa limitée, manifeste SHA-256, compilation neuve, activation et ancienne livraison conservée |
| Processus et coupure SSH | Sessions détachées, arrêt ciblé par groupe et jeton |
| Réservation, redémarrage, Wi-Fi | Procédures du README mises à jour |

L’ancien guide parle d’un total de 100 points plus bonus vocal et le referee
local distingue points de visite et navigation sûre. L’annonce plus récente
précise 20/10 par tag selon l’autonomie ; le code ne prétend pas attribuer le
verdict ni modifier le barème des organisateurs.

## Nettoyage

- Suppression de `tuer.sh` et `tuer.md` : doublon de diagnostic, commandes
  d’arrêt trop larges et consigne liée à une ancienne adresse robot.
- Suppression de `README_Fosa_Guide.md` : guide concurrent devenu obsolète.
- Remplacement du README long par un parcours PC → livraison → créneau → match.
- Conservation des documents originaux organisateurs, de l’audit ROS et des
  diagnostics réellement utilisés.
- Aucune importation du dossier dupliqué, de l’ancien assistant conversationnel
  ou du second moteur de course. Pas de faux paquet `parc_msgs` dans la livraison.
- Builds, résultats, environnements Python et téléchargements restent hors Git.

## Validation

### Résultats acquis sur la base

`925ca34` : **212 tests réussis, 2 ignorés** dans Docker Jazzy, après compilation
des vrais messages `caytu_interfaces` et `parc_msgs`. Les deux tests ignorés
réservent les ports DDS et exigent un conteneur explicitement isolé.

### Validation de l’intégration

Commande de tests unitaires et ROS, exécutée sur l’intégration :

```bash
FOSA_IMAGE_PRETE=1 FOSA_SIM_WORKSPACE=/tmp/opencode/fosa-validation-2 \
  ./outils/verifier.sh --docker
```

Résultat : **243 tests réussis, 0 échec, 0 ignoré** dans Docker Jazzy
(`resultats/tests.xml`). Les tests couvrent notamment : ordre, géométrie,
alignement, stratégie, JPEG, association image/tag, capteurs périmés, empreinte
balayée, surcharge DDS, archive limitée, modes exécutables et arrêt ciblé. Le
banc ROS utilise les vrais messages, vérifie un ordre publié avant l’abonnement,
un départ, une visite JPEG, son maintien et l’arrêt arbitre.

La panne Gazebo du pont officiel a été reproduite : le `PosePublisher` Jazzy
publie `/model/sitoe_robot/pose`, alors que `parameter_bridge` attend
`sitoe_robot/pose`, ce qui laisse `/sim/pose` vide. Le correctif `pont_pose` a
été vérifié séparément : `/sim/pose_verite` reçoit bien la pose du robot
(`x≈0`, `y≈-7`, orientation cohérente). La répétition complète des quatre
visites reste à confirmer sur un PC assez rapide ; elle n’est pas revendiquée
comme acquise dans ce document.

### Validation terrain nécessaire

Une course complète sur le robot, le transfert SSH vers le Jetson et la voix
avec son ReSpeaker exigent le matériel et un créneau réservé. Relever au créneau
les dimensions, TF, calibration, données caméra, freinage, quatre visites et
arrêt arbitre. Les résultats unitaires ne constituent pas cette preuve terrain.
