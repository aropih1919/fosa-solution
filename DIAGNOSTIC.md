# Diagnostic des blocages de navigation

Analyse des journaux du 6 novembre 2026, croisée avec le code actuellement
présent dans le dépôt. Les paramètres effectivement chargés pendant l'essai
ne sont pas archivés : les valeurs actuelles ne prouvent pas celles de l'essai.
Les heures ci-dessous suivent l'heure locale des journaux de lancement
(UTC+3). Les références `fichier:ligne` sont relatives à ce dossier.

## Conclusion

Trois incidents distincts apparaissent :

1. **Préparation : immobilisation pendant un but Nav2.** RPP signale une
   collision imminente et `garde_vitesse` signale des arrêts, tandis que des
   chemins continuent d'être transmis. Le motif exact de chaque arrêt n'est
   pas enregistré ; les journaux ne départagent donc pas le contrôleur Nav2
   de la garde Fosa.
2. **Match : aucun ordre de tags accepté**, donc attente avant même l'envoi
   d'un but Nav2, malgré le départ reçu et une pile Nav2 active.
3. **Relances : absence du repère `odom`**, donc échec d'activation de Nav2.
   Le simulateur et son pont ont été arrêtés avant ces relances ; le repère
   demandé passe aussi de `base_footprint` à `base_link`.

Les éléments établissent des arrêts et des défauts de configuration/
environnement, mais **pas une cause unique permettant de corriger les
paramètres de navigation à l'aveugle**.

## 1. Préparation : Nav2 planifie, mais les commandes sont bloquées

### Chronologie démontrée

| Heure | Événement | Preuve |
| --- | --- | --- |
| 22:49:17–22:54:19 | Trois buts réussissent ; le tag 7 est approché et mesuré | `ros/log/bt_navigator_17323_1793994541910.log:11-16` |
| 22:55:51 | But vers le tag 1 : de `(1.57, 1.81)` vers `(-0.77, 2.03)` | même fichier, ligne 17 |
| 22:56:34–22:56:36 | 34 avertissements `RegulatedPurePursuitController detected collision ahead!` | `ros/log/controller_server_17320_1793994541961.log:491-527` |
| Vers 22:56:40 | Distance restante figée à 2,23 m ; les avertissements de la garde se répètent environ toutes les 2 s réelles | `ros/log/python3_17330_1793994549559.log:74-127` ; `ros/log/python3_17329_1793994546119.log:89-222` |
| 22:58:51 | `Failed to make progress`, abandon de `follow_path`, effacement de la costmap locale et nouvelle tentative | `ros/log/controller_server_17320_1793994541961.log:781-784` |
| 23:01:08 | Le budget de préparation provoque l'annulation du but | `ros/log/python3_17330_1793994549559.log:128` |
| 23:01:15 | Carte sauvegardée ; tags 1, 4, 7 mémorisés ; tag 9 absent | même fichier, ligne 132 |

Le moniteur confirme une position figée vers `(1.46, 1.77)`, cap `177°`,
avec `v=0, w=0` sur `/cmd_vel` — la sortie finale de la garde — et un compteur
de chemins qui continue à augmenter de 513 à 953 :
`fosa_simulation/journaux/20261106_224839_16629/preparation/moniteur.log:48-57`.
Il ne signale aucun contact, ce qui ne suffit pas à prouver l'absence de tout
obstacle ou la couverture complète des capteurs de contact.

**Interprétation :** le blocage se situe dans le suivi et l'autorisation du
mouvement. Ce n'est pas un échec général du calcul de chemin ou de la
localisation : trois trajets ont réussi auparavant, et les chemins continuent
à être transmis pendant l'immobilité. Le moniteur ne mesure pas l'entrée de
la garde (`/fosa/cmd_vel`) : `v=0, w=0` ne permet pas de savoir si le
contrôleur a publié zéro ou si la garde a rejeté une commande non nulle.

### Ce que révèle le code actuel

`caytu_course/caytu_course/noeud_garde_vitesse.py:114-140` regroupe sous un
seul avertissement toutes les raisons suivantes :

- mouvement interdit par l'état de course ;
- scan absent, invalide, périmé ou impossible à transformer ;
- commande périmée ;
- collision prédite sur l'empreinte balayée.

L'avertissement agrégé apparaît dès les premières secondes de la préparation
et revient ensuite à de nombreuses reprises
(`ros/log/python3_17329_1793994546119.log:1-67` et lignes suivantes). Pendant
la préparation, `mouvement_autorise()` renvoie normalement vrai
(`noeud_garde_vitesse.py:79-84`). Si la version exécutée correspond au code
actuel, les pistes prioritaires sont donc **obstacle, fraîcheur des données
ou TF capteur**, plutôt que « course inactive ». Le texte de l'avertissement
ne prouve pas quelle condition a bloqué une commande.

Deux mécanismes méritent une vérification ciblée :

1. **Désaccord géométrique Nav2 / garde.** La configuration actuelle ajoute
   3 cm de padding Nav2, tandis que la garde ajoute aussi 5 cm de marge et
   une marge d'échantillonnage. `securite.py:27-39` vérifie dès l'instant zéro :
   si un point est déjà dans cette empreinte élargie, toutes les trajectoires
   testées sont refusées, y compris un recul qui pourrait éloigner le robot.
   Cela peut entretenir un blocage après une approche d'obstacle. Effacer la
   costmap Nav2 ne supprime pas les points mesurés utilisés par la garde.
2. **Fraîcheur et horloges.** La garde contrôle l'âge des scans en temps ROS
   et le délai depuis leur réception en temps réel ; les commandes expirent
   également en temps réel. Le profil actuel donne 2 s aux scans et 0,5 s
   aux commandes. Une simulation lente ou une source caméra irrégulière peut
   donc provoquer des coupures même si Nav2 reste actif.

Ces mécanismes sont des **hypothèses étayées par le code**, pas une preuve
du déclencheur précis pendant l'essai. Les scans, costmaps et raisons détaillées
des veto ne figurent pas dans l'archive.

RPP rapporte 34 collisions imminentes sur environ 1,7 s
(`controller_server_17320_1793994541961.log:491-527`), puis le contrôleur
termine par `Failed to make progress` et efface sa costmap locale
(même journal, lignes 781-784). C'est une preuve de collision prédite par
RPP, **pas** la preuve d'un faux positif : sans scans et costmaps de cet
instant, l'obstacle réel, la géométrie et les données périmées ne peuvent pas
être départagés. Effacer la costmap ne corrige pas un veto provenant des scans
bruts de la garde.

### Ralentissement et récupération tardive

Les rotations de 6,28 rad prennent environ 92 s réelles chacune
(`ros/log/behavior_server_17322_1793994541818.log:18-23`). Les durées
d'immobilité du moniteur utilisent l'horloge ROS, alors que le budget de
préparation et les messages « secondes écoulées » utilisent le temps réel
dans le code actuel. Il ne faut donc pas comparer directement les « 24 s »
du moniteur aux quelque 4 min 30 de distance restante figée.

Le `progress_checker` actuel utilise une allocation de 15 s, évaluée avec
l'horloge du nœud Nav2. En simulation ralentie, la récupération arrive beaucoup
plus tard en temps réel, tandis que le budget de mission continue à s'épuiser.
L'archive ne permet pas de calculer un facteur temps réel précis sur tout
l'essai, ni de distinguer entièrement ralentissement et coupures de commande.

## 2. Match : attente d'ordre, pas de but Nav2

La carte est chargée et les nœuds sont actifs à 23:02:01 :
`fosa_simulation/journaux/20261106_224839_16629/course/course.log:164`.

À 23:02:12, le départ est reçu, puis le nœud revient immédiatement à :

```text
[ATTENTE_ORDRE] : départ déjà donné
```

Preuve : `ros/log/python3_28259_1793995319466.log:1-8`.
Aucun `Begin navigating` ne figure dans le journal du `bt_navigator_28251`.
`noeud_course.py:347-351` attend explicitement que l'ordre soit disponible.

**Cause immédiate certaine : aucun ordre valide n'a été accepté par le nœud
de course.** Le départ seul ne lance pas les déplacements.

Attention : le moniteur voit l'arbitre passer à `ARMED`, puis `RUNNING`
(`fosa_simulation/journaux/20261106_224839_16629/course/moniteur.log:4-7`).
Il serait donc prématuré d'affirmer que le juge n'a pas donné l'ordre.
Le journal du nœud de course ne contient même pas les annonces de découverte
des types `/parc/tag_order` et `/parc/run_status` présentes dans l'essai antérieur.

À vérifier : découverte et abonnement à `/parc/tag_order`, type et QoS,
ordre réellement publié et progression de `/clock`. Dans le code actuel,
la découverte automatique utilise un timer ROS
(`entrees_course.py:109-124`) : une horloge simulée arrêtée peut empêcher
la création de ces abonnements. Une pause de Gazebo est une piste, pas un fait
établi par ces journaux.

## 3. Relances : TF `odom` absente

Le pont Gazebo est arrêté à 23:03:34
(`ros/log/parameter_bridge_27609_1793995284174.log:23`), après la fin du
processus Gazebo signalée dans `course/monde.log:349-361` sous
`fosa_simulation/journaux/20261106_224839_16629/`.

Les relances vers 23:03:50 et 23:04:12 attendent ensuite :

```text
transform from base_link to odom
Invalid frame ID "odom" ... frame does not exist
```

La seconde termine après environ 60 s par :

```text
Failed to activate local_costmap because transform from base_link to odom
did not become available before timeout
```

Preuves : `ros/log/controller_server_30683_1793995430300.log:36-56`,
`ros/log/controller_server_31742_1793995452173.log:36-157`,
`ros/log/lifecycle_manager_31746_1793995452136.log:9-10`.

**Cause immédiate certaine : la chaîne TF nécessaire n'est pas disponible.**
Les nouveaux essais sont en mode sans carte, mais ce mode exige toujours
l'odométrie et sa TF. Le changement `base_footprint` → `base_link` est aussi
cohérent avec un lancement utilisant le profil robot au lieu du profil
simulation. Les variables d'environnement n'étant pas archivées, ce dernier
point reste à confirmer, notamment le domaine ROS utilisé.

## 4. Anomalies secondaires

- **Disponibilité Nav2 annoncée trop tôt.** À 22:49:15, Fosa annonce Nav2
  prêt ; `behavior_server` et `bt_navigator` refusent pourtant les premiers
  buts car leurs actions sont inactives. Le code actuel teste la présence
  des serveurs, sans vérifier leur état lifecycle `active`
  (`navigation_runtime.py:123-141`). Cela explique le premier tour raté et
  le premier point rejeté, mais pas le blocage plusieurs minutes plus tard.
- **Avertissement d'inflation Smac**, ligne 29 du
  `ros/log/planner_server_17321_1793994541871.log` : Smac ne trouve pas la
  couche ou juge son rayon insuffisant. La couche est pourtant initialisée
  aux lignes 25-26, puis le planner continue à fonctionner. Dans la
  configuration actuelle, le rayon global est de 0,90 m pour une empreinte
  d'environ 0,53 × 0,50 m ; cette valeur dépasse le minimum décrit par Smac.
  Mais les paramètres effectifs du lancement ne sont pas archivés. Il faut
  donc capturer le rayon, l'empreinte et la liste des plugins réellement
  chargés avant de changer cette valeur.
- **TF de démarrage transitoires.** Les extrapolations initiales disparaissent
  avant l'activation complète ; elles sont distinctes de l'absence persistante
  d'`odom` lors des relances.
- **Erreur à l'annulation.** `Failed to get result for follow_path in node halt!`
  survient après l'expiration du budget, pas au début de l'immobilité.
- **Arrêts explicites.** Les `KeyboardInterrupt` et sorties `-2` de la première
  course suivent les SIGINT ; ils ne démontrent pas un crash spontané à
  l'origine du problème.
- **Essai antérieur inclus dans l'archive.** Les PID 82xx correspondent à un
  lancement précédent, avant cette préparation : ordre `[4, 9, 1, 7]` bien
  reçu, mais 67 avertissements de collision, un `Controller patience exceeded`
  et trois `Failed to make progress`. Ils renforcent la piste d'un problème
  récurrent de suivi/garde, sans appartenir au même trajet.

## Priorités pour la prochaine reproduction

1. **Instrumenter la garde sans assouplir sa sécurité.** Distinguer et
   journaliser l'état de course, chaque source de scan (âge ROS, âge réel,
   échec TF), la fraîcheur de commande et le résultat de la collision prédite.
   Pour une collision, noter le premier point et l'instant de trajectoire qui
   la déclenchent.
2. **Rejouer le même scénario avec une capture synchronisée.** Enregistrer
   `/fosa/cmd_vel`, `/cmd_vel`, les scans, `/tf`, `/tf_static`, `/clock`,
   l'odométrie, `/plan`, les costmaps locales/globales et l'empreinte/rayons
   réellement chargés. Vérifier que les horodatages et les fenêtres couvrent
   un épisode complet d'immobilité.
3. **Corriger uniquement le déclencheur confirmé.** Si la garde rejette,
   comparer son empreinte balayée aux obstacles des scans et à celle de Nav2 ;
   si RPP publie déjà zéro, examiner la collision prédite et la costmap.
   Ne pas désactiver la détection de collision ni réduire les marges sans
   preuve et test de sécurité.
4. **Fiabiliser le démarrage Nav2.** Attendre l'état lifecycle actif, pas
   seulement la découverte des serveurs d'action ; vérifier que le premier
   `Spin` et le premier but sont acceptés avant de déclarer le robot prêt.
5. **Diagnostiquer séparément le match et les relances.** Confirmer la
   réception de `/parc/tag_order` avant le départ ; pour les relances,
   confirmer simulateur/pont, domaine ROS, profil et TF `odom` vers le repère
   robot attendu. Ne pas attribuer leur panne aux paramètres du contrôleur.
6. **Valider les réglages réellement déployés.** Capturer empreinte, couches
   et rayons d'inflation ; résoudre l'avertissement Smac à partir de ces
   valeurs, puis mesurer le temps réel de récupération lorsque `/clock` est
   ralenti.

Critères de sortie : plus aucun arrêt inexpliqué de la garde, un but de
préparation terminé ou refusé avec cause identifiable, démarrage sans rejet
d'action, et ordre de tags reçu avant toute navigation de match.

**Aucune réparation du code n'a été appliquée** : cette note fixe le diagnostic
et l'ordre des vérifications avant toute modification.

---

## Corrections appliquées le 7 novembre 2026

Le diagnostic ci-dessus reste la référence des incidents observés. Les
corrections correspondantes sont détaillées dans
[`docs/PLAN-CORRECTIONS-2026-11-07.md`](docs/PLAN-CORRECTIONS-2026-11-07.md) :

- **Arrêt et relance** : `run.sh` surveille la session au premier plan ;
  `Ctrl+C` l'arrête, `SIGHUP` la laisse continuer. Les PID sont identifiés par
  jeton et empreinte `starttime`. Un nouveau lancement remplace l'ancienne
  session au lieu de la refuser.
- **Carte et tags** : chemins relatifs résolus, dossier avec un seul
  `*.posegraph` accepté, fichiers vides refusés. Les tags sont exportés dans
  `map` via la TF finale `map → odom`, pour rester cohérents après une
  correction SLAM.
- **Garde** : distance signée à l'empreinte, tolérance de pénétration bornée
  (`penetration_tolerance_m`) et sources d'obstacles `required` ou
  optionnelles. Un recul lent qui diminue la pénétration reste autorisé.
- **Nav2** : état lifecycle `active` vérifié pour chaque nœud avant le premier
  but, au lieu de la seule présence des serveurs d'action.
- **Récupération** : après des échecs d'approche répétés, les costmaps sont
  vidées puis le robot recule lentement, dans la limite
  `max_unstuck_attempts` par tag. Aucun recul systématique n'est déclenché
  après une visite réussie ou après la mesure d'un tag en préparation.
- **Visites** : `tags_reported` est suivi, la confirmation attend
  `referee_confirmation_sec` sans bloquer la course, et le résumé final
  distingue les visites confirmées, non confirmées et sautées.
- **Voix** : préflight non bloquant dans `run.sh`, préparation automatique de
  Vosk par `deployer_robot.sh --envoyer`, et `FOSA_SANS_VOIX=1` pour déployer
  sans voix.

### À vérifier sur le robot

1. **Audio ReSpeaker** : `ros2 topic hz` sur le topic audio, puis vérifier que
   les octets PCM sont en 16 bits, 16 kHz, mono, sans en-tête (format attendu
   par `mot_depart`). Comparer le champ `sample_rate` de `start_word` à la
   sortie réelle de `respeaker_node`.
2. **Empreinte et centre** : mesurer l'empreinte réelle, `center_offset_x` et
   les TF caméra → `base_link` ; les valeurs actuelles viennent de la
   simulation.
3. **Pose de départ** : replacer le robot à la pose `map_start_pose` et
   vérifier que la localisation ne saute pas au démarrage.
4. **Confirmations** : jouer quatre tags, vérifier que `tags_reported` accuse
   chaque tag signalé, puis contrôler le résumé final (visites confirmées,
   non confirmées et sautées).
5. **Arrêt/relance** : `Ctrl+C`, puis relance immédiate ; répéter avec
   `./run.sh arreter` depuis un second terminal.
