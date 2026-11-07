# Plan de correction — 7 novembre 2026

> **Document de reprise.** Il contient le diagnostic, les décisions et la
> checklist des corrections. Si une session d’agent s’arrête avant la fin,
> reprendre ici : chaque point indique les fichiers concernés, le comportement
> attendu et les tests à exécuter.

- **Branche de travail** : `integration/finale-propre-2026-11-06`
- **Commit de départ de l’audit** : `ae3d5c6`
- **Objectif** : rendre le parcours « préparation manuelle → carte + tags →
  match par commande ou par voix » aussi fiable que possible sur le robot
  réel, sans dégrader la simulation.
- **État** : volets A à H terminés (processus, carte, garde, lifecycle Nav2,
  mémoire des tags, récupération, voix, confirmations). Volet I : documentation
  mise à jour, tests purs et ROS 2 natifs passants ; validation robot restant
  à exécuter.

---

## 1. Périmètre et contraintes

### Ce qui doit fonctionner

1. Lancer une préparation, guider le robot au clavier, enregistrer la carte et
   les positions des quatre AprilTags.
2. Arrêter proprement la session, puis relancer un match sans être bloqué par
   une fausse « session déjà active ».
3. Recharger automatiquement la bonne carte et les tags associés.
4. Naviguer vers les tags dans l’ordre de l’arbitre, s’aligner à 0,5 m et
   signaler chaque visite.
5. Accepter un départ par commande (`./run.sh depart`) ou par la voix
   (« allez ») lorsque Vosk et le micro sont réellement installés.
6. En cas d’échec, repartir ou dégager le robot au lieu de rester immobile.

### Contraintes

- **ROS 2 Jazzy n’est pas installé sur le PC d’analyse** (`/opt/ros` absent).
  Les tests exécutables ici sont les tests Python purs. Nav2, Gazebo, slam_toolbox
  et les services ROS doivent être validés sur le robot ou dans le conteneur.
- Ne pas renommer les interfaces ROS déjà publiées : topics, types de messages,
  services, noms de paquets, clés YAML existantes.
- Les valeurs métier (seuils, topics, durées, topics audio, chemins de modèles)
  restent dans `caytu_config`, jamais codées en dur dans les nœuds.
- Les corrections doivent rester compatibles avec le profil
  `caytu_config/config/profils/simulation.yaml`.

### Commandes de test utilisées

```bash
env PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=caytu_config:caytu_course:caytu_perception:caytu_nav_solution \
  python3 -B -m pytest -p no:cacheprovider -q \
  caytu_course/test caytu_config/test caytu_nav_solution/test \
  caytu_perception/test outils/test
```

Les tests ROS natifs ont été exécutés sur ce PC avec ROS 2 Jazzy :

```bash
./outils/verifier.sh
```

Les tests d'intégration qui publient des commandes moteur restent réservés à
un réseau isolé. Ils peuvent être exécutés en domaine dédié et localhost :

```bash
set +u
source /opt/ros/jazzy/setup.bash
source "${FOSA_WS:-$HOME/ros2_ws}/install/setup.bash"
set -u
FOSA_TEST_ROS_ISOLE=1 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest -q outils/test/test_ros_integration.py
```

Le build complet se vérifie avec `./simulation/simulation.sh preparer` (dix
paquets Fosa et officiels).

---

## 2. Diagnostic consolidé

### 2.1 Arrêt, Ctrl+C et relance — **bloquant**

**Symptôme rapporté** : après `Ctrl+C` dans le terminal de lancement, puis
`./run.sh arreter`, le lancement suivant dit encore qu’une session Fosa est
active.

**Causes identifiées dans le code :**

1. `run.sh:258-285` lance la course **détachée** (`start_new_session=True`
   dans `outils/processus.py:205-209`). Le `Ctrl+C` du terminal ne peut donc
   pas atteindre les processus ROS : le shell revient à l’invite et la session
   continue.
2. `outils/processus.py:122-137` fait confiance au PID racine de l’état même
   quand ce PID a été réutilisé par un autre programme. Les descendants du
   nouveau PID sont alors considérés comme membres de Fosa. Reproduit en
   mémoire : un PID racine réutilisé, sans jeton Fosa, classe quand même ses
   descendants comme membres.
3. `run.sh:122-129` (`signaler_course_ancienne`) utilise `pgrep -af` sur
   `course_parc|course.launch.py` sans vérifier le workspace. Un éditeur, un
   `grep`, un `less` ou un script ouvert sur `course.launch.py` peut être
   signalé à tort comme processus Fosa.
4. Si `processus.py arreter` échoue sur un faux positif, il lève une exception
   avant de supprimer le fichier d’état. `actif` continue donc de répondre
   « oui » et le lancement suivant est refusé.
5. `outils/processus.py:199-200` refuse un nouveau lancement dès qu’un membre
   est détecté, sans distinguer un état périmé d’une vraie session.

**Correction attendue :**

- Vérifier l’identité du PID racine par un **empreinte de démarrage**
  (`/proc/<pid>/stat`, champ `starttime`) en plus du jeton.
- Ne plus inclure les descendants d’un PID réutilisé.
- Ajouter une action `lister` à `outils/processus.py` et remplacer le `pgrep`
  large de `run.sh`.
- Toujours supprimer l’état quand aucun membre réel ne reste.
- **Un nouveau lancement remplace toute session ou tout nœud Fosa déjà en
  cours** : arrêt automatique de l’ancienne session avant de démarrer, jamais
  de refus « session déjà active ».
- Garder `run.sh` en **surveillance de premier plan** après le lancement :
  - `Ctrl+C` → arrêt contrôlé de la session ;
  - fermeture du terminal / coupure SSH (`SIGHUP`) → la session détachée
    continue, avec un message explicite ;
  - `./run.sh arreter` depuis un autre terminal reste disponible.
- Le moniteur de session utilise le jeton de sa session : un ancien terminal
  ne peut pas surveiller ni arrêter une nouvelle session.
- Afficher les PID et commandes restants en cas d’échec.

### 2.2 Chargement de la carte — **bloquant**

**Défaut reproduit :** depuis la racine du dépôt, la carte livrée n’est pas
reconnue.

- `caytu_config/config/fosa.yaml:252` : `carte_embarquee: cartes/arene_2026_11_06`
- `caytu_course/caytu_course/lancement.py:27-34` : `_base_carte` teste
  `is_dir()` **avant** de résoudre un chemin relatif.
- `lancement.py:42-51` : le chemin relatif n’est résolu qu’après.
- Résultat reproduit : le code cherche
  `config/cartes/arene_2026_11_06.posegraph` au lieu de
  `config/cartes/arene_2026_11_06/carte.posegraph` ; `carte_preparee()`
  retourne `None`.
- Conséquence : `./run.sh course` bascule en mode sans carte alors que la
  carte livrée est présente.
- La carte livrée n’a **aucun `tags.yaml`** : même correctement chargée, elle
  ne fournit pas les positions des tags.
- `lancement.py:112-115` impose `map_start_pose = [0, 0, 0]` : le robot doit
  être replacé à la position et à l’orientation du début de la cartographie.

**Correction attendue :**

- Résoudre le chemin relatif (contre `config_directory()` pour la carte
  embarquée, contre le répertoire courant pour `FOSA_CARTE`) **avant** de
  tester le dossier.
- Accepter un dossier contenant un seul `*.posegraph`, sinon `carte.posegraph`.
- Vérifier que `.posegraph` et `.data` existent et ne sont pas vides.
- Annoncer clairement la carte utilisée et la présence ou l’absence de
  `tags.yaml`.
- Rendre la pose de départ configurable (`map_start_pose`) et l’afficher.
- Ajouter des tests de régression pour les chemins relatifs, absolus, les
  fichiers incomplets et la priorité préparation > carte livrée.

### 2.3 Garde de vitesse — **bloquant par immobilisation**

- `caytu_course/caytu_course/securite.py:27-39` teste la collision **à
  l’instant zéro** avant tout déplacement.
- Si le robot est déjà dans la marge de sécurité (par exemple un obstacle
  latéral à 30 cm avec une demi-largeur élargie d’environ 33 cm), l’avance, le
  recul et la rotation sont tous refusés. Reproduit avec la configuration
  actuelle.
- `noeud_garde_vitesse.py:127-140` exige que **toutes** les sources soient
  fraîches. Une seule caméra de profondeur absente ou périmée immobilise le
  robot, même si le lidar et l’autre caméra fonctionnent.
- Le message d’arrêt mélange « course inactive », « obstacle » et « données
  périmées » : impossible de savoir pourquoi le robot s’est arrêté.

**Correction attendue :**

- Calculer une distance signée à l’empreinte et n’interdire que les
  trajectoires qui **aggravent** une pénétration déjà présente. Une trajectoire
  qui conserve ou augmente la distance est autorisée (échappement lent).
- Ajouter un seuil de tolérance de pénétration dans `caytu_config`.
- Distinguer les sources **obligatoires** (lidar) des sources **optionnelles**
  (caméras) : une caméra absente ne doit pas immobiliser le robot si le lidar
  est frais ; le lidar absent doit arrêter le robot.
- Journaliser la cause exacte : commande périmée, scans périmés, source
  obligatoire absente, obstacle sur la trajectoire, course inactive.
- Tests : obstacle latéral, obstacle devant/derrière, recul de dégagement,
  source optionnelle absente, source obligatoire absente.

### 2.4 Nav2 prêt sans vérification du lifecycle — **important**

- `caytu_nav_solution/caytu_nav_solution/navigation_runtime.py:126-141`
  vérifie seulement la présence des serveurs d’action et une pose robot.
- Les nœuds `controller_server`, `planner_server`, `behavior_server`,
  `bt_navigator` et `slam_toolbox` peuvent être encore `inactive` : les
  premiers buts sont alors refusés alors que la course les croit prêts.
- `DIAGNOSTIC.md` relève déjà ce risque.

**Correction attendue :**

- Interroger les services lifecycle `/<noeud>/get_state` avant de déclarer
  Nav2 prêt.
- Vérifier l’état `ACTIVE` (id 3) pour chaque nœud géré, en excluant
  `slam_toolbox` en mode sans carte.
- Journaliser les nœuds non actifs et ne pas envoyer de but avant qu’ils le
  soient.
- Ajouter `lifecycle_msgs` aux dépendances de `caytu_nav_solution`.
- Test pur : liste des nœuds attendus selon le mode.

### 2.5 Positions des tags après corrections SLAM — **important**

- `entrees_course.py:225-238` ajoute les détections dans `map` au moment de
  l’observation.
- `memoire_tags.py:59-105` moyenne ces coordonnées `map`.
- Si slam_toolbox corrige la carte après une fermeture de boucle, les
  observations déjà accumulées ne sont **pas** recalculées : la carte finale
  et `tags.yaml` peuvent être décalés.
- `noeud_course.py:937-944` exporte directement cette mémoire.
- Pendant le match, `_pose_tag_courante` (`noeud_course.py:558-580`) utilise
  une détection fraîche quand elle existe, ce qui rattrape une partie du
  problème ; ce n’est pas le cas des positions enregistrées.

**Correction attendue :**

- Tenir une mémoire parallèle dans le repère `odom`, alimentée par
  `pose_robot` et la TF `odom → base_frame` à l’instant de la détection.
- À l’enregistrement, transformer cette mémoire par la TF courante
  `map → odom` et exporter le résultat dans `map`.
- Conserver une repli sur la mémoire `map` si la TF ou la mémoire `odom` est
  indisponible, avec un avertissement clair.
- Tests : translation/rotation d’une mémoire, conservation des poids,
  fusion avec les tags déjà présents dans `map`.

### 2.6 Échecs d’approche répétés — **important**

- `noeud_course.py:389-395` compte les échecs et replanifie, mais ne dégage
  pas systématiquement le robot après des échecs d’approche.
- `_degager` existe (`noeud_course.py:721-748`) et est utilisé après une
  mesure de tag, une visite, ou des échecs de **recherche**
  (`noeud_course.py:516-520`), pas après des échecs d’**approche**.
- Les costmaps ne sont pas vidées après un échec, alors qu’un obstacle mobile
  peut rester marqué.

**Correction attendue :**

- Après `unstuck_after_failures` échecs d’approche/alignement, vider les
  costmaps puis exécuter `_degager`.
- Conserver la cible si elle est encore rentable ; sinon la stratégie la
  saute comme aujourd’hui.
- Éviter les boucles infinies : borner les dégagements et journaliser.

### 2.7 Voix et Vosk — **conditionnel**

- `noeud_mot_depart.py:55-69` charge Vosk et le modèle ; sans eux, il
  s’arrête proprement et le départ par commande reste disponible.
- Dans cette copie du dépôt, le modèle `models/vosk-model-small-fr-0.22`
  est absent.
- `deployer_robot.sh:99-114` installe la voix seulement si
  `caytu_config/config/livraison/roues` et le zip du modèle existent ; sinon
  il affiche « Voix non préparée ».
- `deployer_robot.sh --envoyer` ne prépare pas automatiquement ces fichiers.
- `fosa.yaml:260-262` suppose PCM 16 bits, 16 kHz, mono, une phrase par
  message ; la fréquence est encore « À CONFIRMER ».

**Correction attendue :**

- Préflight dans `run.sh course` : dire clairement si Vosk, le modèle et le
  topic audio sont prêts, sans empêcher le départ par commande.
- `deployer_robot.sh --envoyer` prépare automatiquement la voix si elle
  manque (avec possibilité de l’en désactiver explicitement).
- Documenter la vérification du format audio sur le robot.
- Ne pas promettre le bonus vocal tant que le ReSpeaker réel n’a pas été
  testé.

### 2.8 Visites et accusés de réception de l’arbitre — **important**

- `entrees_course.py:204-222` lit `RunStatus` mais n’exploite pas
  `tags_reported`.
- `statut.py:94-114` publie `TagVisit` et considère la visite faite
  localement.
- `_valider_visite` (`noeud_course.py:793-824`) maintient la pose 5 s mais ne
  vérifie pas que l’arbitre a bien reçu le tag.
- Le résumé final peut donc dire « terminée » alors que des tags ont été
  sautés ou que l’arbitre n’a pas confirmé une visite.

**Correction attendue :**

- Suivre `tags_reported` dans `EntreesCourse`.
- Après le maintien, attendre un court délai de confirmation et journaliser
  « confirmée » ou « non confirmée ».
- Utiliser un QoS compatible avec le protocole officiel (`reliable`,
  `transient_local`) pour `RunStatus`.
- Ajouter au résumé final le nombre de visites confirmées et la distinction
  « mission terminée » / « quatre visites acceptées ».
- Ne pas bloquer la course indéfiniment si l’arbitre ne confirme pas : la
  décision des juges reste séparée.

### 2.9 Stratégie et quatre tags — **limite de conception**

- `strategie.py` peut volontairement sauter un tag pour maximiser les points.
- Avec les valeurs actuelles, quatre tags inconnus coûtent
  `4 × (90 + 45) × 1,20 = 648 s`, au-delà des 595 s utiles. Le plan initial
  ne retient donc que trois tags.
- Une préparation avec les quatre tags localisés change ce calcul ; c’est la
  voie recommandée.
- Le programme peut terminer avec des tags sautés : ce n’est pas un bug, mais
  il faut le dire clairement dans le résumé et la documentation.

### 2.10 Validation physique — **hors code**

- `fosa.yaml:37-43` : l’empreinte est celle du URDF simulé, à mesurer sur le
  robot réel.
- `center_offset_x: 0.0` : à vérifier.
- La calibration des caméras, les TF, la taille réelle du carré noir et le
  format audio du ReSpeaker doivent être validés sur le robot.
- Aucun essai physique n’a été réalisé pendant cet audit.

---

## 3. Décisions d’implémentation

| Sujet | Décision |
|---|---|
| Arrêt | `run.sh` reste au premier plan après le lancement et intercepte `Ctrl+C` ; `SIGHUP` laisse la session détachée continuer. |
| Identité des processus | Jeton `FOSA_SESSION_ID` + empreinte `starttime` du PID racine. Un PID réutilisé n’est jamais suivi. |
| Carte | Résolution du chemin avant `is_dir()`, validation des deux fichiers non vides, diagnostic explicite. |
| Pose de départ | Ajouter `course.preparation.map_start_pose` (liste de 3 nombres) et une surcharge `FOSA_POSE_DEPART`. |
| Garde | Distance signée à l’empreinte ; une trajectoire qui n’aggrave pas la pénétration est autorisée. |
| Sources d’obstacles | Champ `required` : lidar obligatoire, caméras optionnelles. Une source optionnelle absente est ignorée avec avertissement. |
| Nav2 | Vérification `lifecycle_msgs/GetState` avant le premier but. |
| Tags | Mémoire `odom` parallèle, export corrigé par `map → odom` au moment de la sauvegarde. |
| Visites | Suivi de `tags_reported`, confirmation non bloquante, résumé honnête. |
| Voix | Préflight + préparation automatique au déploiement ; aucune promesse sans test ReSpeaker. |

---

## 4. Ordre de travail

1. **A. Processus / arrêt / relance** — indépendant, testable ici.
2. **B. Cartes** — indépendant, testable ici.
3. **C. Garde de vitesse** — testable ici.
4. **D. Lifecycle Nav2** — testable partiellement ici.
5. **E. Mémoire des tags / SLAM** — testable ici.
6. **F. Récupération après échecs d’approche** — logique de course.
7. **G. Voix / déploiement** — scripts + préflight.
8. **H. Visites / arbitre** — suivi et résumé.
9. **I. Documentation et tests complets.**

---

## 5. Fichiers à modifier

| Fichier | Raison |
|---|---|
| `outils/processus.py` | Identité PID, `starttime`, action `lister`, nettoyage d’état. |
| `outils/test/test_processus.py` | Tests PID réutilisé, état périmé, arrêt réel. |
| `run.sh` | Surveillance premier plan, `Ctrl+C`, `lister`, messages carte/voix. |
| `caytu_course/caytu_course/lancement.py` | Résolution de carte, validation, pose de départ. |
| `caytu_course/test/test_lancement.py` | Tests chemins et validation. |
| `caytu_config/caytu_config/modeles_course.py` | `map_start_pose`, `required`, tolérance de pénétration, confirmation arbitre. |
| `caytu_config/caytu_config/models.py` | Champ `required` des sources. |
| `caytu_config/config/fosa.yaml` | Nouvelles valeurs métier. |
| `caytu_config/test/test_config_course.py` | Validation des nouvelles clés. |
| `caytu_course/caytu_course/securite.py` | Distance signée / échappement. |
| `caytu_course/test/test_securite.py` | Nouveaux cas de blocage/dégagement. |
| `caytu_course/caytu_course/noeud_garde_vitesse.py` | Sources obligatoires/optionnelles, logs précis. |
| `caytu_nav_solution/caytu_nav_solution/navigation_runtime.py` | Lifecycle Nav2. |
| `caytu_nav_solution/caytu_nav_solution/vie_nav2.py` | Liste partagée des nœuds lifecycle. |
| `caytu_nav_solution/package.xml` | `lifecycle_msgs`. |
| `caytu_nav_solution/test/test_vie_nav2.py` | Liste des nœuds lifecycle (test pur). |
| `caytu_course/launch/course.launch.py` | Liste des nœuds lifecycle partagée. |
| `caytu_course/caytu_course/memoire_tags.py` | Transformation et fusion d’une mémoire. |
| `caytu_course/test/test_memoire_tags.py` | Tests de transformation et de fusion. |
| `caytu_course/caytu_course/entrees_course.py` | Mémoire odom, `tags_reported`, QoS. |
| `caytu_course/caytu_course/noeud_course.py` | Export corrigé, dégagement après échecs, confirmation. |
| `caytu_course/test/test_entrees_course.py` | Mise à jour du faux nœud et mémoire odom, `tags_reported`. |
| `caytu_course/caytu_course/recit.py` | Résumé final avec visites confirmées et motif. |
| `caytu_course/test/test_recit.py` | Tests du résumé confirmé / non confirmé / interrompu. |
| `deployer_robot.sh` | Préparation automatique de la voix. |
| `README.md` | Procédure d’arrêt, carte, voix, limites. |
| `DIAGNOSTIC.md` | Mise à jour du diagnostic après corrections. |

---

## 6. Checklist

### A. Processus / arrêt / relance

- [x] Empreinte `starttime` stockée dans l’état de session.
- [x] `membres()` ne suit plus les descendants d’un PID réutilisé.
- [x] État périmé supprimé automatiquement.
- [x] Action `lister` disponible et utilisée par `run.sh`.
- [x] `run.sh` intercepte `Ctrl+C` et arrête la session.
- [x] `SIGHUP` laisse la session détachée continuer.
- [x] `./run.sh arreter` vérifie les vrais processus Fosa.
- [x] Un nouveau lancement arrête automatiquement l’ancienne session.
- [x] Tests `test_processus.py` ajoutés et passants.

### B. Cartes

- [x] `_base_carte` résout les chemins relatifs avant `is_dir()`.
- [x] Dossier avec un seul `.posegraph` accepté.
- [x] Fichiers vides refusés.
- [x] `run.sh` annonce la carte et les tags réellement utilisés.
- [x] `map_start_pose` configurable et affiché.
- [x] Tests `test_lancement.py` ajoutés et passants.

### C. Garde de vitesse

- [x] Distance signée et politique de non-aggravation.
- [x] Tolérance de pénétration dans `caytu_config`.
- [x] Sources `required` / optionnelles.
- [x] Logs de cause précis.
- [x] Tests `test_securite.py` ajoutés et passants.

### D. Nav2

- [x] Vérification `GetState` des nœuds lifecycle.
- [x] Liste des nœuds adaptée au mode `sans_carte`.
- [x] `lifecycle_msgs` déclaré.
- [x] Test de la liste des nœuds.

### E. Tags / SLAM

- [x] Mémoire `odom` alimentée par les détections.
- [x] Transformation `map → odom` à l’export.
- [x] Repli sur la mémoire `map` avec avertissement.
- [x] Tests de transformation.

### F. Récupération

- [x] Vidage des costmaps après échec d’approche.
- [x] `_degager` après échecs répétés.
- [x] Journalisation et bornes (`max_unstuck_attempts`).

### G. Voix

- [x] Préflight `run.sh` (Vosk, modèle, topic).
- [x] `deployer_robot.sh --envoyer` prépare la voix.
- [x] Documentation du format audio à vérifier (README, DIAGNOSTIC).

### H. Visites / arbitre

- [x] `tags_reported` suivi.
- [x] Confirmation non bloquante après le maintien.
- [x] Résumé avec visites confirmées / sautées.
- [x] QoS `RunStatus` aligné sur le protocole.

### I. Documentation et validation

- [x] `README.md` mis à jour.
- [x] `DIAGNOSTIC.md` mis à jour.
- [x] Tests purs complets passants.
- [x] Tests ROS 2 natifs exécutés : 280 passés, 4 sautés.
- [x] Tests d'intégration ROS exécutés en isolation locale : 2 passés.
- [x] Procédure de validation robot écrite.
- [ ] Validation exécutée sur le robot (matériel et capteurs réels indisponibles ici).

---

## 7. Validation sur le robot (après les corrections)

1. **Arrêt/relance** : lancer `./run.sh course`, faire `Ctrl+C`, vérifier que
   le robot s’arrête et que `./run.sh course` relance sans message de session
   active. Répéter avec `./run.sh arreter` depuis un second terminal.
2. **Carte** : lancer `./run.sh course` sans préparation locale et vérifier
   que la carte livrée est annoncée ; sinon, vérifier le chemin avec
   `FOSA_CARTE`.
3. **Préparation manuelle** : `./run.sh preparation-manuelle`, téléopérer,
   `./run.sh sauver-carte`, `./run.sh arreter`. Vérifier `carte.posegraph`,
   `carte.data`, `tags.yaml` et les quatre IDs.
4. **Localisation** : replacer le robot exactement à la pose de départ,
   lancer le match, vérifier que Nav2 devient actif et que le robot se localise
   sans sauter.
5. **Garde** : approcher un obstacle latéral, vérifier que le robot peut
   reculer ou tourner pour se dégager au lieu de rester bloqué.
6. **Visites** : jouer un ordre de quatre tags, vérifier les confirmations
   `tags_reported` et le résumé final.
7. **Voix** : vérifier `python3 -c 'import vosk'`, le modèle, puis dire
   « allez » après réception de l’ordre. Vérifier le champ `heard` et le bonus.
8. **Obstacles déplacés** : déplacer un meuble entre la préparation et le
   match, vérifier que la costmap se met à jour et que le robot contourne.
9. **Absence de recul après un tag** : visiter un tag puis vérifier que le
   robot repart vers le suivant sans reculer systématiquement. En préparation,
   contrôler qu'après une mesure de près il reste sur place pendant les
   dernières observations avant de repartir.

---

## 8. Journal des décisions

- **7 novembre 2026** : audit initial, reproduction du défaut de carte et du
  blocage de la garde. Décision de corriger d’abord l’arrêt/relance, la carte,
  la garde et la mémoire des tags, puis la voix et les confirmations.
- **7 novembre 2026** : ajout de ce document de reprise pour permettre à une
  autre session de continuer sans relire tout l’historique.
- **7 novembre 2026** : reprise de la branche `fix/sans-recul-apres-tag`
  (commit `36a1570`). Le recul n'est plus déclenché après une visite ni après
  la mesure d'un tag ; le robot s'arrête pendant les dernières observations.
  Seul le commit utile a été repris, sans le merge qui supprimait
  `laser_filter_params.yaml`. Tests purs : 238 passés, 6 sautés.
- **7 novembre 2026** : validation locale sur ROS 2 Jazzy natif avec
  `./outils/verifier.sh` : 280 tests passés, 4 sautés (DDS et intégration
  réservés au conteneur). Les deux tests de `test_ros_integration.py` ont été
  exécutés en domaine isolé et passent après alignement du test de la garde
  sur les sources `required` / optionnelles. `./simulation/simulation.sh
  preparer` reconstruit les dix paquets sans erreur. La validation physique
  sur le robot reste seule ouverte.
- **7 novembre 2026** : `./run.sh course` remplace toujours sa propre session
  Fosa, mais ne confond plus un `ros2 launch ... course.launch.py` homonyme
  d'une autre équipe avec les nôtres. La détection exige un marqueur du
  workspace (cwd, environnement ou exécutable sous `build`/`install`), et
  `deployer_robot.sh --installer` utilise la même liste. Les processus des
  autres équipes ne sont jamais arrêtés. Tests de régression ajoutés dans
  `outils/test/test_processus.py`, dont l'arrêt d'un enfant orphelin placé
  dans une autre session ; `./outils/verifier.sh` passe à 284 tests réussis,
  4 sautés.
- **7 novembre 2026** : README réorganisé en ordre chronologique (déploiement,
  robot, cartographie manuelle, simulation, aide). Chaque commande tient sur
  une ligne avec un commentaire court à droite.
- **7 novembre 2026** : `deployer_robot.sh --envoyer` remplace désormais toute
  l'installation du robot au lieu d'empiler les livraisons. La nouvelle
  livraison est extraite et compilée à part ; l'ancienne reste active en cas
  d'échec d'arrêt ou de compilation. Après succès, `livraisons/` ne contient
  plus que la nouvelle et `donnees/` (cartes, tags, sessions) est conservé.
  `outils/livraison.py activer` effectue le remplacement, refuse tout chemin
  hors de `livraisons/` et tout dossier inattendu à la place de `actuel`.
  Trois tests ajoutés ; `./outils/verifier.sh` passe à 287 tests réussis,
  4 sautés.
