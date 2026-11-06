# Équipe Fosa — PARC Engineers League 2026

Guide pratique pour préparer le PC, tester la solution dans Gazebo, la déployer sur le Jetson et effectuer les essais réels.

**Robot :** CAYTU Sito-É — Jetson AGX Orin — ROS 2 Jazzy  
**Workspace PC :** `~/ros2_ws`  
**Dossier du projet :** `~/ros2_ws/src/fosa-solution`  
**Compte SSH du robot :** `sitoe`

> Remplacez `ROBOT_IP` par l’adresse IP communiquée par les organisateurs.

---

## 1. Préparer le PC (une seule fois)

Le PC doit disposer de ROS 2 Jazzy, de Gazebo et des dépendances ROS utilisées par le projet.

### 1.1 Installer les dépendances

Après avoir installé ROS 2 Jazzy, installer les paquets nécessaires :

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-slam-toolbox \
  ros-jazzy-laser-filters ros-jazzy-ros-gz \
  ros-jazzy-depthimage-to-laserscan ros-jazzy-audio-common-msgs \
  ros-jazzy-teleop-twist-keyboard python3-opencv zip
```

### 1.2 Placer les dépôts

Dans `~/ros2_ws/src/`, placer :
- le dépôt de l’équipe : `fosa-solution` ;
- le dépôt officiel PARC 2026, qui fournit notamment le simulateur, l’arbitre et les messages ROS.

Conserver **une seule copie** de `fosa-solution` et des paquets ROS dans le workspace pendant la compilation. Des copies ou worktrees contenant les mêmes paquets peuvent provoquer des doublons dans `colcon`.

### 1.3 Compiler le workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 1.4 Préparer la simulation

```bash
cd ~/ros2_ws/src/fosa-solution
simulation/simulation.sh preparer
```

Cette commande construit les paquets nécessaires à la simulation et vérifie les dépendances.

### 1.5 Exécuter les tests

```bash
cd ~/ros2_ws
colcon test --packages-select caytu_config caytu_perception caytu_course caytu_nav_solution
colcon test-result --verbose
```

Vérifier qu’aucun test n’échoue.

**Après une modification du code**, reconstruire au minimum les paquets concernés :

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to caytu_course
```

---

## 2. Tester la solution dans Gazebo (sur le PC)

Toutes les commandes de cette section sont à exécuter sur le **PC**, depuis le dossier `fosa-solution`.

### 2.1 Préparation manuelle : construire la carte et mémoriser les tags

Cette procédure permet de piloter toi-même le robot dans Gazebo, avec le clavier, pendant que SLAM construit la carte et que les caméras mémorisent les AprilTags détectés.

Choisir une graine (ici `7`) et respecter l'ordre des étapes. Ouvrir plusieurs terminaux sur le PC et laisser les terminaux actifs ouverts jusqu'à la fin de la préparation.

**Terminal 1 — lancer le monde Gazebo :**

```bash
cd ~/ros2_ws/src/fosa-solution
simulation/simulation.sh monde 7
```

Attendre que Gazebo et le robot simulé soient démarrés.

**Terminal 2 — lancer l'enregistrement guidé :**

```bash
cd ~/ros2_ws/src/fosa-solution
FOSA_PROFIL=simulation ./run.sh preparation-manuelle
```

Attendre le message indiquant que la cartographie guidée est lancée. Ce mode ne donne aucun objectif autonome au robot : il attend que tu le pilotes.

**Terminal 3 — suivre les tags détectés :**

```bash
cd ~/ros2_ws/src/fosa-solution
FOSA_PROFIL=simulation ./run.sh suivi
```

Le suivi annonce les identifiants des tags détectés et mémorisés. Les observations suivantes sont fusionnées et sauvegardées en continu. Il faut donc parcourir l'arène et présenter les tags aux caméras pour qu'ils soient détectés.

**Terminal 4 — piloter le robot au clavier :**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Utiliser les touches affichées par le téléopérateur pour déplacer le robot. Explorer les couloirs et les zones utiles sans heurter les meubles. Les tags ne sont mémorisés que lorsque les caméras les détectent.

**Sauvegarder la carte et les tags :**

Quand l'exploration est suffisante :

1. Dans le terminal 4, arrêter d'abord le téléopérateur avec `Ctrl+C`.
2. Laisser la préparation et SLAM actifs.
3. Dans un autre terminal, lancer :

```bash
cd ~/ros2_ws/src/fosa-solution
FOSA_PROFIL=simulation ./run.sh sauver-carte
```

Vérifier que la commande confirme l'enregistrement de `carte.posegraph` et `carte.data`, puis affiche les tags mémorisés.

4. Arrêter ensuite la préparation :

```bash
FOSA_PROFIL=simulation ./run.sh arreter
```

5. Revenir au terminal 1 et arrêter le monde Gazebo avec `Ctrl+C`.

**Lancer un match sur cette préparation manuelle :**

Le match doit utiliser la même graine que celle de la préparation. Par exemple :

```bash
cd ~/ros2_ws/src/fosa-solution
simulation/simulation.sh course 7,1,9,4 7
```

Ici, `7,1,9,4` est l'ordre des tags et `7` la graine de l'arène. Le script relance Gazebo et l'arbitre. Par défaut, le départ est envoyé par commande : ne dites pas « allez ».

> Important : une nouvelle commande `preparation-manuelle` démarre une nouvelle préparation et archive l'ancienne. Les cartes de plusieurs séances ne sont pas fusionnées. Pense à mémoriser tous les tags nécessaires pendant la même séance.

### 2.2 Répétition complète automatique

```bash
cd ~/ros2_ws/src/fosa-solution
simulation/simulation.sh essai
```

Le script lance automatiquement :
1. le monde Gazebo et le robot simulé ;
2. la phase de préparation et l’enregistrement de la carte et des tags ;
3. un nouveau lancement du même monde ;
4. le match avec l’arbitre et un ordre de tags ;
5. le suivi et le bilan de la simulation.

Par défaut, le départ est envoyé automatiquement par commande : **ne dites pas « allez »**.

Le tableau de bord de l’arbitre est disponible sur `http://localhost:8080/`.

### 2.3 Rejouer une arène précise

Une même graine produit la même disposition des obstacles et des tags :

```bash
simulation/simulation.sh essai 7,1,9,4 12
```

- `7,1,9,4` : ordre des tags ;
- `12` : graine de l’arène.

Sans ordre indiqué, l’arbitre choisit l’ordre aléatoirement.

### 2.4 Tester le départ vocal (facultatif)

Préparer Vosk et le modèle français une seule fois :

```bash
simulation/simulation.sh voix-preparer
```

Puis lancer :

```bash
FOSA_SIM_VOIX=1 simulation/simulation.sh essai
```

Cette fois, attendre le message demandant le départ vocal, puis prononcer clairement **« allez »** dans le micro du PC.

### 2.5 Arrêter la simulation

```bash
simulation/simulation.sh arreter
```

Ne lancez pas deux simulations en même temps.

> La simulation aide à vérifier les interfaces et le comportement logiciel, mais ne garantit pas le comportement exact du robot physique ni de ses dispositifs de sécurité.

---

## 3. Déployer la solution sur le robot

Cette étape commence sur le **PC**. Le Jetson doit être allumé et accessible en SSH. Le script suppose que le workspace ROS de base du robot, `~/ros2_ws`, a déjà été préparé par les organisateurs.

### 3.1 Envoyer le projet depuis le PC

```bash
cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --envoyer ROBOT_IP
```

Le script :
- archive uniquement le dossier `src/` ;
- transfère l’archive au compte `sitoe` ;
- remplace `~/teams/fosa/src` sur le Jetson ;
- compile le projet dans `~/teams/fosa`.

Attendre le message **« Déploiement terminé »**. En cas d’erreur, la corriger avant de continuer.

### 3.2 Préparer le départ vocal (facultatif)

Si vous souhaitez utiliser Vosk sur le robot, préparer les fichiers compatibles avec le Jetson **sur le PC, avant le transfert** :

```bash
cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --preparer
./deployer_robot.sh --envoyer ROBOT_IP
```

Puis se connecter au robot et installer Vosk :

```bash
ssh sitoe@ROBOT_IP
cd ~/teams/fosa/src/fosa-solution
./deployer_robot.sh
```

La reconnaissance vocale est facultative. Sans elle, le départ peut être donné avec `./run.sh depart`.

### 3.3 Se connecter et vérifier le robot

Depuis le PC :

```bash
ssh sitoe@ROBOT_IP
cd ~/teams/fosa/src/fosa-solution
```

Lancer le diagnostic :

```bash
./run.sh diagnostic-robot
```

Vérifier les interfaces ROS, les caméras, le LiDAR, les transformations TF, l’audio et la communication avec l’arbitre. Ne commencez pas les essais si une anomalie importante reste inexpliquée.

---

## 4. Effectuer un essai réel sur le robot

Le robot doit commencer la préparation au **point de départ du match**. Pour tester la mission sur la carte enregistrée, il devra ensuite être replacé à ce même point.

### 4.1 Préparer la carte et mémoriser les tags

Dans un premier terminal SSH (**terminal 1**) :

```bash
./run.sh preparation-manuelle
```

Ce mode lance la cartographie et la mémorisation des tags, sans piloter lui-même le robot. Utilisez le moyen de téléopération autorisé pour guider le robot dans l’arène et faire détecter les tags par les caméras.

Dans un deuxième terminal SSH (**terminal 2**) :

```bash
ssh sitoe@ROBOT_IP
cd ~/teams/fosa/src/fosa-solution
./run.sh suivi
```

Le suivi indique les tags détectés et mémorisés.

### 4.2 Enregistrer la carte

Quand la cartographie est suffisante :

1. Arrêter d’abord le téléopérateur.
2. **Laisser la préparation et SLAM actifs.**
3. Dans un troisième terminal SSH (**terminal 3**), exécuter :

```bash
ssh sitoe@ROBOT_IP
cd ~/teams/fosa/src/fosa-solution
./run.sh sauver-carte
```

Vérifier que la sauvegarde confirme :
- `carte.posegraph` ;
- `carte.data` ;
- les identifiants des tags mémorisés dans `tags.yaml`.

Ensuite, arrêter la préparation :

```bash
./run.sh arreter
```

### 4.3 Lancer la mission

Replacer le robot à son point de départ.

Dans un terminal SSH :

```bash
./run.sh course
```

Vérifier que le message indique que la préparation enregistrée est utilisée. La course attend ensuite l’ordre des tags et le signal de départ.

L’ordre officiel doit être communiqué par l’organisateur. Une fois l’ordre reçu, donner le départ :

**Par commande :**

```bash
./run.sh depart
```

**Ou par la voix :** prononcer « allez » près du robot si la reconnaissance vocale a été préparée.

Garder `./run.sh suivi` ouvert dans un autre terminal pour observer le déroulement.

Après l’essai, arrêter la course si elle tourne encore :

```bash
./run.sh arreter
```

---

## 5. Ranking match

Le ranking reprend la procédure d’essai réel, avec les durées et les consignes de la compétition : **15 minutes de préparation**, puis **10 minutes de match**.

### Préparation — 15 minutes

1. Placer le robot au point de départ.
2. Lancer `./run.sh preparation-manuelle`.
3. Guider le robot pour construire la carte et détecter les tags.
4. Surveiller les détections avec `./run.sh suivi`.
5. Avant la fin de la préparation, arrêter le téléopérateur puis exécuter `./run.sh sauver-carte` pendant que SLAM tourne encore.
6. Vérifier les fichiers de carte et les tags mémorisés.
7. Arrêter la préparation avec `./run.sh arreter`.
8. Laisser ou replacer le robot au point de départ utilisé pour la préparation.

**Ne relancez pas `preparation-manuelle` après la sauvegarde** : une nouvelle préparation repart de zéro et archive la précédente.

### Match — 10 minutes

1. Lancer `./run.sh course`.
2. Vérifier que la carte préparée est chargée.
3. Attendre l’ordre officiel communiqué par les organisateurs.
4. Donner le départ avec « allez » ou `./run.sh depart`.
5. Laisser le robot exécuter la mission et surveiller son état.

Pendant le match, ne publiez pas de faux ordre et ne relancez pas la préparation. N’utilisez pas `ordre-test` : cette commande est réservée aux essais.

---

## 6. Commandes utiles

| Commande | Utilité |
|---|---|
| `simulation/simulation.sh preparer` | Préparer Gazebo sur le PC |
| `simulation/simulation.sh essai` | Répétition complète sur le PC |
| `simulation/simulation.sh arreter` | Arrêter la simulation |
| `./deployer_robot.sh --envoyer ROBOT_IP` | Envoyer et compiler le projet sur le Jetson |
| `./deployer_robot.sh` | Préparer Vosk sur le robot si les fichiers ont été transférés |
| `./run.sh diagnostic-robot` | Vérifier les interfaces du robot |
| `./run.sh preparation-manuelle` | Cartographier et mémoriser les tags en guidant le robot |
| `./run.sh preparation` | Lancer la préparation avec exploration autonome |
| `./run.sh suivi` | Suivre le déroulement de la solution |
| `./run.sh sauver-carte` | Enregistrer la carte et vérifier les tags |
| `./run.sh course` | Lancer le match |
| `./run.sh depart` | Donner le départ par commande |
| `./run.sh etat` | Afficher l’état brut de la course |
| `./run.sh arreter` | Arrêter la préparation ou la course |

### Rappels importants

- `simulation/simulation.sh` s’utilise sur le **PC uniquement**.
- `./deployer_robot.sh --envoyer ROBOT_IP` s’utilise sur le **PC**.
- `./deployer_robot.sh` sans argument s’utilise sur le **robot**.
- `./run.sh ...` s’utilise sur le robot par défaut. Sur le PC, il faut explicitement utiliser `FOSA_PROFIL=simulation`.
- Les paquets ROS système manquants sur le Jetson doivent être signalés aux organisateurs : le déploiement ne fait pas d’installation système avec `sudo`.
- La préparation manuelle nécessite un téléopérateur séparé ; `preparation-manuelle` ne déplace pas le robot de lui-même.
