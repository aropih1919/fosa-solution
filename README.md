# Fosa — PARC 2026

## Déployer depuis le PC

```bash
cd ~/Solution/ros2_ws/src/fosa-solution
./deployer_robot.sh --envoyer 192.168.1.3
```

## Installer sur le robot

```bash
ssh sitoe@192.168.1.3
source /opt/ros/jazzy/setup.bash
source ~/parc_ws/install/setup.bash   # ou ~/ros2_ws/install/setup.bash
cd ~/teams/fosa/actuel
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select caytu_config caytu_interfaces caytu_nav_bringup \
  caytu_nav_solution caytu_perception caytu_course --executor sequential
source install/setup.bash
cd src/fosa-solution
```

## Cartographier manuellement

Terminal 1 :
```bash
./run.sh preparation-manuelle
```

Terminal 2 :
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/fosa/cmd_vel
```

Après le guidage, arrêter le téléopérateur (`Ctrl+C`), puis dans le terminal 1 :
```bash
./run.sh sauver-carte
./run.sh arreter
```

`Ctrl+C` dans le terminal de lancement arrête aussi la session et le robot.
Fermer le terminal (ou couper SSH) laisse au contraire la session détachée
continuer ; `./run.sh arreter` reste disponible depuis un autre terminal.
Un nouveau lancement arrête automatiquement toute session Fosa précédente.

Les fichiers sont enregistrés dans `~/teams/fosa/donnees/preparation`.
La carte et `tags.yaml` sont ensuite rechargés automatiquement par le match.

## Lancer le match

Placez le robot à la position et orientation de départ de la carte.

```bash
./run.sh course
./run.sh depart         # ou dire « allez »
./run.sh suivi          # suivi, dans un autre terminal si souhaité
./run.sh arreter
```

Au lancement, `run.sh` vérifie sans bloquer que Vosk, le modèle et le topic
audio sont disponibles, puis annonce la carte, les tags et la pose de départ
utilisés. Le départ par commande reste toujours possible ; le départ vocal
demande en plus de prononcer un mot de `start_word.words` (par défaut
« allez »).

Pour déployer sans préparer la voix :

```bash
FOSA_SANS_VOIX=1 ./deployer_robot.sh --envoyer 192.168.1.3
```

Sinon `--envoyer` télécharge automatiquement Vosk et son modèle s'ils manquent.

## Simuler sur le PC

Avec ROS 2 Jazzy et les paquets officiels PARC dans le workspace :

```bash
./simulation/simulation.sh preparer
./simulation/simulation.sh essai   # préparation puis match
```

Sans fenêtre Gazebo : préfixer la commande par `FOSA_SIM_GUI=0`.

La carte des organisateurs est utilisée par défaut. Pour la forcer :

```bash
FOSA_CARTE="$(ros2 pkg prefix caytu_config)/share/caytu_config/config/cartes/arene_2026_11_06" ./run.sh course
```

## Autres commandes

```bash
./run.sh preparation              # cartographie autonome et tags
./run.sh course-sans-carte        # match sans carte
./run.sh ordre-test '4,9,1,7'     # test seulement, sans arbitre
```
