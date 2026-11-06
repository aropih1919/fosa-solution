# Fosa — PARC 2026

## Déployer ou mettre à jour depuis le PC

```bash
cd ~/Solution/ros2_ws/src/fosa-solution
./deployer_robot.sh --envoyer ROBOT_IP
```

## Compiler sur le robot

```bash
ssh sitoe@ROBOT_IP
source /opt/ros/jazzy/setup.bash
source ~/parc_ws/install/setup.bash   # sinon ~/ros2_ws/install/setup.bash
cd ~/teams/fosa/actuel
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select caytu_config caytu_interfaces \
  caytu_nav_bringup caytu_nav_solution caytu_perception caytu_course \
  --executor sequential
source install/setup.bash
```

## Lancer le match

```bash
cd ~/teams/fosa/actuel/src/fosa-solution
./run.sh course
./run.sh depart          # ou dire « allez »
```

## Suivre et arrêter

```bash
./run.sh suivi
./run.sh arreter
```

## Carte

La carte livrée est `caytu_config/config/cartes/arene_2026_11_06`.
Placer le robot à la position de départ de l'enregistrement avant
`./run.sh course`.

```bash
FOSA_CARTE=/chemin/vers/carte ./run.sh course   # autre carte
./run.sh course-sans-carte                     # sans carte
./run.sh preparation                           # refaire la carte
./run.sh course
```
