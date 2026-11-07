# Fosa — PARC 2026

Depuis la racine du dépôt. ROS 2 Jazzy et les paquets officiels PARC sont
supposés installés.

## PC → robot

```bash
./deployer_robot.sh --envoyer 192.168.1.3
```

Sans voix :

```bash
FOSA_SANS_VOIX=1 ./deployer_robot.sh --envoyer 192.168.1.3
```

## Robot

```bash
./run.sh installer
./run.sh diagnostic-robot
./run.sh preparation
./run.sh course
./run.sh depart         # ou dire « allez »
./run.sh suivi          # autre terminal
./run.sh arreter
```

`Ctrl+C` arrête la session ; fermer le terminal la laisse tourner.

## Cartographie manuelle

Terminal 1 :

```bash
./run.sh preparation-manuelle
```

Terminal 2 :

```bash
./run.sh teleop
```

Puis :

```bash
./run.sh sauver-carte
./run.sh arreter
```

## Simulation

```bash
./simulation/simulation.sh preparer
./simulation/simulation.sh essai
./simulation/simulation.sh course '4,9,1,7' 42
```

Sans fenêtre :

```bash
FOSA_SIM_GUI=0 ./simulation/simulation.sh essai
```

## Aide

```bash
./run.sh help
./run.sh etat
./run.sh ordre-test '4,9,1,7'
./run.sh diagnostic-dds
./run.sh mesure-rotation
```

Carte forcée :

```bash
FOSA_CARTE=/chemin/carte ./run.sh course
```
