# Fosa — PARC 2026

Commandes dans l'ordre. ROS 2 Jazzy et les paquets officiels PARC sont
supposés installés.

## 1. Depuis le PC : déployer

```bash
./deployer_robot.sh --envoyer 192.168.1.3                    # envoie et compile sur le robot
FOSA_SANS_VOIX=1 ./deployer_robot.sh --envoyer 192.168.1.3   # sans voix
```

## 2. Sur le robot : dans l'ordre

```bash
./run.sh installer         # 1. dépendances + compilation (une fois)
./run.sh diagnostic-robot  # 2. premier créneau : vérifier le robot
./run.sh preparation       # 3. 15 min avant : carte + 4 tags
./run.sh course            # 4. lance le match
./run.sh depart            # 5. donne le départ (ou dire « allez »)
./run.sh suivi             # pendant la course, autre terminal
./run.sh arreter           # 6. coupe la session
```

`Ctrl+C` arrête la session ; fermer le terminal la laisse tourner en
arrière-plan.

## 3. Cartographie manuelle (remplace l'étape 3)

Terminal 1 :

```bash
./run.sh preparation-manuelle   # 3 bis. guidage manuel
```

Terminal 2 :

```bash
./run.sh teleop                 # pilote le robot au clavier
```

Retour au terminal 1, après le guidage :

```bash
./run.sh sauver-carte           # enregistre la carte et les tags
./run.sh arreter                # coupe la session
```

## 4. Simulation PC : dans l'ordre

```bash
./simulation/simulation.sh preparer              # 1. compile une fois
./simulation/simulation.sh essai                 # 2. préparation + match
./simulation/simulation.sh course '4,9,1,7' 42   # 3. rejoue un match
./simulation/simulation.sh arreter               # 4. si une session reste bloquée
FOSA_SIM_GUI=0 ./simulation/simulation.sh essai  # sans fenêtre
```

## 5. Aide

```bash
./run.sh help                            # tous les modes
./run.sh etat                            # état brut de la course
./run.sh ordre-test '4,9,1,7'            # essai : publie un ordre
./run.sh diagnostic-dds                  # imports et nœud ROS
./run.sh mesure-rotation                 # odométrie contre gyromètre
FOSA_CARTE=/chemin/carte ./run.sh course # force une carte
```
