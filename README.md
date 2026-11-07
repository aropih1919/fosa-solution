# Fosa — PARC 2026

## 1. Depuis le PC : déployer

```bash
./deployer_robot.sh --envoyer 192.168.1.3                    # remplace l'installation du robot
FOSA_SANS_VOIX=1 ./deployer_robot.sh --envoyer 192.168.1.3   # idem, sans voix
```

## 2. Sur le robot : dans l'ordre

```bash
ssh sitoe@192.168.1.3                     # 1. se connecter au robot
cd ~/teams/fosa/actuel/src/fosa-solution  # 2. dossier du code sur le robot
./run.sh installer                        # 3. dépendances + compilation (une fois)
./run.sh diagnostic-robot                 # 4. premier créneau : vérifier le robot
./run.sh preparation                      # 5. 15 min avant : carte + 4 tags
./run.sh course                           # 6. lance le match
./run.sh depart                           # 7. donne le départ (ou dire « allez »)
./run.sh arreter                          # 8. coupe la session
```

`Ctrl+C` arrête la session ; fermer le terminal la laisse tourner en
arrière-plan.

## 3. Pendant la course : autre terminal

```bash
ssh sitoe@192.168.1.3                     # se connecter au robot
cd ~/teams/fosa/actuel/src/fosa-solution  # dossier du code sur le robot
./run.sh suivi                            # affiche ce que fait le robot
```

## 4. Cartographie manuelle (remplace l'étape 5)

Terminal 1 :

```bash
ssh sitoe@192.168.1.3                     # se connecter au robot
cd ~/teams/fosa/actuel/src/fosa-solution  # dossier du code sur le robot
./run.sh preparation-manuelle             # guidage manuel
```

Terminal 2 :

```bash
ssh sitoe@192.168.1.3                     # se connecter au robot
cd ~/teams/fosa/actuel/src/fosa-solution  # dossier du code sur le robot
./run.sh teleop                           # pilote le robot au clavier
```

Retour au terminal 1, après le guidage :

```bash
./run.sh sauver-carte                     # enregistre la carte et les tags
./run.sh arreter                          # coupe la session
```

## 5. Simulation PC : dans l'ordre

```bash
cd ~/ros2_ws/src/fosa-solution                    # dossier du code sur le PC
./simulation/simulation.sh preparer               # 1. compile une fois
./simulation/simulation.sh essai                  # 2. préparation + match (graine 7)
./simulation/simulation.sh course '4,9,1,7' 7     # 3. rejoue un match sur la graine 7
./simulation/simulation.sh arreter                # 4. si une session reste bloquée
FOSA_SIM_GUI=0 ./simulation/simulation.sh essai   # sans fenêtre
```

## 6. Aide

```bash
./run.sh help                             # tous les modes
./run.sh etat                             # état brut de la course
./run.sh ordre-test '4,9,1,7'             # essai : publie un ordre
./run.sh diagnostic-dds                   # imports et nœud ROS
./run.sh mesure-rotation                  # odométrie contre gyromètre
FOSA_CARTE=/chemin/carte ./run.sh course  # force une carte
```
