# Validation Jazzy et simulation isolées

Docker est une option de CI, pas le chemin de test principal : sur le PC
Ubuntu 24.04 avec ROS 2 Jazzy natif, utiliser directement `./outils/verifier.sh`
après `./simulation/simulation.sh preparer`.

Depuis la racine :

```bash
./outils/verifier.sh --docker
./outils/verifier.sh --docker --simulation
```

L’image `fosa-finale:jazzy` contient Jazzy, Nav2, SLAM, Gazebo et les dépendances
officielles. Prévoir plusieurs Go libres et Internet pour sa construction.
L’exécution utilise `--network none`, notamment pour les tests qui saturent
volontairement les ports DDS. Aucun topic moteur ne sort vers le robot.

Les paquets officiels `parc_msgs`, `parc_referee`, `parc_robot_bringup` et
`parc_robot_description` sont lus dans le `src` voisin, puis copiés et compilés
avec les six paquets Fosa. Le dépôt hôte est en lecture seule.

```bash
FOSA_IMAGE_PRETE=1 \
FOSA_SIM_WORKSPACE=/chemin/vers/nouvelle-validation \
FOSA_PARC_SRC=/chemin/vers/sources-parc \
./outils/verifier.sh --docker --simulation
```

- `FOSA_IMAGE_PRETE=1` réutilise l’image ; `FOSA_IMAGE_TESTS` change son nom.
- `FOSA_SIM_WORKSPACE` choisit la sortie ; son dossier `src` doit être absent.
- Par défaut : `results/validation-<date>/`.

Résultats : `resultats/tests.xml` (JUnit), `log/` (colcon), et
`.maison/.fosa_simulation/journaux/…/course/rapport_course.json`.
Le script ajoute le pont `/model/sitoe_robot/pose -> /sim/pose_verite` attendu
par le moniteur ; sans lui, le pont officiel laisse `/sim/pose` vide.
Le rendu logiciel est sans fenêtre et sa vitesse dépend du PC. Le budget
reste 600 s réelles. Parcours incomplet, visite invalide ou délai dépassé :
retour non nul.
