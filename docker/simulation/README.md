# Préparer le workspace de simulation ROS 2 dans Docker

Depuis la racine du dépôt :

```bash
bash docker/simulation/preparer.sh
```

Le script construit `fosa-simulation:jazzy`, copie les paquets Fosa et les
paquets PARC présents dans `../../src`, installe leurs dépendances avec
`rosdep`, puis compile un workspace persistant dans
`results/docker/workspace-simulation`. Les dépôts sources sont montés en
lecture seule ; seul ce nouveau workspace reçoit des écritures. L’option
`FOSA_SIM_WORKSPACE=/chemin/vide` permet de choisir un autre emplacement.

Cette étape prépare et compile les paquets ; elle ne lance pas encore Gazebo.
Le premier build nécessite Internet et peut télécharger plusieurs Go. Une
erreur de compilation ou de dépendance arrête le script avec un code non nul.
