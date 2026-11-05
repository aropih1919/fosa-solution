# Tests ROS 2 Jazzy dans Docker

Depuis la racine du dépôt, sur un poste Linux avec Docker démarré :

```bash
bash outils/tester_docker.sh
```

La première exécution télécharge ROS 2 et les dépendances déclarées dans les
`package.xml` (Nav2, OpenCV, etc.) : prévoir plusieurs Go et une connexion réseau.
Les suivantes réutilisent les couches Docker tant que les dépendances ne changent
pas. ROS/Gazebo n'ont pas besoin d'être installés sur le poste.

Les manifestes déclarent actuellement `ament_python` comme dépendance de build,
mais `rosdep` ne résout pas cette clé. L'image exclut uniquement cette clé :
le support des paquets Python est fourni par `python3-colcon-common-extensions`.

## Ce qui est exécuté

1. Construction de l'image ROS 2 Jazzy et installation par `rosdep`.
2. Copie des six paquets du dépôt, y compris les modifications non commitées,
   dans un workspace temporaire du conteneur.
3. Compilation `colcon build` des paquets et des messages de simulation
   `parc_msgs` (copiés explicitement malgré `simulation/COLCON_IGNORE`).
4. Tests Pytest de configuration, course, perception, client Nav2 et BT.
5. Vérification des imports ROS, ressources installées, profils robot/simulation,
   lecture des arguments du launch et transport d'un message `caytu_interfaces`
   par DDS.

Le dépôt est monté en lecture seule. Les tests utilisent `--network none`, sans
périphérique ni réseau hôte : ils ne peuvent pas joindre le robot. Seuls les
rapports sont écrits sur le poste, avec ton UID/GID. Aucun nœud de course n'est
lancé. Le conteneur et son workspace disparaissent après l'exécution.

## Résultats

Le script affiche un dossier `results/docker/essai-…/` contenant :

- `image.log` : construction de l'image ;
- `tests.log` : compilation et tests ;
- `pytest.xml` : résultats JUnit, y compris les tests ignorés ;
- `colcon/` : journaux détaillés de compilation ;
- `code_sortie.txt` : `0` si la compilation et les tests ont réussi ;
- `commit.txt`, `etat-git.txt`, `image-id.txt`, `versions-ros.txt` : contexte de l'essai.

Un échec d'image, de compilation ou de test donne un code de sortie non nul au
script. Si la compilation échoue, il n'y a pas de rapport Pytest. Les rapports
sont ignorés par Git. Conserver le rapport avec le commit testé dans la PR ;
commiter les modifications après vérification pour que l'équipe puisse les reproduire.

## Portée de la validation

Un résultat vert prouve que ce build et ces tests passent sous Jazzy. Il ne
valide pas encore l'exécution du BT par Nav2, l'évitement en mouvement, les
capteurs, la concurrence des exécuteurs, les délais d'arrêt ou le vrai protocole
arbitre. La copie `parc_msgs` est uniquement celle de simulation. Les points de
revue identifiés sur l'alignement et la gestion TF restent à traiter.

L'image suit l'architecture du poste : sur PC x86_64, ce n'est pas une validation
des binaires ARM64 du Jetson. `ros:jazzy-ros-base` et les dépôts apt évoluent ;
les versions et l'identifiant d'image sont enregistrés, sans promettre un build
identique à une date future.

Avant déploiement : compléter par des tests ROS/Nav2 avec capteurs simulés ou
rosbags, puis par des essais encadrés sur le robot et le vrai `parc_msgs`.
Partager cette branche via une PR permet de faire ces vérifications avant fusion.
