# Utilisation

Ces commandes supposent le workspace PARC installé et le monde officiel disponible.

Le dépôt fournit aussi `cmd.sh` pour le lancement à deux terminaux : dans le premier, `./cmd.sh gazebo` démarre le monde officiel avec la sortie de Gazebo masquée; laissez le processus ouvert. Dans le second, `./cmd.sh benchmark` démarre le sélecteur en réutilisant ce monde. Si les exécutables du laboratoire ne sont pas encore installés, le script construit d'abord `caytu_nav_bringup` et `caytu_nav_solution` dans le workspace. Pour arrêter Gazebo, faites `Ctrl+C` dans le premier terminal.

Pour choisir directement une configuration dans le deuxième terminal :

```bash
./cmd.sh benchmark -p controller:=rpp -p localization:=odom_imu \
  -p profile:=balanced -p scenario:=full_course
```

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to caytu_nav_solution caytu_nav_bringup
source install/setup.bash
cd ~/ros2_ws/src/fosa-solution
ros2 run caytu_nav_solution fosa_benchmark
```

Le menu sélectionne contrôleur, localisation, profil et scénario. Il lance ensuite `parc_robot_bringup task.launch.py`, attend `/odom` et le scan brut `/scan`, valide la configuration, puis lance `task_solution.py`; c'est alors que le bringup publie `/scan_clean`. À la fin, le monde est arrêté et son rapport écrit. Lancer le monde séparément est possible avec `-p start_simulation:=false`.

Lancement non interactif, mêmes commandes ROS 2 que le menu :

```bash
ros2 run caytu_nav_solution fosa_benchmark --ros-args \
  -p controller:=rpp \
  -p localization:=odom_imu \
  -p profile:=balanced \
  -p scenario:=full_course
```

AMCL :

```bash
ros2 run caytu_nav_solution fosa_benchmark --ros-args \
  -p controller:=rpp -p localization:=amcl \
  -p profile:=balanced -p scenario:=full_course
```

La mission AMCL s'arrête avant le lancement si la carte café n'est pas lisible, si le scan initial ne la confirme pas ou si le spawn/but n'est pas dans sa zone libre.

Le menu affiche `tight_clearance` comme « Marge minimale ». Ce profil réduit le champ d'inflation pour les essais de proximité en simulation. Il conserve l'empreinte du robot et les contrôles de collision, mais peut conduire à un contact; ne pas l'utiliser sur le robot physique.

Pendant le lancement, le terminal du benchmark réactualise une ligne de monitoring chaque seconde. Il distingue le chemin global `/plan` du plan local, et montre l'état du goal, la pose et la distance au but, les vitesses de commande/odométrie, la localisation, la TF, le scan et les contacts détectés.

Comparer tous les essais ou filtrer :

```bash
ros2 run caytu_nav_solution fosa_compare --results-dir results
ros2 run caytu_nav_solution fosa_compare --results-dir results \
  --controller mppi --localization odom_imu --profile balanced
```

L'entrée officielle reste disponible indépendamment du laboratoire : démarrer d'abord le monde PARC, puis `ros2 run caytu_nav_solution task_solution.py`. Son défaut reste RPP + `odom_imu` + `balanced`.

`amcl_ekf` et l'environnement physique sont bloqués volontairement jusqu'à vérification de l'URDF/producteur TF, des covariances et des limites matérielles. `fast` n'est pas un profil validé sur le robot réel.
