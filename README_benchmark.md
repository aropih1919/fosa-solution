# Benchmark de navigation (tools/)

Mesure objectivement un run de navigation, pour savoir si un changement
(caméra, tuning de vitesse) améliore ou dégrade le score.

## Fichiers

| Fichier | Rôle |
|---|---|
| `tools/benchmark.py` | Noeud ROS 2 qui écoute un run et écrit un JSON. Ne pilote pas le robot. |
| `tools/summarize.py` | Fait la moyenne de tous les runs par label, écrit `results/summary.csv`. Sans ROS. |
| `tools/bench_core.py` | Calculs (distance au but, épisodes de contact, statistiques). Sans ROS. |
| `tools/test_bench_core.py` | Tests de la logique de calcul. |
| `tools/results/` | Un `run_<date>_<label>.json` par run + `summary.csv`. |

## Lancer un run

Le benchmark doit démarrer en même temps que la solution (le temps compte
depuis son lancement). Il faut relancer Gazebo ET la solution entre deux runs.

```bash
# Terminal 1 : Gazebo  (ros2 launch parc_robot_bringup task.launch.py use_sim_time:=true)
# Terminal 2 : navigation FOSA  (voir stepCompleted.md)
# Terminal 3 : le benchmark, puis tout de suite la solution
source /opt/ros/jazzy/setup.bash && source ~/PNSolution/ros2_ws/install/setup.bash
cd <chemin>/fosa-solution
python3 tools/benchmark.py --label baseline_sans_camera
# Terminal 4 :
ros2 run caytu_nav_solution task_solution --ros-args -p use_sim_time:=true
```

Le benchmark s'arrête tout seul quand : le robot touche le cercle (`SUCCESS`),
600 s simulées sont écoulées (`TIMEOUT`), Nav2 a fini son goal sans arriver
(`NAV_ENDED_EARLY`), ou l'horloge de Gazebo ne bouge plus (`SIM_FROZEN`).
`Ctrl+C` enregistre quand même ce qui a été mesuré (`INTERRUPTED`).

Convention de labels : `baseline_sans_camera`, `camera_v1`, `vitesse_0.6`...
Faire environ 5 runs par label, puis `python3 tools/summarize.py`.

## Ce qui est mesuré

| Clé du JSON | Signification |
|---|---|
| `time_sec` | Temps SIMULÉ depuis le lancement du benchmark (critère #3) |
| `final_center_distance` | Distance centre du robot -> centre du but à la fin (critère #2) |
| `contacts_episodes` | Nombre d'épisodes de contact hors sol (critère #1) |
| `recoveries` | Nombre de recoveries Nav2 (spin, backup, wait, clear costmap) |
| `nav2.last_status`, `status_history` | État du goal Nav2 : SUCCEEDED / ABORTED / CANCELED |
| `localization.loss_count` | Nombre de fois où `/localization_ready` est passé à false |
| `sim_freeze.max_wall_seconds` | Plus longue période sans avancée de l'horloge simulée |
| `real_time_factor` | Temps simulé / temps réel (vitesse de la machine) |
| `t_first_goal`, `t_first_motion` | Durée du démarrage avant le premier goal / mouvement |

## Limites connues

- La position vient par défaut de la TF `map -> base_footprint` (AMCL, précision ~20 cm).
  Si la pose réelle de Gazebo est bridgée vers ROS, utiliser `--pose-source topic
  --pose-topic <topic> --pose-type posestamped`.
- Le rayon du cercle est lu dans `parc_robot_bringup/models/goal_location/model.sdf`.
  Si le fichier est introuvable, la valeur par défaut est 0.5 m : le JSON l'indique
  (`goal.radius_source`). Donner alors le vrai rayon avec `--goal-radius`.
- Les contacts ne sont mesurés que si Gazebo publie des topics `*_collisions` côté ROS.
  Sinon `contacts_available` vaut false. `--debug-contacts` affiche les paires de
  collisions vues, pour régler `--ignore-keywords`.
- Le critère officiel de temps commence au lancement de la solution, pas à celui du
  benchmark : les lancer ensemble.
