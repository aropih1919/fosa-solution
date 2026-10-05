# Benchmark de navigation (tools/)

Mesure objectivement un run de navigation, pour savoir si un changement
(caméra, tuning de vitesse, réglage d'AMCL) améliore ou dégrade le score.

## Fichiers

| Fichier | Rôle |
|---|---|
| `tools/benchmark.py` | Noeud ROS 2 qui écoute un run et écrit un JSON. Ne pilote pas le robot. |
| `tools/summarize.py` | Moyenne de tous les runs par label, écrit `results/summary.csv`. Sans ROS. |
| `tools/bench_core.py` | Calculs (distance au but, épisodes de contact, erreurs de pose, statistiques). Sans ROS. |
| `tools/test_bench_core.py` | Tests de la logique de calcul : `python3 tools/test_bench_core.py` |
| `tools/results/` | Un `run_<date>_<label>.json` par run + `summary.csv`. |

## Lancer un run

Le benchmark doit démarrer en même temps que la solution (le temps compte
depuis son lancement). Relancer Gazebo ET la solution entre deux runs.

```bash
# Terminal 1 : Gazebo      ros2 launch parc_robot_bringup task.launch.py
# Terminal 2 : navigation  (voir stepCompleted.md)
# Terminal 3 : le benchmark, puis tout de suite la solution
source ~/PNSolution/ros2_ws/install/setup.bash
cd <chemin>/fosa-solution
python3 tools/benchmark.py --label baseline_sans_camera
# Terminal 4 :
ros2 run caytu_nav_solution task_solution --ros-args -p use_sim_time:=true
```

Le benchmark s'arrête tout seul quand :

| Résultat | Signification |
|---|---|
| `SUCCESS` | une partie du robot touche le cercle vert |
| `TIMEOUT` | 600 s simulées écoulées |
| `NAV_ENDED_EARLY` | Nav2 a fini son goal (ABORTED, CANCELED...) sans arriver |
| `CHECKPOINT` | arrêt volontaire demandé avec `--stop-at` |
| `SIM_FROZEN` | l'horloge de Gazebo ne bouge plus depuis 60 s réelles |
| `NO_POSE`, `NO_CLOCK` | pas de position ou pas de `/clock` |
| `INTERRUPTED` | Ctrl+C (les mesures sont quand même enregistrées) |

Convention de labels : `baseline_sans_camera`, `camera_v1`, `vitesse_0.6`, `amcl_cov_serree`...
Faire environ 5 runs par label, puis `python3 tools/summarize.py`.

## Runs courts (machine lente)

Le simulateur tourne à environ 0,12 fois le temps réel : un run de 600 s simulées
dure plus d'une heure. Pour comparer des réglages, utiliser des runs courts :

```bash
python3 tools/benchmark.py --label vitesse_0.6 --stop-at 120
```

Le run s'arrête à 120 s simulées (`CHECKPOINT`). Comparer alors `progress_m`
(distance gagnée vers le but), `average_speed_mps`, la covariance et les contacts.
Réserver les runs complets (600 s) aux validations finales.

## Mesurer la vitesse de la machine

Toutes les 10 s, le benchmark affiche `RTF=...` (temps simulé / temps réel).
Test sans lancer la solution :

```bash
python3 tools/benchmark.py --label rtf_test --stop-at 30
```

Pour tester l'effet de la configuration DDS (`CYCLONEDDS_URI`), faire dans TOUS
les terminaux, puis relancer Gazebo, et comparer le RTF affiché :

```bash
unset CYCLONEDDS_URI
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## Ce qui est mesuré

| Clé du JSON | Signification |
|---|---|
| `time_sec` | Temps SIMULÉ depuis le lancement du benchmark (critère #3) |
| `final_center_distance` | Distance centre du robot -> centre du but à la fin (critère #2) |
| `contacts_episodes` | Nombre d'épisodes de contact hors sol (critère #1) |
| `progress_m`, `average_speed_mps`, `path_length_m` | Avancement et vitesse |
| `recoveries` | Nombre de recoveries Nav2 (spin, backup, wait, clear costmap) |
| `nav2.last_status`, `status_history` | État du goal Nav2 : SUCCEEDED / ABORTED / CANCELED |
| `localization.loss_count` | Nombre de fois où `/localization_ready` est passé à false |
| `amcl_cov` / `amcl_cov_max_xy` | Covariance d'AMCL : départ, maximum, dernière valeur |
| `localization_error` / `loc_error_max_xy` | Écart entre la position d'AMCL et la VRAIE pose Gazebo |
| `at_first_loss` | Covariance et erreur réelle au moment de la première perte de localisation |
| `amcl_series` | Courbe `[t, cov_xx, cov_yy, cov_yaw, erreur_m]` pour tracer l'évolution |
| `sim_freeze.max_wall_seconds` | Plus longue période sans avancée de l'horloge simulée |
| `real_time_factor` | Temps simulé / temps réel sur le run |
| `t_first_goal`, `t_first_motion` | Durée du démarrage avant le premier goal / mouvement |

## Comment lire les diagnostics de localisation

- `localization_error.first_xy` grand (> 0,5 m) dès le départ : le repère `map` est décalé
  par rapport au monde Gazebo, ou AMCL est mal initialisé. Le benchmark affiche un avertissement.
- `at_first_loss.loc_error_xy` petit alors que la perte est déclarée : la localisation
  est en réalité correcte et seuls les seuils du watchdog sont trop stricts.
- `at_first_loss.loc_error_xy` grand : AMCL dérive vraiment (carte, paramètres AMCL).

## Source de la position

Par défaut (`--pose-source auto`), le benchmark juge l'arrivée avec la vraie pose Gazebo
(topic `/sitoe_robot/pose`), comme l'évaluation officielle. S'il est absent, il se replie sur
AMCL (TF `map -> base_footprint`). Options : `--pose-source truth|tf`, `--truth-topic <topic>`.

## Limites connues

- Le rayon du cercle est lu dans `parc_robot_bringup/models/goal_location/model.sdf`
  (0,6 m sur votre machine). Sinon la valeur par défaut est 0,5 m, signalée dans
  `goal.radius_source` ; donner alors `--goal-radius`.
- Les contacts avec le sol et avec le robot lui-même sont ignorés. `--debug-contacts`
  affiche les paires de collisions vues, pour régler `--ignore-keywords`.
- Le temps officiel commence au lancement de la solution, pas à celui du benchmark :
  les lancer ensemble. Rien n'indique dans la documentation si ce temps est simulé ou réel :
  à demander aux organisateurs (le JSON garde `wall_seconds` et `real_time_factor`).
