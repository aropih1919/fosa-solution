# Johny — TUNING NOTES — PARC 2026

## Baseline Configuration (config A) — validée dans branches précédentes

- Footprint polygone 0.54×0.44 m, padding 0.02, frames base_footprint/odom/map
- LiDAR /scan (lidar_link), obstacle 0.15–10 m, raytrace 0.15–12 m, marking/clearing true
- Global : 0.05 m res, 5×5? non (full map), update 5 Hz, inflation 0.35, cost_scaling 3.0
- Local : rolling 5×5 m, 0.05 m res, 10 Hz, inflation 0.30
- Planner SmacPlanner2D : tolerance 0.3, max_time 2.0 s, lissage activé
- Controller DWB : max_vel_x 0.40 (80% Gazebo 0.5), vtheta 1.0, accel 1.0/2.0, sim_time 1.7, 20 samples
- Critics BaseObstacle 0.7, PathAlign/GoalAlign/GoalDist 24–32, PreferForward 3.0
- Goal tolerance 0.15 m / 0.25 rad, progress 0.3 m / 10 s

## Méthodologie de tuning (§29)

Groupe 1 : footprint/géométrie → validé URDF, hypothèse à mesurer au mètre
Groupe 2 : obstacle detection → vérifier en RViz que /scan alimente bien les costmaps
Groupe 3 : clearing → raytrace doit effacer obstacles mobiles <0.5 s après départ
Groupe 4 : inflation → ajuster par pas 0.05 m
Groupe 5 : planner → comparer Smac vs NavFn sur temps/ longueur
Groupe 6 : controller → tester 0.35 vs 0.40 m/s
Groupe 7 : vitesse/accélération → monter jusqu'à limite Gazebo avec marge sécurité

## Proposition Config B (tuning pour passages étroits + vitesse)

Si baseline jugée trop conservative en passages café (tables espacées ~1.0–1.2 m) :
- Global inflation_radius : 0.35 → **0.30** (gain ~10 cm de dégagement)
- Local inflation_radius : 0.30 → **0.27**
- cost_scaling_factor : 3.0 → **4.0** (pente plus raide = moins de trajectoires longues)
- DWB max_vel_x : 0.40 → **0.45** (si collisions = 0 en baseline, gagner 12% temps)
- DWB BaseObstacle scale : 0.7 → **0.5** (si robot freine trop tôt, autoriser approche plus proche)

Mesures attendues (à consigner par run 600 s) :
| Config | Goal atteint | Temps | Collisions | Dist finale | Path len | Min dist obs | Replan | Oscillations |
|--------|--------------|-------|------------|-------------|----------|--------------|--------|--------------|
| A      | oui/non      | s     | N          | m           | m        | m            | N      | oui/non      |
| B      | oui/non      | s     | N          | m           | m        | m            | N      | oui/non      |

## Proposition Config C (sécurité maximale, foules denses)

Si collisions >0 en baseline :
- inflation_radius global 0.35 → **0.40**, local 0.30 → **0.35**
- max_vel_x 0.40 → **0.30**, acc_lim_x 1.0 → **0.6**
- DWB sim_time 1.7 → **2.2** (anticipation plus longue)
- Ajouter static_layer en local (décommenter) pour mieux gérer murs

## Tests recommandés (§23/27)

Sans simulation exhaustive ici (Nav2 non installé, Gazebo Harmonic requis), les tests
doivent être rejoués sur machine avec ros2 launch parc_robot_bringup task.launch.py :

Test A ligne droite : spawn (-0.20,-7.48) → goal (-2.29,2.23) sans obstacles mobiles
Test B virage : goal intermédiaire latéral
Test C obstacle fixe entre robot et goal (cafe_table)
Test D passage étroit (2 tables distantes 1.0 m)
Test E replanning : placer obstacle mobile sur chemin après 5 s, vérifier nouveau path
Test F foule : 3 FemaleVisitor traversant, vérifier clearing

Critères diagnostic §31 :
- Cas 1 path trop près mur → vérifier inflation/footprint avant controller
- Cas 2 path correct mais oscillation → baisser max_vel_theta, vérifier odom
- Cas 3 LiDAR voit mais costmap non → topic/frame/TF/observation_sources
- Cas 4 obstacle parti reste → clearing/raytrace/observation_persistence

## Intégration Nav2 pour Faneva

Faneva doit fusionner dans nav2_params.yaml les blocs :
- global_costmap/global_costmap (ce fichier)
- local_costmap/local_costmap
- planner_server (GridBased SmacPlanner2D)
- controller_server (FollowPath DWB)

Exemple complet voir `config/nav2_params.example.yaml` (à venir, non commité ici pour éviter conflit).
Contrat Johny→Faneva :
  planner_plugin_id: GridBased → nav2_smac_planner/SmacPlanner2D
  controller_plugin_id: FollowPath → dwb_core/DWBLocalPlanner
  cmd_vel_topic: /cmd_vel (bridge vers robot_base_controller/cmd_vel_unstamped si besoin)
  frames: map/odom/base_footprint/lidar_link
  topic scan: /scan (LaserScan, lidar_link)
  map: stadium_map.yaml (0.05, origin [-7.118,-7.909,0]) fourni Trinôme1
