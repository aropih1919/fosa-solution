# Métriques et rapports

Chaque exécution crée `results/FOSA-<année>-NNN/` avec `summary.json`, `metadata.json`, `config_used.yaml`, `metrics.csv`, `simulation.log` et `task_solution.log`. `results/` est ignoré par Git. Le rosbag n'est pas enregistré automatiquement.

Pendant l'attente des capteurs puis pendant la navigation, le monitoring réactualise une seule ligne du terminal chaque seconde : pose estimée, distance au but, vitesses mesurées par l'odométrie et commandées, état du goal Nav2, chemin global `/plan`, plan local s'il est publié, disponibilité de la localisation/TF, portée minimale du scan et épisodes de contact observés.

## Provenance

- Réussite : code de sortie `task_solution.py` (0 = succès).
- Temps d'exécution : horloge ROS si `use_sim_time` et `/clock` fonctionnel; sinon horloge monotone. Le temps réel est également conservé. L'initialisation de localisation est mesurée depuis le début de mission jusqu'au premier message de pose localisée. Le temps de navigation effective commence au premier goal `navigate_to_pose` en exécution.
- Distance parcourue prioritaire : intégrale des positions successives de `/sitoe_robot/pose` (vérité terrain Gazebo). Si ce topic manque, le rapport utilise `/odom` et indique sa source. La distance de la pose estimée est enregistrée séparément.
- Distance finale : vérité terrain si elle existe, sinon la pose de localisation; `final_goal_distance_source` indique laquelle.
- Localisation : ODOM_IMU/ODOM_ONLY via `/localization_pose`, AMCL via `/amcl_pose`. En simulation, les erreurs position/angle sont calculées seulement quand les horodatages reçus sont à moins de 0,25 s de la vérité terrain.
- Collisions : épisodes par paire de contacts sur `/base_collisions`, `/top_chassis_collisions`, `/left_wheel_collisions`, `/right_wheel_collisions`. Les contacts au sol sont ignorés; les messages répétitifs d'une paire sur moins de 0,5 s forment un épisode. Sans message, la métrique est `null`/`N/A`, jamais zéro.
- Obstacle minimum : plus petite portée finie reçue dans `/scan_clean` ou les scans caméra; ce n'est pas une distance certifiée entre empreintes du robot.
- Fréquence : fréquence observée des publications `/local_plan`, proxy explicite du cycle du contrôleur.
- CPU et RAM : mesures système via `psutil`, pas attribution à un seul processus. Absence de `psutil` donne `N/A`.
- Récupérations : goals exécutés sur les actions `spin`, `backup`, `wait`; retries : identifiants de goals `navigate_to_pose` moins le premier. Sans messages de statut, `N/A`.
- Oscillations : heuristique déclarée (au moins quatre inversions de vitesse angulaire en 3 s et déplacement net inférieur à 0,15 m); le champ est marqué comme estimé.

Tous les champs absents restent `null`. Les unités sont dans les noms de métriques; les sources se trouvent dans les champs `*_source` ou `metadata.json`. Aucune note pondérée n'est calculée.
