# Campagnes d'expériences

Le seul scénario disponible dans le dépôt est `full_course`, basé sur le lancement officiel `parc_robot_bringup/task.launch.py` et son `task_params.yaml`. Les autres options de menu sont explicitement indisponibles : le dépôt ne livre pas de mondes ou fixtures reproductibles de ligne droite, virages, passage étroit, obstacles statiques ou mobiles.

Campagnes initiales, à répéter cinq fois par combinaison :

1. **Contrôleurs** : `rpp`, `dwb`, `mppi` + `odom_imu` + `balanced` + `full_course`.
2. **Localisation** : `rpp` + `odom_imu`, `odom_only`, `amcl` + `balanced` + `full_course`. N'exécuter AMCL qu'avec sa carte validée. `amcl_ekf` ne peut pas encore être lancé.
3. **Profils de vitesse et de prudence** : `rpp`, `dwb`, `mppi` chacun avec `ultra_safe`, `safe`, `balanced`, `fast`, `ultra_fast`, en gardant localisation et scénario identiques.
4. **Marge de collision** : répéter chaque contrôleur avec `tight_clearance` à vitesse limitée (0,25 m/s), séparément des comparaisons de vitesse. Ce test reste réservé à Gazebo.

Comparer les collisions connues d'abord, puis réussite, temps et distance. Un résultat sans donnée de collision reste inconnu; il ne bat pas automatiquement un essai sûr. Ne pas agréger les répétitions en un score unique; comparer également médiane, dispersion et échecs. Les scénarios supplémentaires doivent être ajoutés seulement avec une fixture Gazebo versionnée, des conditions de départ définies et un moyen de remettre le monde dans le même état.
