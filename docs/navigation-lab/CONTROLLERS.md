# Contrôleurs

Le sélecteur central est `controller=rpp|dwb|mppi`. Tous utilisent le même planificateur SmacPlanner2D, la même BT, les mêmes costmaps et le même topic de commande du robot. Le plugin chargé et son YAML sont contrôlés avant le lancement.

| Contrôleur | État | Configuration de départ |
|---|---|---|
| RPP | Référence de la soumission | `controller_rpp.yaml`, mêmes valeurs en `balanced`. |
| DWB | Déjà disponible dans la soumission | `controller_dwb.yaml`, mêmes valeurs en `balanced`. |
| MPPI | Ajouté pour Jazzy | Modèle `DiffDrive`, horizon 40 × 0,05 s, batch 1000, une itération, visualisation désactivée. Valeurs initiales à mesurer. |

Nav2 Jazzy documente MPPI pour les bases différentielles. La configuration du dépôt reprend les noms de paramètres de l'exemple installé dans `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`; le batch et l'horizon sont réduits pour ménager le CPU. La machine auditée a 8 cœurs logiques et 7,5 GiB de RAM, mais ces chiffres ne garantissent pas une fréquence de contrôle en simulation. Le rapport enregistre la fréquence observée de `/local_plan`.

Les limites communes restent celles déjà documentées pour le plugin de base en simulation : 1,5 m/s et 1,0 rad/s. Les profils actuels restent bien en dessous. Cela ne valide aucune vitesse pour le robot physique.

MPPI est déclaré comme dépendance ROS. S'il n'est pas installé, le laboratoire refuse le choix avant le lancement. L'installation observée ici inclut `nav2_mppi_controller`; aucune mission Gazebo complète n'a été exécutée dans cet environnement.

Références : [plugin MPPI Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/), [guide officiel des plugins](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/).
