# Leçons apprises

## [2026-11-05] — Attendre la découverte d'un topic ROS avant `echo --once`
- Timing : runtime
- Symptôme : le contrôle de préparation arrêtait Gazebo avec « robot simulé n'a pas démarré ».
- Cause racine : `ros2 topic echo --once` quitte immédiatement avec une erreur si le topic n'est pas encore découvert par le graphe ROS.
- Détecteur manquant : aucun contrôle de présence du topic avant l'appel à `echo --once`.
- Garde-fou : sonder `ros2 topic list` jusqu'à l'apparition du topic, puis vérifier la réception d'un message avant de déclarer le lancement prêt.

## [2026-11-05] — S'abonner aux cartes SLAM avec le QoS transient-local
- Timing : runtime
- Symptôme : la préparation attend une carte, sans planifier de trajet ni publier de commande moteur.
- Cause racine : l'abonnement de `course_parc` en `VOLATILE` manque la carte déjà publiée par `slam_toolbox` en `TRANSIENT_LOCAL`.
- Détecteur manquant : aucun contrôle que la grille reçue par `course_parc` contient la carte publiée.
- Garde-fou : conserver `TRANSIENT_LOCAL` pour l'abonnement à `/map`, et attendre les serveurs Nav2 requis avant la première action de recherche.
