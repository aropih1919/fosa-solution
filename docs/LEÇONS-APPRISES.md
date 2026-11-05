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

## [2026-11-05] — Garder une recherche de tags praticable dans une carte encombrée
- Timing : runtime
- Symptôme : le robot répète des tours sur place, sans trajet Nav2, jusqu'à expiration de la préparation.
- Cause racine : un échantillonnage unique et une marge identique autour des obstacles et de l'inconnu peuvent éliminer tous les points dans une arène dense.
- Détecteur manquant : aucun signalement borné quand deux balayages consécutifs ne produisent aucun point de navigation.
- Garde-fou : échantillonner plus finement près des frontières de carte, laisser Nav2 valider l'empreinte complète, et arrêter avec un diagnostic après deux balayages sans passage.

## [2026-11-05] — Ne pas envoyer le robot dans un passage plus étroit que son empreinte
- Timing : runtime
- Symptôme : Nav2 planifie un point de recherche, puis le contrôleur répète « collision ahead » et le robot reste immobile.
- Cause racine : la marge de secours de 0,35 m était inférieure au rayon d'encombrement du robot (environ 0,43 m avec padding).
- Détecteur manquant : la configuration acceptait une marge de secours inférieure à la marge normale.
- Garde-fou : garder 0,50 m autour des obstacles, relâcher uniquement la marge à la frontière inconnue et rejeter une marge de secours plus faible que la marge normale.
