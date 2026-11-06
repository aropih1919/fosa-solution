# Exigences techniques PARC 2026 — Phase 2 (état au 02/11)

**Sources :** discussions avec les organisateurs (29/09 → 02/11) et pages officielles de la documentation.
**À lire avec prudence :** je n'ai pas vu le fichier `ros2_topics.md`, le PDF des tags, ni les dimensions du robot. Rien n'est dit sur OpenCV : aucune bibliothèque de vision n'est imposée.

---

## 1. Ce qui est demandé

| Sujet | Statut | Ce que disent les organisateurs |
|---|---|---|
| **ROS 2** | Confirmé | Base de la ligue. Exécution à distance, avec accès SSH au robot. |
| **Nav2** | Recommandé, attendu en phase 2 | 29/09 : « recommandation », car Nav2 facilite le passage au robot physique. 24/10 : « en plus de naviguer avec Nav2 », vous irez vers des lieux marqués. |
| **AprilTags** | Annoncé pour la phase 2 | Famille **36h11**, IDs **1, 4, 7, 9** (pas d'ID 0), **16,5 cm**, posés à **1 m** du sol. Positions **à découvrir**. |
| **Interaction vocale avec un LLM** | Possible (« you might be asked ») | Micro sur le robot. Après l'échange, le robot se déplace vers un ou plusieurs points. |
| **Calcul** | Confirmé | Traitement sur vos PC (les données viennent des capteurs du robot) ou sur le **Jetson AGX Orin 32 Go** embarqué. |
| **Chrono** | Confirmé | **Temps réel** (chronomètre), pas le temps simulé. Plus court = récompensé. |
| **Environnement** | Annoncé | Boîte de **6 m × 6 m**, hauteur **0,4 m**. De nouveaux obstacles pourraient apparaître. |
| **Carte** | Confirmé (18/10) | Vous construisez votre carte sur place et ajustez vos paramètres Nav2 pendant les jours précédant la finale. |
| **Règlement et plan de l'arène** | Pas encore publiés | « Plus d'informations dans les prochains jours » (01/11). |

Pour la vision, l'organisateur demande de chercher comment lire les tags 36h11 et les intégrer à ROS et Nav2. D'après mes connaissances, le module aruco d'OpenCV et les paquets ROS 2 `apriltag` lisent cette famille : vérifiez avec vos versions.

---

## 2. Mise à jour du 02/11

| Sujet | Réponse officielle | Impact sur votre solution |
|---|---|---|
| **IMU** | Les topics IMU existent maintenant (capteur sur la caméra RealSense). | Votre localisation et votre filtre de sol en dépendent. Vérifier le nom du topic et le repère : l'IMU étant sur la caméra, ses axes ne sont pas ceux du châssis. |
| **Repères TF** | L'arbre montre `odom → base_link`, sans `base_footprint`. Evans vérifiera demain si `base_footprint` existe et à quelle hauteur est `base_link`. | Presque toute votre config utilise `base_footprint` : Nav2, footprint, filtre de sol, localisation. |
| **Commandes** | Partir sur `/cmd_vel` « pour le moment ». | Votre launch envoie sur `/robot_base_controller/cmd_vel_unstamped`. Le type de message (Twist simple ou horodaté) n'est pas précisé. |
| **Audio** | `/speech_audio` en `audio_common_msgs/AudioData` (octets bruts). Aucun son vers le haut-parleur du robot. | Pas de voix du robot : la réponse passe par du texte ou par un déplacement. |
| **Paquets** | Nav2 n'est **pas** préinstallé. Seuls les paquets des capteurs le sont. | Tout ce dont vous avez besoin doit venir de vous. |
| **Exécution** | À distance, avec accès SSH. Jetson utilisable. | Exécuter sur vos PC évite d'installer Nav2 sur la Jetson. |

D'après ce que je connais du paquet, `AudioData` ne contient que les octets, sans fréquence d'échantillonnage ni nombre de canaux. Cherchez ces informations dans le fichier des topics, ou demandez-les.

---

## 3. Ce que cela change pour votre solution

1. **L'IMU est maintenant publiée**, mais sur la caméra : repère et orientation à vérifier avant de s'en servir pour le cap et l'inclinaison.
2. **Votre localisation lit un départ connu dans `task_params.yaml`.** Sur le robot réel, ce fichier n'existe pas. Vous cartographiez sur place : prévoir `slam_toolbox`, ou AMCL sur la carte construite.
3. **Les noms de topics et de capteurs ne sont pas confirmés identiques à la simulation.** Comparez-les au fichier `ros2_topics.md`.
4. **La détection des tags est un point de risque.** Estimation : avec une caméra de 640 px de large et 90° de champ (celle de la simulation), un tag de 16,5 cm mesure environ 18 px à 3 m, ce qui est limite. Dans une boîte de 6 m, le robot devra s'approcher. La résolution de la vraie caméra reste à mesurer.
5. **Les murs de 0,4 m** sont plus bas que votre filtre de caméra haute (`min_height` 0,60 m) : seul le lidar les verrait. À vérifier sur place.

---

## 4. À préparer sans le robot

1. **Rendre configurables** le repère de base, le topic de commande et le topic IMU (arguments de launch). Passer `use_sim_time` à **`false`** sur le robot réel, qui n'a pas d'horloge Gazebo.
2. **Prévoir un pont de repères.** Si `base_link` est bien au sol, une TF statique identité `base_link → base_footprint` suffit. Sinon, un décalage en hauteur. À décider après la réponse d'Evans.
3. **Revoir les réglages tirés du simulateur** : inclinaison de 2,5°, secteurs de masquage du lidar, seuils de hauteur des caméras.
4. **Localisation sans départ connu** : `slam_toolbox` ou AMCL.
5. **Tout embarquer dans votre dépôt** : Nav2 et les dépendances ne seront pas fournis.

---

## 5. À vérifier demain sur le robot

```bash
printenv ROS_DISTRO                 # version de ROS du robot
ros2 topic list
ros2 topic info /cmd_vel -v         # type du message de commande
ros2 topic echo --once /imu         # repère et unités de l'IMU
ros2 run tf2_tools view_frames      # base_footprint ? hauteur de base_link ?
```

Vérifier aussi que votre PC voit les topics du robot sur le réseau (même domaine ROS) et que les deux machines utilisent des versions de ROS compatibles. Je n'ai aucune information sur ce point.

---

## 6. À demander à Evans

- Le fichier exact des topics et la position de l'IMU
- Le modèle de caméra et sa résolution
- Le type de message de `/cmd_vel` (Twist ou TwistStamped)
- La fréquence d'échantillonnage et le nombre de canaux de `/speech_audio`
- La date de publication du règlement et le plan de l'arène
- Si l'interaction doit obligatoirement être vocale (question du 01/11 sans réponse directe)

---

## 7. Corrections de mes réponses précédentes

- **Le script `parc_resultats.py` mesure le temps simulé** (`/clock`), alors que le chrono officiel est le **temps réel**. Si votre simulation tourne plus lentement que le temps réel, ses chiffres seront trop bons. Il peut être modifié pour afficher les deux temps.
- **La voix n'est plus un bonus** côté robot : pas de haut-parleur utilisable. Restez sur voix en entrée (micro), réponse en texte, puis déplacement.
