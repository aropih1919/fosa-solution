johny-olivier@johny-olivier-VivoBook:~$ cd ros2_ws/src/fosa-solution/
johny-olivier@johny-olivier-VivoBook:~/ros2_ws/src/fosa-solution$ code .
johny-olivier@johny-olivier-VivoBook:~/ros2_ws/src/fosa-solution$ cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --envoyer 192.168.1.126
[1/3] Archive des seules sources : /home/johny-olivier/ros2_ws/fosa.zip
[2/3] Copie vers sitoe@192.168.1.126:~/fosa.zip
sitoe@192.168.1.126's password: 
fosa.zip                                                                                                                                                    100%  353MB   3.2MB/s   01:51    
[3/3] Extraction et compilation dans ~/teams/fosa sur le robot...
sitoe@192.168.1.126's password: 
Starting >>> caytu_config
Starting >>> caytu_interfaces
Starting >>> parc_msgs
Starting >>> caytu_nav_bringup
Starting >>> parc_robot_description
Starting >>> parc_robot
Finished <<< caytu_nav_bringup [2.11s]
Finished <<< parc_robot [2.14s]
Finished <<< parc_robot_description [2.28s]
Starting >>> parc_robot_bringup
Finished <<< caytu_config [3.00s]
Starting >>> caytu_nav_solution
Finished <<< parc_robot_bringup [2.59s]
Finished <<< caytu_nav_solution [2.99s]
Finished <<< caytu_interfaces [12.9s]
Starting >>> caytu_perception
Finished <<< parc_msgs [15.7s]
Starting >>> parc_referee
Finished <<< caytu_perception [2.83s]
Starting >>> caytu_course
Finished <<< parc_referee [3.18s]
Finished <<< caytu_course [3.24s]

Summary: 11 packages finished [19.3s]
Sources compilées dans /home/sitoe/teams/fosa.

Déploiement terminé. Archive conservée sur le PC : /home/johny-olivier/ros2_ws/fosa.zip
Étape suivante, sur le PC :
  ssh sitoe@192.168.1.126
  cd ~/teams/fosa/src/fosa-solution
  ./run.sh diagnostic-robot
johny-olivier@johny-olivier-VivoBook:~/ros2_ws/src/fosa-solution$ cd ~/ros2_ws/src/fosa-solution
./deployer_robot.sh --preparer
./deployer_robot.sh --envoyer 192.168.1.126
[1/3] Vosk (roue aarch64, sans dépendances)...
Collecting vosk
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/vosk-0.3.45-py3-none-manylinux2014_aarch64.whl
Successfully downloaded vosk
[2/3] Dépendances de Vosk (roues aarch64 ; srt n'existe qu'en source)...
Collecting cffi
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/cffi-2.1.1-cp312-cp312-manylinux2014_aarch64.manylinux_2_17_aarch64.whl
Collecting requests
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/requests-2.34.2-py3-none-any.whl
Collecting tqdm
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/tqdm-4.70.1-py3-none-any.whl
Collecting websockets
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/websockets-17.4-cp312-cp312-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl
Collecting pycparser (from cffi)
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/pycparser-3.12-py3-none-any.whl
Collecting charset_normalizer<4,>=2 (from requests)
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/charset_normalizer-3.5.2-cp312-cp312-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl
Collecting idna<4,>=2.5 (from requests)
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/idna-3.20-py3-none-any.whl
Collecting urllib3<3,>=1.26 (from requests)
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/urllib3-2.8.0-py3-none-any.whl
Collecting certifi>=2023.5.7 (from requests)
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/certifi-2026.10.23-py3-none-any.whl
Successfully downloaded cffi requests tqdm websockets certifi charset_normalizer idna urllib3 pycparser
Collecting srt
  File was already downloaded /home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config/config/livraison/roues/srt-3.5.3.tar.gz
  Preparing metadata (setup.py) ... done
Successfully downloaded srt
[3/3] Modèle Vosk vosk-model-small-fr-0.22...
Puis : ./deployer_robot.sh --envoyer ROBOT_IP
[1/3] Archive des seules sources : /home/johny-olivier/ros2_ws/fosa.zip
[2/3] Copie vers sitoe@192.168.1.126:~/fosa.zip
sitoe@192.168.1.126's password: 
fosa.zip                                                                                                                                                    100%  353MB   2.5MB/s   02:20    
[3/3] Extraction et compilation dans ~/teams/fosa sur le robot...
sitoe@192.168.1.126's password: 
Starting >>> caytu_config
Starting >>> caytu_interfaces
Starting >>> parc_msgs
Starting >>> caytu_nav_bringup
Starting >>> parc_robot_description
Starting >>> parc_robot
Finished <<< parc_robot_description [0.66s]
Starting >>> parc_robot_bringup
Finished <<< caytu_nav_bringup [0.75s]
Finished <<< parc_robot [0.80s]
Finished <<< caytu_interfaces [1.69s]
Finished <<< parc_msgs [1.74s]
Starting >>> parc_referee
Finished <<< parc_robot_bringup [1.63s]
Finished <<< caytu_config [3.50s]
Starting >>> caytu_nav_solution
Starting >>> caytu_perception
Finished <<< parc_referee [3.41s]
Finished <<< caytu_nav_solution [3.38s]
Finished <<< caytu_perception [3.40s]
Starting >>> caytu_course
Finished <<< caytu_course [2.95s]

Summary: 11 packages finished [10.2s]
Sources compilées dans /home/sitoe/teams/fosa.

Déploiement terminé. Archive conservée sur le PC : /home/johny-olivier/ros2_ws/fosa.zip
Étape suivante, sur le PC :
  ssh sitoe@192.168.1.126
  cd ~/teams/fosa/src/fosa-solution
  ./run.sh diagnostic-robot
johny-olivier@johny-olivier-VivoBook:~/ros2_ws/src/fosa-solution$ ssh sitoe@192.168.1.126
cd ~/teams/fosa/src/fosa-solution
./deployer_robot.sh
sitoe@192.168.1.126's password: 
Welcome to Ubuntu 24.04.5 LTS (GNU/Linux 6.8.12-1021-tegra aarch64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/pro

This system has been minimized by removing packages and content that are
not required on a system that users do not log into.

To restore this content, you can run the 'unminimize' command.

Expanded Security Maintenance for Applications is not enabled.

35 updates can be applied immediately.
30 of these updates are standard security updates.
To see these additional updates run: apt list --upgradable

115 additional security updates can be applied with ESM Apps.
Learn more about enabling ESM Apps service at https://ubuntu.com/esm

Last login: Thu Nov  5 18:41:23 2026 from 192.168.1.145
sitoe@ubuntu:~$ cd ~/teams/fosa/src/fosa-solution
sitoe@ubuntu:~/teams/fosa/src/fosa-solution$ ./run.sh diagnostic-robot

== Environnement
  [INFO] ROS_DISTRO=jazzy ROS_DOMAIN_ID=0 RMW=rmw_cyclonedds_cpp
  [INFO] OpenCV : 4.6.0
  [OK]   Vosk importable
  [OK]   paquet nav2_bt_navigator
  [OK]   paquet nav2_controller
  [OK]   paquet nav2_smac_planner
  [OK]   paquet nav2_regulated_pure_pursuit_controller
  [OK]   paquet slam_toolbox
  [OK]   paquet caytu_course
  [OK]   modèle Vosk : /home/sitoe/teams/fosa/src/fosa-solution/caytu_config/config/models/vosk-model-small-fr-0.22

== Topics attendus par la configuration
  [OK]   /scan_filtered [sensor_msgs/msg/LaserScan]
  [OK]   /botwheel_explorer/odom [nav_msgs/msg/Odometry]
  [OK]   /cmd_vel [geometry_msgs/msg/Twist]
  [ÉCHEC] /speech_audio absent (audio du mot de départ)
  [OK]   /parc/tag_order [parc_msgs/msg/TagOrder]
  [OK]   /camera1/camera1/color/image_raw [sensor_msgs/msg/Image]
  [OK]   /camera1/camera1/color/camera_info [sensor_msgs/msg/CameraInfo]
  [OK]   /camera2/camera2/color/image_raw [sensor_msgs/msg/Image]
  [OK]   /camera2/camera2/color/camera_info [sensor_msgs/msg/CameraInfo]
  [OK]   /scan_filtered [sensor_msgs/msg/LaserScan]
  [ÉCHEC] /fosa/scan_camera1 absent (source d'obstacles (activée ou non dans fosa.yaml))
  [ÉCHEC] /fosa/scan_camera2 absent (source d'obstacles (activée ou non dans fosa.yaml))

== Arbitre (parc_msgs)
  [OK]   parc_msgs installé
  [INFO] parc_msgs/msg/Finish
      builtin_interfaces/Time stamp
      uint8 reason
      constante REASON_COMPLETED = 0
      constante REASON_GAVE_UP = 1
  [INFO] parc_msgs/msg/RunStatus
      builtin_interfaces/Time stamp
      uint8 state
      float elapsed_s
      float time_limit_s
      sequence<uint8> tag_order
      sequence<uint8> tags_reported
      uint8 start_method
      boolean false_start
      constante ARMED = 1
      constante FINISHED = 3
      constante IDLE = 0
      constante RUNNING = 2
      constante TERMINATED = 5
      constante TIMED_OUT = 4
  [INFO] parc_msgs/msg/StartSignal
      builtin_interfaces/Time stamp
      uint8 method
      string heard
      constante METHOD_COMMAND = 0
      constante METHOD_VOICE = 1
  [INFO] parc_msgs/msg/TagOrder
      builtin_interfaces/Time stamp
      sequence<uint8> tag_ids
  [INFO] parc_msgs/msg/TagVisit
      std_msgs/Header header
      uint8 tag_id
      sensor_msgs/CompressedImage image
  [INFO] /parc/tag_order
      Type: parc_msgs/msg/TagOrder
      Node name: parc_referee
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: rosbag2_recorder
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: parc_apriltag_detector
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: parc_competition_controller
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
  [INFO] /parc/start
      Type: parc_msgs/msg/StartSignal
      Node name: competition_mission_node
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_competition_controller
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_referee
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: rosbag2_recorder
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
  [INFO] /parc/tag_visit
      Type: parc_msgs/msg/TagVisit
      Node name: competition_mission_node
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_competition_controller
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_referee
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: rosbag2_recorder
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_tag_navigation_controller
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
  [INFO] /parc/finish
      Type: parc_msgs/msg/Finish
      Node name: competition_mission_node
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_competition_controller
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: parc_referee
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
      Node name: rosbag2_recorder
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: VOLATILE
  [INFO] /parc/run_status
      Type: parc_msgs/msg/RunStatus
      Node name: parc_referee
      Node namespace: /
      Endpoint type: PUBLISHER
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: rosbag2_recorder
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: parc_apriltag_detector
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: parc_tag_navigation_controller
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL
      Node name: parc_competition_controller
      Node namespace: /
      Endpoint type: SUBSCRIPTION
        Reliability: RELIABLE
        Durability: TRANSIENT_LOCAL

== Type de la commande de vitesse
  Type: geometry_msgs/msg/Twist
  Publisher count: 3
  Subscription count: 2
  [INFO] Nav2 publie un Twist non horodaté (enable_stamped_cmd_vel: false).

== Mesures (caméras, scans, repères, audio)

== Caméras (camera_info)
  camera1: 640x360, fx=458.9, modèle plumb_bob, repère camera1_color_optical_frame
     dans base_link : x=0.140 y=0.015 z=0.530
     visée dans base_link : (+0.38, +0.00, -0.92), pente -67°  [À VÉRIFIER] l'axe x de base_link n'est pas la direction de visée
  [ÉCHEC] camera2 : aucune calibration reçue

== Scans
  lidar: 720 rayons (388 valides), [-3.12, 3.14] rad, portée 0.15-16.00 m, repère laser
     dans base_link : x=0.000 y=0.000 z=-0.060 tangage=0.0° lacet=180.0°

== Repères
  base_footprint relié à base_link : non
  odom relié à base_link : oui
  map relié à base_link : non

== Audio
  /audio : aucun message (pour /speech_audio, parler pendant la mesure)
  /speech_audio : aucun message (pour /speech_audio, parler pendant la mesure)

== Un message de chaque capteur
  [OK]   /botwheel_explorer/odom publie
  [OK]   /scan_filtered publie

== Rapport enregistré : /home/sitoe/.ros/fosa/diagnostic_20261105_184716.txt
sitoe@ubuntu:~/teams/fosa/src/fosa-solution$ 

