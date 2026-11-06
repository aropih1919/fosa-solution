# Détection d'un AprilTag par une caméra.
# Repère du tag (convention OpenCV) : x vers la droite, y vers le haut,
# z sortant du tag vers la caméra (normale de la surface).
std_msgs/Header header
int32 tag_id
geometry_msgs/PoseStamped pose_camera   # repère optique de la caméra
geometry_msgs/PoseStamped pose_robot    # repère de base du robot
bool has_pose_robot
geometry_msgs/PoseStamped pose_map      # repère global (SLAM)
bool has_pose_map
float32 distance_m
float32 confidence
