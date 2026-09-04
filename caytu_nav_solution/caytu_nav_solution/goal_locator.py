"""
Fournit le goal (pose cible) à envoyer à Nav2.
STATUT (J2) : source réelle du but toujours pas identifiée (nécessite
inspection de task.launch.py / ros2 topic-param-list sur The Construct).
En attendant : mode test piloté par paramètres ROS2 (use_test_goal).
"""

from geometry_msgs.msg import PoseStamped


class GoalLocator:
    """Fournit le goal à atteindre sous forme de PoseStamped."""

    def __init__(self, node):
        self._node = node

        self._node.declare_parameter('use_test_goal', True)
        self._node.declare_parameter('goal_x', 2.0)
        self._node.declare_parameter('goal_y', 1.0)
        self._node.declare_parameter('goal_yaw', 0.0)

    def get_goal(self) -> PoseStamped:
        """Retourne le goal courant sous forme de PoseStamped."""
        if self._node.get_parameter('use_test_goal').value:
            return self._get_test_goal()

        # TODO : brancher ici la vraie source une fois identifiée sur Construct
        raise NotImplementedError(
            'Source réelle du goal non identifiée — utiliser use_test_goal=True.'
        )

    def _get_test_goal(self) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()

        goal.pose.position.x = self._node.get_parameter('goal_x').value
        goal.pose.position.y = self._node.get_parameter('goal_y').value
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # yaw=0 simplifié

        self._node.get_logger().warn('GoalLocator: goal de TEST utilisé (use_test_goal=True).')
        return goal