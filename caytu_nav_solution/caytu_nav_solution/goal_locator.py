from geometry_msgs.msg import PoseStamped


class GoalLocator:
    """Fournit le goal à atteindre sous forme de PoseStamped."""

    def __init__(self, node):
        self._node = node

        # Placeholder de test — à remplacer une fois la vraie source identifiée
        self._node.declare_parameter('goal_x', 2.0)
        self._node.declare_parameter('goal_y', 1.0)
        self._node.declare_parameter('goal_yaw', 0.0)

    def get_goal(self) -> PoseStamped:
        """Retourne le goal courant sous forme de PoseStamped.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()

        goal.pose.position.x = self._node.get_parameter('goal_x').value
        goal.pose.position.y = self._node.get_parameter('goal_y').value
        goal.pose.position.z = 0.0

        # Orientation simplifiée (yaw=0 par défaut, quaternion identité)
        goal.pose.orientation.w = 1.0

        return goal