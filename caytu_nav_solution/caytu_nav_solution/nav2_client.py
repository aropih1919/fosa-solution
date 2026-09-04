from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class Nav2Client:
    """Encapsule l'envoi d'un goal NavigateToPose à Nav2."""

    def __init__(self, node):
        self._node = node
        self._action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose'
        )

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """Attend que le serveur d'action Nav2 soit disponible."""
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, pose_stamped):
        """Envoie le goal. TODO J2: feedback callback, gestion résultat,
        timeout 600s, retour SUCCESS/FAILURE/TIMEOUT."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        # TODO J2: send_goal_async + callbacks feedback/result
        raise NotImplementedError('Logique d\'envoi à implémenter en J2')