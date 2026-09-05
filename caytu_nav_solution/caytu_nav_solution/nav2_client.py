"""
Client de l'action NavigateToPose : envoi du goal, suivi du feedback,
gestion SUCCESS / FAILURE / TIMEOUT (limite officielle : 600 s).
"""

from enum import Enum

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class NavResult(Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    TIMEOUT = 'TIMEOUT'


class Nav2Client:
    """Encapsule l'envoi d'un goal NavigateToPose à Nav2 et son suivi."""

    ACTION_NAME = 'navigate_to_pose'
    TIMEOUT_SEC = 600.0

    def __init__(self, node, timeout_sec: float = TIMEOUT_SEC):
        self._node = node
        self._timeout_sec = timeout_sec
        self._action_client = ActionClient(node, NavigateToPose, self.ACTION_NAME)

        self._goal_handle = None
        self._result_callback = None
        self._timeout_timer = None
        self._finished = False

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """Attend que le serveur d'action Nav2 soit disponible."""
        self._node.get_logger().info('Nav2Client: attente du serveur navigate_to_pose...')
        available = self._action_client.wait_for_server(timeout_sec=timeout_sec)
        if available:
            self._node.get_logger().info('Nav2Client: serveur disponible.')
        else:
            self._node.get_logger().error('Nav2Client: serveur indisponible après timeout.')
        return available

    def send_goal(self, pose_stamped, result_callback):
        """Envoie le goal de façon asynchrone.

        result_callback(NavResult) est appelé exactement une fois
        (SUCCESS, FAILURE ou TIMEOUT).
        """
        self._result_callback = result_callback
        self._finished = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self._node.get_logger().info('Nav2Client: envoi du goal...')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback
        )
        send_goal_future.add_done_callback(self._on_goal_response)

        # Garde-fou des 600 s dès l'envoi du goal
        self._timeout_timer = self._node.create_timer(
            self._timeout_sec, self._on_timeout
        )

    def cancel_active_goal(self):
        """Annule le goal en cours sans conclure avant le résultat de l'action.

        Le callback résultat conserve ainsi un point de sortie unique pour le node.
        """
        if self._finished:
            return
        if self._goal_handle is None:
            self._node.get_logger().warn(
                'Nav2Client: annulation demandée avant acceptation du goal.'
            )
            return
        self._node.get_logger().warn('Nav2Client: annulation du goal actif.')
        self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error('Nav2Client: goal rejeté par Nav2.')
            self._finish(NavResult.FAILURE)
            return

        self._node.get_logger().info('Nav2Client: goal accepté.')
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        distance = getattr(feedback_msg.feedback, 'distance_remaining', None)
        if distance is not None:
            self._node.get_logger().debug(f'Nav2Client: distance restante = {distance:.2f} m')

    def _on_result(self, future):
        if self._finished:
            return  # déjà tranché par un timeout

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish(NavResult.SUCCESS)
        else:
            self._node.get_logger().warn(f'Nav2Client: échec, status={status}')
            self._finish(NavResult.FAILURE)

    def _on_timeout(self):
        if self._finished:
            return

        self._node.get_logger().error(
            f'Nav2Client: TIMEOUT après {self._timeout_sec:.0f}s — annulation du goal.'
        )
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        self._finish(NavResult.TIMEOUT)

    def _finish(self, result: NavResult):
        if self._finished:
            return
        self._finished = True

        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        if self._result_callback is not None:
            self._result_callback(result)
