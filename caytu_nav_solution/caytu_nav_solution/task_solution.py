#!/usr/bin/env python3
"""
J2 : attente localization_ready (fallback test 5s) -> goal -> Nav2 -> résultat.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from caytu_nav_solution.goal_locator import GoalLocator
from caytu_nav_solution.nav2_client import Nav2Client, NavResult

LOCALIZATION_TEST_FALLBACK_SEC = 5.0


class TaskSolution(Node):

    def __init__(self):
        super().__init__('task_solution')

        self._localization_ready = False
        self._navigation_started = False

        # Un goal réel ne doit jamais partir après le simple délai de développement.
        # Les tests locaux peuvent réactiver ce comportement avec -p test_mode:=true.
        self.declare_parameter('test_mode', False)
        self.declare_parameter('cancel_on_localization_loss', True)

        self.create_subscription(Bool, '/localization_ready', self._on_localization_ready, 10)

        self.goal_locator = GoalLocator(self)
        self.nav2_client = Nav2Client(self)

        self._fallback_timer = self.create_timer(
            LOCALIZATION_TEST_FALLBACK_SEC, self._on_fallback_timeout
        )

        self.get_logger().info(
            'task_solution: attente de /localization_ready '
            f'(fallback test après {LOCALIZATION_TEST_FALLBACK_SEC:.0f}s si test_mode=True)'
        )

    def _on_localization_ready(self, msg: Bool):
        was_ready = self._localization_ready
        self._localization_ready = msg.data

        if msg.data and not was_ready:
            self.get_logger().info('Signal localization_ready reçu (réel).')
            self._start_navigation()
        elif not msg.data and was_ready and self._navigation_started:
            # Une covariance redevenue mauvaise peut rendre le plan en map dangereux.
            # La politique est paramétrable pour garder les essais de tuning possibles.
            if self.get_parameter('cancel_on_localization_loss').value:
                self.get_logger().error(
                    'Localisation perdue pendant la navigation — annulation du goal.'
                )
                self.nav2_client.cancel_active_goal()
            else:
                self.get_logger().warn(
                    'Localisation perdue pendant la navigation (annulation désactivée).'
                )

    def _on_fallback_timeout(self):
        self._fallback_timer.cancel()
        if self._localization_ready:
            return

        if self.get_parameter('test_mode').value:
            self.get_logger().warn('Aucun localization_ready reçu — démarrage MODE TEST.')
            self._start_navigation()
        else:
            self.get_logger().error('localization_ready absent, test_mode=False — bloqué.')

    def _start_navigation(self):
        if self._navigation_started:
            return
        self._navigation_started = True

        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 indisponible — arrêt.')
            self._shutdown()
            return

        goal = self.goal_locator.get_goal()
        self.nav2_client.send_goal(goal, self._on_navigation_result)

    def _on_navigation_result(self, result: NavResult):
        self.get_logger().info(f'task_solution: résultat = {result.value}')
        self._shutdown()

    def _shutdown(self):
        self.get_logger().info('task_solution: arrêt propre.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TaskSolution()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
