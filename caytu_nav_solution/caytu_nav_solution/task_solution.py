#!/usr/bin/env python3
"""
J3 : attente de DEUX signaux "prêt" (localisation ET caméra) -> goal -> Nav2 -> résultat.

- /localization_ready        (Bool) : publié par localization_watchdog.py (Trinôme 1 sprint précédent)
- /camera_perception_ready   (Bool) : publié par camera_perception_watchdog.py (Trinôme OpenCV)
  -> nom du topic réglable via le paramètre `camera_ready_topic`.

Sécurité : si la caméra ne devient jamais "prête", on ne bloque pas toute la tâche
(limite 600 s) : après `camera_wait_timeout_sec`, on démarre en mode dégradé (LiDAR seul).
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
        self._camera_ready = False
        self._navigation_started = False
        self._camera_timer = None

        # Un goal réel ne doit jamais partir après le simple délai de développement.
        # Les tests locaux peuvent réactiver ce comportement avec -p test_mode:=true.
        self.declare_parameter('test_mode', False)
        self.declare_parameter('cancel_on_localization_loss', True)
        # --- nouveaux paramètres (extension caméra) ---
        self.declare_parameter('require_camera', True)
        self.declare_parameter('camera_ready_topic', '/camera_perception_ready')
        self.declare_parameter('camera_wait_timeout_sec', 30.0)   # <= 0 : attendre sans limite
        self.declare_parameter('cancel_on_camera_loss', False)    # LiDAR reste utilisable

        self.create_subscription(Bool, '/localization_ready', self._on_localization_ready, 10)
        camera_topic = self.get_parameter('camera_ready_topic').value
        self.create_subscription(Bool, camera_topic, self._on_camera_ready, 10)

        self.goal_locator = GoalLocator(self)
        self.nav2_client = Nav2Client(self)

        self._fallback_timer = self.create_timer(
            LOCALIZATION_TEST_FALLBACK_SEC, self._on_fallback_timeout
        )

        self.get_logger().info(
            f'task_solution: attente de /localization_ready ET {camera_topic} '
            f'(require_camera={self.get_parameter("require_camera").value}, '
            f'fallback test après {LOCALIZATION_TEST_FALLBACK_SEC:.0f}s si test_mode=True)'
        )

    # ------------------------------------------------------------ signaux ---
    def _on_localization_ready(self, msg: Bool):
        was_ready = self._localization_ready
        self._localization_ready = msg.data

        if msg.data and not was_ready:
            self.get_logger().info('Signal localization_ready reçu (réel).')
            self._try_start_navigation()
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

    def _on_camera_ready(self, msg: Bool):
        was_ready = self._camera_ready
        self._camera_ready = msg.data

        if msg.data and not was_ready:
            self.get_logger().info('Signal camera_perception_ready reçu.')
            self._try_start_navigation()
        elif not msg.data and was_ready and self._navigation_started:
            if self.get_parameter('cancel_on_camera_loss').value:
                self.get_logger().error('Caméra perdue pendant la navigation — annulation du goal.')
                self.nav2_client.cancel_active_goal()
            else:
                self.get_logger().warn(
                    'Caméra perdue pendant la navigation — on continue (LiDAR seul).'
                )

    # ----------------------------------------------------------- démarrage ---
    def _camera_ok(self) -> bool:
        return self._camera_ready or not self.get_parameter('require_camera').value

    def _try_start_navigation(self):
        """Démarre seulement si TOUTES les conditions requises sont réunies."""
        if self._navigation_started or not self._localization_ready:
            return

        if self._camera_ok():
            self._stop_camera_timer()
            self._start_navigation()
        else:
            self._arm_camera_timer()

    def _arm_camera_timer(self):
        if self._camera_timer is not None:
            return
        timeout = self.get_parameter('camera_wait_timeout_sec').value
        if timeout > 0:
            self.get_logger().info(
                f'Localisation OK, attente de la caméra (max {timeout:.0f}s avant mode dégradé).'
            )
            self._camera_timer = self.create_timer(timeout, self._on_camera_timeout)
        else:
            self.get_logger().info('Localisation OK, attente de la caméra (sans limite).')

    def _stop_camera_timer(self):
        if self._camera_timer is not None:
            self._camera_timer.cancel()
            self._camera_timer = None

    def _on_camera_timeout(self):
        self._stop_camera_timer()
        if self._navigation_started or not self._localization_ready or self._camera_ok():
            return
        self.get_logger().warn(
            'Caméra toujours pas prête — démarrage en MODE DÉGRADÉ (LiDAR seul).'
        )
        self._start_navigation()

    def _on_fallback_timeout(self):
        self._fallback_timer.cancel()
        if self._navigation_started or self._localization_ready:
            return  # cas localisation OK : géré par la logique caméra ci-dessus

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