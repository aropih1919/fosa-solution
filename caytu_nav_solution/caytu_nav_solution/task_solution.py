import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from caytu_nav_solution.goal_locator import GoalLocator
from caytu_nav_solution.nav2_client import Nav2Client


class TaskSolution(Node):

    def __init__(self):
        super().__init__('task_solution')

        self._localization_ready = False

        # TODO J1/J2: confirmer nom exact du topic avec Trinôme 1
        self.create_subscription(
            Bool, '/localization_ready', self._on_localization_ready, 10
        )

        self.goal_locator = GoalLocator(self)
        self.nav2_client = Nav2Client(self)

        self.get_logger().info('task_solution: node initialisé (squelette J1)')

    def _on_localization_ready(self, msg: Bool):
        self._localization_ready = msg.data
        if self._localization_ready:
            self.get_logger().info('Signal localization_ready reçu.')


def main(args=None):
    rclpy.init(args=args)
    node = TaskSolution()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()