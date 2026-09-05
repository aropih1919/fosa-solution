"""
Fournit le goal (pose cible) à envoyer à Nav2.

STATUT (màj après investigation) :
- La source réelle du but est le fichier task_params.yaml du package
  parc_robot_bringup (paramètres fixes : goal_x, goal_y, goal_z).
- /goal_pose n'est PAS la bonne source : ce topic est publié par RViz2
  (bouton "2D Goal Pose", usage manuel humain), pas par la tâche elle-même.
  bt_navigator y est abonné mais rien ne le publie automatiquement pour
  la compétition.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped


class GoalLocator:
    """Fournit le goal à atteindre sous forme de PoseStamped, lu depuis task_params.yaml."""

    def __init__(self, node):
        self._node = node

        self._node.declare_parameter('use_test_goal', False)
        self._node.declare_parameter('goal_x', 2.0)
        self._node.declare_parameter('goal_y', 1.0)
        self._node.declare_parameter('goal_yaw', 0.0)

    def get_goal(self) -> PoseStamped:
        """Retourne le goal à atteindre sous forme de PoseStamped."""
        if self._node.get_parameter('use_test_goal').value:
            return self._get_test_goal()
        return self._get_goal_from_task_params()

    def _get_goal_from_task_params(self) -> PoseStamped:
        """Lit goal_x / goal_y depuis config/task_params.yaml de parc_robot_bringup."""
        bringup_dir = get_package_share_directory('parc_robot_bringup')
        params_path = os.path.join(bringup_dir, 'config', 'task_params.yaml')

        try:
            with open(params_path, 'r') as f:
                data = yaml.safe_load(f)
            params = data['/**']['ros__parameters']
            goal_x = float(params['goal_x'])
            goal_y = float(params['goal_y'])
        except (FileNotFoundError, KeyError) as e:
            self._node.get_logger().error(
                f'GoalLocator: impossible de lire task_params.yaml ({e}). '
                f'Chemin tenté: {params_path}')
            raise

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # yaw non fourni pour le but, orientation neutre

        self._node.get_logger().info(
            f'GoalLocator: but réel lu depuis task_params.yaml '
            f'(x={goal_x:.3f}, y={goal_y:.3f})')
        return goal

    def _get_test_goal(self) -> PoseStamped:
        """Goal de secours pour développer/tester sans dépendre du fichier réel."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x = self._node.get_parameter('goal_x').value
        goal.pose.position.y = self._node.get_parameter('goal_y').value
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self._node.get_logger().warn('GoalLocator: goal de TEST utilisé (use_test_goal=True).')
        return goal
