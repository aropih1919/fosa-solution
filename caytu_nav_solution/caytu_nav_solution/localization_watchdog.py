#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

class LocalizationWatchdog(Node):
    def __init__(self):
        super().__init__('localization_watchdog')

        # Seuils d'entrée « prêt ». Plusieurs messages stables sont exigés pour
        # éviter de naviguer sur une unique estimation AMCL fortuite.
        self.declare_parameter('xy_covariance_threshold', 0.3)   # m^2
        self.declare_parameter('yaw_covariance_threshold', 0.15)  # rad^2
        self.declare_parameter('stability_samples', 5)

        # Hystérésis de perte : une covariance qui varie légèrement autour de
        # 0.3 ne doit pas annuler un goal valide. Une vraie perte persistante
        # reste arrêtée par un seuil distinct et plusieurs observations.
        self.declare_parameter('xy_loss_covariance_threshold', 0.5)  # m^2
        self.declare_parameter('yaw_loss_covariance_threshold', 0.25)  # rad^2
        self.declare_parameter('loss_samples', 5)

        self.xy_threshold = self.get_parameter('xy_covariance_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_covariance_threshold').value
        self.stability_samples = self.get_parameter('stability_samples').value
        self.xy_loss_threshold = self.get_parameter(
            'xy_loss_covariance_threshold'
        ).value
        self.yaw_loss_threshold = self.get_parameter(
            'yaw_loss_covariance_threshold'
        ).value
        self.loss_samples = self.get_parameter('loss_samples').value

        self.is_ready = False
        self._stable_samples = 0
        self._lost_samples = 0

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

        self.pub = self.create_publisher(Bool, '/localization_ready', 10)

        # republie l'état à 2 Hz, même sans nouveau message AMCL
        self.timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('Localization watchdog démarré, en attente de /amcl_pose...')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        # Indices de la matrice 6x6 (row-major) : xx=0, yy=7, yaw(theta-theta)=35
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yaw = cov[35]

        is_stable = bool(
            cov_xx < self.xy_threshold and
            cov_yy < self.xy_threshold and
            cov_yaw < self.yaw_threshold
        )

        if not self.is_ready:
            self._stable_samples = self._stable_samples + 1 if is_stable else 0
            if self._stable_samples < self.stability_samples:
                return

            self.is_ready = True
            self._lost_samples = 0
            self.get_logger().info(
                'Localisation CONFIRMÉE stable après '
                f'{self._stable_samples} mesures '
                f'(cov_xx={cov_xx:.4f}, cov_yy={cov_yy:.4f}, '
                f'cov_yaw={cov_yaw:.4f})'
            )
            return

        is_lost = bool(
            cov_xx > self.xy_loss_threshold or
            cov_yy > self.xy_loss_threshold or
            cov_yaw > self.yaw_loss_threshold
        )
        self._lost_samples = self._lost_samples + 1 if is_lost else 0
        if self._lost_samples >= self.loss_samples:
            self.is_ready = False
            self._stable_samples = 0
            self.get_logger().warn(
                'Localisation redevenue incertaine après '
                f'{self._lost_samples} mesures au-dessus du seuil de perte.'
            )

    def publish_status(self):
        msg = Bool()
        msg.data = self.is_ready
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = LocalizationWatchdog()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
