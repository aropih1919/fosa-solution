#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

class LocalizationWatchdog(Node):
    def __init__(self):
        super().__init__('localization_watchdog')

        # Seuils de confiance (à ajuster selon vos tests réels de Tâche 5)
        self.declare_parameter('xy_covariance_threshold', 0.05)   # m^2
        self.declare_parameter('yaw_covariance_threshold', 0.05)  # rad^2

        self.xy_threshold = self.get_parameter('xy_covariance_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_covariance_threshold').value

        self.is_ready = False

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

        was_ready = self.is_ready
        self.is_ready = bool(
            cov_xx < self.xy_threshold and
            cov_yy < self.xy_threshold and
            cov_yaw < self.yaw_threshold
        )

        if self.is_ready and not was_ready:
            self.get_logger().info(
                f'Localisation CONFIRMÉE stable (cov_xx={cov_xx:.4f}, '
                f'cov_yy={cov_yy:.4f}, cov_yaw={cov_yaw:.4f})')
        elif not self.is_ready and was_ready:
            self.get_logger().warn('Localisation redevenue incertaine.')

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