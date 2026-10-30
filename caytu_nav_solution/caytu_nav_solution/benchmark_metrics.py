"""Collecte ROS des métriques de navigation sans modifier les commandes du robot."""

import math
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # ros_gz_interfaces is optional outside Gazebo.
    Contacts = None

try:
    import psutil
except ImportError:  # Keep navigation available when host metrics are unavailable.
    psutil = None


CONTACT_TOPICS = (
    "/base_collisions",
    "/top_chassis_collisions",
    "/left_wheel_collisions",
    "/right_wheel_collisions",
)
SCAN_TOPICS = ("/scan_clean", "/top_camera_scan", "/bottom_camera_scan")
BEHAVIOR_ACTIONS = ("/spin/_action/status", "/backup/_action/status", "/wait/_action/status")


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def distance_between(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def pose_errors(truth, estimate):
    """Retourne l'erreur euclidienne (m) et angulaire (rad) entre deux poses x/y/yaw."""
    position = distance_between(truth[0], truth[1], estimate[0], estimate[1])
    delta = truth[2] - estimate[2]
    angle = abs(math.atan2(math.sin(delta), math.cos(delta)))
    return position, angle


def travelled_distance(truth_available, odom_available, truth_distance, odom_distance):
    """Choisit la meilleure distance et sa provenance, ou N/A sans échantillon."""
    if truth_available:
        return truth_distance, "gazebo_truth_pose"
    if odom_available:
        return odom_distance, "/odom"
    return None, None


class CollisionEpisodeCounter:
    """Regroupe les messages répétés d'un même contact en un épisode."""

    def __init__(self, episode_gap_sec=0.5):
        self.episode_gap_sec = float(episode_gap_sec)
        self.last_seen = {}
        self.episodes = 0
        self.message_count = 0
        self.topics_seen = set()

    def update(self, topic, contacts, now=None):
        now = time.monotonic() if now is None else float(now)
        self.message_count += 1
        self.topics_seen.add(topic)
        for contact in contacts:
            first = getattr(getattr(contact, "collision1", None), "name", "")
            second = getattr(getattr(contact, "collision2", None), "name", "")
            if not first or not second:
                continue
            names = (first.lower(), second.lower())
            if any("floor" in name or "ground" in name for name in names):
                continue
            key = tuple(sorted((first, second)))
            previous = self.last_seen.get(key)
            if previous is None or now - previous > self.episode_gap_sec:
                self.episodes += 1
            self.last_seen[key] = now


class BenchmarkMetrics(Node):
    """ROS subscriber node used by fosa_benchmark while task_solution runs."""

    def __init__(self, localization, goal_x, goal_y, use_sim_time=True):
        super().__init__(
            "fosa_benchmark_metrics",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", value=bool(use_sim_time))],
        )
        self.localization = localization
        self.goal = (float(goal_x), float(goal_y))
        self.use_sim_time = bool(use_sim_time)
        self.started_monotonic = None
        self.started_ros = None
        self.mission_started_monotonic = None
        self.mission_started_ros = None
        self.finished_monotonic = None
        self.finished_ros = None
        self.first_navigation_ros = None
        self.localization_ready_monotonic = None

        self.truth = None
        self.estimate = None
        self.odom_pose = None
        self.odom_distance_m = 0.0
        self.truth_distance_m = 0.0
        self.estimate_distance_m = 0.0
        self._previous_odom = None
        self._previous_truth = None
        self._previous_estimate = None
        self._previous_odom_time = None
        self._speed_integral = 0.0
        self._speed_time = 0.0
        self._stopped_time = 0.0
        self._max_speed = None
        self._latest_linear_speed = None
        self._latest_angular_speed = None
        self._latest_command_linear = None
        self._latest_command_angular = None
        self._angular_sign = 0
        self._direction_changes = 0
        self._turn_samples = deque()
        self._last_oscillation_time = -math.inf
        self._oscillation_count = 0

        self.minimum_obstacle_distance_m = None
        self.scan_topics_received = set()
        self.position_errors = []
        self.angle_errors = []
        self.amcl_covariance_xy = []
        self.nav_goal_ids = set()
        self.recovery_goal_ids = set()
        self.nav_status_messages = 0
        self.nav_goal_state = "en attente"
        self.recovery_status_messages = 0
        self.localization_samples = 0
        self.tf_samples = 0
        self.tf_missing_samples = 0
        self.sample_rows = []
        self._last_local_plan_stamp = None
        self._local_plan_pose_count = 0
        self._global_plan_stamp = None
        self._global_plan_pose_count = 0
        self._local_plan_intervals = []
        self._tf_last_available = False
        self._command_samples = 0
        self._contact_counter = CollisionEpisodeCounter()
        self._resource_cpu = []
        self._resource_ram_mb = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Pose, "/sitoe_robot/pose", self._on_truth, 10)
        self.create_subscription(
            PoseStamped, "/localization_pose", self._on_localization_pose, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        self.create_subscription(
            Twist, "/robot_base_controller/cmd_vel_unstamped", self._on_command, 10)
        self.create_subscription(Path, "/plan", self._on_global_plan, 10)
        self.create_subscription(Path, "/local_plan", self._on_local_plan, 10)
        for topic in SCAN_TOPICS:
            self.create_subscription(
                LaserScan, topic,
                lambda message, source=topic: self._on_scan(source, message),
                qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/scan", self._on_raw_scan, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_nav_status, 10)
        for topic in BEHAVIOR_ACTIONS:
            self.create_subscription(
                GoalStatusArray, topic, self._on_recovery_status, 10)
        if Contacts is not None:
            for topic in CONTACT_TOPICS:
                self.create_subscription(
                    Contacts,
                    topic,
                    lambda message, source=topic: self._on_contacts(source, message),
                    10,
                )
        self.create_timer(0.5, self._record_sample)

    def _ros_seconds(self):
        stamp = self.get_clock().now().nanoseconds
        return stamp * 1e-9 if stamp > 0 else None

    def mark_experiment_start(self):
        self.started_monotonic = time.monotonic()
        self.started_ros = self._ros_seconds()

    def mark_mission_start(self):
        self.mission_started_monotonic = time.monotonic()
        self.mission_started_ros = self._ros_seconds()

    def mark_finished(self):
        self.finished_monotonic = time.monotonic()
        self.finished_ros = self._ros_seconds()

    def _stamp_seconds(self, stamp):
        value = stamp.sec + stamp.nanosec * 1e-9
        return value if value > 0.0 else self._ros_seconds()

    def _on_odom(self, message):
        pose = message.pose.pose
        current = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        stamp = self._stamp_seconds(message.header.stamp)
        self.odom_pose = current
        if self._previous_odom is not None:
            dx = current[0] - self._previous_odom[0]
            dy = current[1] - self._previous_odom[1]
            self.odom_distance_m += math.hypot(dx, dy)
        self._previous_odom = current

        speed = abs(float(message.twist.twist.linear.x))
        self._latest_linear_speed = float(message.twist.twist.linear.x)
        self._latest_angular_speed = float(message.twist.twist.angular.z)
        if self._previous_odom_time is not None and stamp is not None:
            dt = stamp - self._previous_odom_time
            if 0.0 < dt < 1.0:
                self._speed_integral += speed * dt
                self._speed_time += dt
                if speed < 0.01:
                    self._stopped_time += dt
        self._previous_odom_time = stamp
        self._max_speed = speed if self._max_speed is None else max(self._max_speed, speed)
        self._track_turns(message.twist.twist.angular.z, current, stamp)

    def _track_turns(self, angular_speed, pose, stamp):
        sign = 1 if angular_speed > 0.1 else (-1 if angular_speed < -0.1 else 0)
        if sign and self._angular_sign and sign != self._angular_sign:
            self._direction_changes += 1
        if sign:
            self._angular_sign = sign
        if stamp is None:
            return
        self._turn_samples.append((stamp, sign, pose[0], pose[1]))
        while self._turn_samples and stamp - self._turn_samples[0][0] > 3.0:
            self._turn_samples.popleft()
        flips = sum(
            1 for previous, current in zip(self._turn_samples, list(self._turn_samples)[1:])
            if previous[1] and current[1] and previous[1] != current[1]
        )
        if (flips >= 4 and stamp - self._last_oscillation_time >= 3.0
                and self._turn_samples
                and distance_between(self._turn_samples[0][2], self._turn_samples[0][3],
                                     pose[0], pose[1]) < 0.15):
            self._oscillation_count += 1
            self._last_oscillation_time = stamp

    def _on_truth(self, message):
        current = (message.position.x, message.position.y, yaw_from_quaternion(message.orientation))
        stamp = self._ros_seconds()
        self.truth = (stamp, *current)
        if self._previous_truth is not None:
            self.truth_distance_m += distance_between(
                self._previous_truth[0], self._previous_truth[1], current[0], current[1])
        self._previous_truth = current
        self._compare_localization()

    def _store_estimate(self, stamp, pose):
        self.estimate = (stamp, *pose)
        if self._previous_estimate is not None:
            self.estimate_distance_m += distance_between(
                self._previous_estimate[0], self._previous_estimate[1], pose[0], pose[1])
        self._previous_estimate = pose
        self.localization_samples += 1
        if self.localization_ready_monotonic is None:
            self.localization_ready_monotonic = time.monotonic()
        self._compare_localization()

    def _on_localization_pose(self, message):
        if self.localization == "amcl":
            return
        pose = message.pose
        self._store_estimate(
            self._stamp_seconds(message.header.stamp),
            (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)),
        )

    def _on_amcl_pose(self, message):
        if self.localization != "amcl":
            return
        pose = message.pose.pose
        self._store_estimate(
            self._stamp_seconds(message.header.stamp),
            (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)),
        )
        covariance = message.pose.covariance
        if len(covariance) >= 36:
            self.amcl_covariance_xy.append((max(0.0, covariance[0]) +
                                            max(0.0, covariance[7])) ** 0.5)

    def _compare_localization(self):
        if self.truth is None or self.estimate is None:
            return
        if self.truth[0] is None or self.estimate[0] is None:
            return
        if abs(self.truth[0] - self.estimate[0]) > 0.25:
            return
        position_error, angle_error = pose_errors(
            self.truth[1:4], self.estimate[1:4])
        self.position_errors.append(position_error)
        self.angle_errors.append(angle_error)

    def _on_scan(self, topic, message):
        self.scan_topics_received.add(topic)
        values = [value for value in message.ranges
                  if math.isfinite(value) and message.range_min <= value <= message.range_max]
        if values:
            value = min(values)
            if (self.minimum_obstacle_distance_m is None
                    or value < self.minimum_obstacle_distance_m):
                self.minimum_obstacle_distance_m = value

    def _on_raw_scan(self, _message):
        # Le scan brut confirme seulement que Gazebo est prêt. Les métriques
        # d'obstacle utilisent les scans nettoyés reçus après le lancement Fosa.
        self.scan_topics_received.add("/scan")

    def _on_command(self, message):
        self._command_samples += 1
        self._latest_command_linear = float(message.linear.x)
        self._latest_command_angular = float(message.angular.z)

    def _on_local_plan(self, message):
        self._local_plan_pose_count = len(message.poses)
        stamp = self._stamp_seconds(message.header.stamp)
        if stamp is not None and self._last_local_plan_stamp is not None:
            interval = stamp - self._last_local_plan_stamp
            if 0.0 < interval < 2.0:
                self._local_plan_intervals.append(interval)
        self._last_local_plan_stamp = stamp

    def _on_global_plan(self, message):
        self._global_plan_pose_count = len(message.poses)
        self._global_plan_stamp = self._stamp_seconds(message.header.stamp)

    def _goal_ids(self, message):
        return [bytes(item.goal_info.goal_id.uuid) for item in message.status_list
                if item.status == GoalStatus.STATUS_EXECUTING]

    def _on_nav_status(self, message):
        self.nav_status_messages += 1
        statuses = [item.status for item in message.status_list]
        status_names = {
            GoalStatus.STATUS_UNKNOWN: "inconnu",
            GoalStatus.STATUS_ACCEPTED: "goal reçu",
            GoalStatus.STATUS_EXECUTING: "navigation active",
            GoalStatus.STATUS_CANCELING: "annulation",
            GoalStatus.STATUS_SUCCEEDED: "mission réussie",
            GoalStatus.STATUS_CANCELED: "goal annulé",
            GoalStatus.STATUS_ABORTED: "goal échoué",
        }
        active_statuses = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = [status for status in statuses if status in active_statuses]
        if active:
            self.nav_goal_state = status_names[active[-1]]
        elif statuses:
            self.nav_goal_state = status_names.get(statuses[-1], "statut reçu")
        ids = self._goal_ids(message)
        if ids and self.first_navigation_ros is None:
            self.first_navigation_ros = self._ros_seconds()
        self.nav_goal_ids.update(ids)

    def _on_recovery_status(self, message):
        self.recovery_status_messages += 1
        self.recovery_goal_ids.update(self._goal_ids(message))

    def live_status(self, phase):
        """État compact sur une ligne pour suivre le robot en direct."""
        now = time.monotonic()
        start = self.started_monotonic or now
        elapsed = max(0, int(now - start))
        if self.estimate is not None:
            pose = self.estimate[1:4]
            pose_frame = "map"
            goal_distance = distance_between(pose[0], pose[1], self.goal[0], self.goal[1])
        elif self.odom_pose is not None:
            pose = self.odom_pose
            pose_frame = "odom"
            goal_distance = None
        else:
            pose = None
            pose_frame = "pose"
            goal_distance = None
        if pose is None:
            pose_text = "pose=N/A"
        else:
            pose_text = (
                f"{pose_frame}=({pose[0]:+.2f},{pose[1]:+.2f},"
                f"{math.degrees(pose[2]):+.0f}deg)"
            )
        goal_text = f"but={goal_distance:.2f}m" if goal_distance is not None else "but=N/A"
        odom_v = "N/A" if self._latest_linear_speed is None else f"{self._latest_linear_speed:.2f}"
        odom_w = "N/A" if self._latest_angular_speed is None else f"{self._latest_angular_speed:+.2f}"
        cmd_v = "N/A" if self._latest_command_linear is None else f"{self._latest_command_linear:.2f}"
        cmd_w = "N/A" if self._latest_command_angular is None else f"{self._latest_command_angular:+.2f}"
        global_plan = (f"reçu ({self._global_plan_pose_count} poses)"
                       if self._global_plan_stamp is not None else "en attente")
        local_plan = (f"reçu ({self._local_plan_pose_count} poses)"
                      if self._last_local_plan_stamp is not None else "non publié")
        localization = "OK" if self.localization_samples else "attente"
        tf_current = "OK" if self._tf_last_available else "MANQUANTE"
        obstacle = ("N/A" if self.minimum_obstacle_distance_m is None else
                    f"{self.minimum_obstacle_distance_m:.2f}m")
        collisions = (str(self._contact_counter.episodes)
                      if self._contact_counter.topics_seen else "N/A")
        local_rate = (f"{1.0 / sum(self._local_plan_intervals) * len(self._local_plan_intervals):.1f} Hz"
                      if self._local_plan_intervals else "N/A")
        return (
            f"[MONITOR {phase} {elapsed:02d}s] {pose_text} {goal_text} "
            f"odom v/w={odom_v}/{odom_w}m/s,rad/s "
            f"cmd={cmd_v}/{cmd_w} goal={self.nav_goal_state} "
            f"path=/plan:{global_plan}, local:{local_plan} "
            f"loc={localization} TF={self.tf_missing_samples}/{self.tf_samples} "
            f"manquantes (actuel:{tf_current}) scan_min={obstacle} "
            f"collisions={collisions} distance_odom={self.odom_distance_m:.2f}m "
            f"freq_local={local_rate}"
        )

    def _on_contacts(self, topic, message):
        self._contact_counter.update(topic, message.contacts)

    def check_tf(self):
        self.tf_samples += 1
        try:
            self._tf_buffer.lookup_transform("map", "base_footprint", Time())
            self._tf_last_available = True
        except TransformException:
            self.tf_missing_samples += 1
            self._tf_last_available = False

    def sample_resources(self):
        if psutil is None:
            return
        self._resource_cpu.append(float(psutil.cpu_percent(interval=None)))
        self._resource_ram_mb.append(float(psutil.virtual_memory().used) / (1024 * 1024))

    def _record_sample(self):
        if self.mission_started_ros is None:
            return
        now = self._ros_seconds()
        if now is None:
            return
        row = {"time_sec": max(0.0, now - self.mission_started_ros)}
        for prefix, sample in (("truth", self.truth), ("estimate", self.estimate)):
            row[f"{prefix}_x"] = sample[1] if sample else None
            row[f"{prefix}_y"] = sample[2] if sample else None
            row[f"{prefix}_yaw"] = sample[3] if sample else None
        row["odom_x"] = self.odom_pose[0] if self.odom_pose else None
        row["odom_y"] = self.odom_pose[1] if self.odom_pose else None
        row["minimum_obstacle_distance_m"] = self.minimum_obstacle_distance_m
        self.sample_rows.append(row)

    @staticmethod
    def _mean(values):
        return sum(values) / len(values) if values else None

    @staticmethod
    def _rmse(values):
        return math.sqrt(sum(value * value for value in values) / len(values)) if values else None

    def summary(self, mission_success, exit_code, error_message=None):
        end_mono = self.finished_monotonic or time.monotonic()
        start_mono = self.mission_started_monotonic
        real_duration = end_mono - start_mono if start_mono is not None else None
        ros_duration = None
        if self.mission_started_ros is not None and self.finished_ros is not None:
            delta = self.finished_ros - self.mission_started_ros
            if delta >= 0.0:
                ros_duration = delta
        elapsed = ros_duration if self.use_sim_time and ros_duration is not None else real_duration

        final_pose = self.truth or self.estimate
        final_goal_distance = None
        final_goal_source = None
        if final_pose is not None:
            final_goal_distance = distance_between(
                final_pose[1], final_pose[2], self.goal[0], self.goal[1])
            final_goal_source = "gazebo_truth_pose" if self.truth is not None else "localization_pose"

        collision_count = (self._contact_counter.episodes
                           if self._contact_counter.message_count else None)
        distance, distance_source = travelled_distance(
            self.truth is not None,
            self._previous_odom is not None,
            self.truth_distance_m,
            self.odom_distance_m,
        )
        retry_count = max(0, len(self.nav_goal_ids) - 1) if self.nav_status_messages else None
        recovery_count = (len(self.recovery_goal_ids)
                          if self.recovery_status_messages else None)
        navigation_elapsed = None
        if self.first_navigation_ros is not None and self.finished_ros is not None:
            delta = self.finished_ros - self.first_navigation_ros
            navigation_elapsed = delta if delta >= 0.0 else None

        return {
            "mission_success": bool(mission_success),
            "exit_code": exit_code,
            "error_message": error_message,
            "execution_time_sec": elapsed,
            "execution_time_source": "ros_sim_time" if self.use_sim_time and ros_duration is not None
            else "monotonic_wall_time",
            "execution_time_real_sec": real_duration,
            "navigation_time_sec": navigation_elapsed,
            "localization_initialization_time_sec": (
                self.localization_ready_monotonic - start_mono
                if self.localization_ready_monotonic is not None and start_mono is not None
                else None
            ),
            "collision_count": collision_count,
            "collision_data_source": ",".join(sorted(self._contact_counter.topics_seen))
            if self._contact_counter.topics_seen else None,
            "minimum_obstacle_distance_m": self.minimum_obstacle_distance_m,
            "travelled_distance_m": distance,
            "travelled_distance_source": distance_source,
            "travelled_distance_estimate_m": self.estimate_distance_m
            if self._previous_estimate is not None else None,
            "final_goal_distance_m": final_goal_distance,
            "final_goal_distance_source": final_goal_source,
            "retry_count": retry_count,
            "recovery_count": recovery_count,
            "turn_direction_changes": self._direction_changes,
            "oscillation_episodes_estimated": self._oscillation_count,
            "stopped_time_sec": self._stopped_time if self._speed_time else None,
            "average_speed_mps": self._speed_integral / self._speed_time
            if self._speed_time else None,
            "maximum_speed_mps": self._max_speed,
            "system_cpu_average_percent": self._mean(self._resource_cpu),
            "system_cpu_maximum_percent": max(self._resource_cpu) if self._resource_cpu else None,
            "system_ram_average_mb": self._mean(self._resource_ram_mb),
            "controller_rate_hz": 1.0 / self._mean(self._local_plan_intervals)
            if self._local_plan_intervals else None,
            "localization_status": "available" if self.localization_samples else "unavailable",
            "localization_pose_samples": self.localization_samples,
            "position_error_mean_m": self._mean(self.position_errors),
            "position_error_max_m": max(self.position_errors) if self.position_errors else None,
            "position_rmse_m": self._rmse(self.position_errors),
            "angle_error_mean_rad": self._mean(self.angle_errors),
            "angle_error_max_rad": max(self.angle_errors) if self.angle_errors else None,
            "angle_rmse_rad": self._rmse(self.angle_errors),
            "amcl_covariance_xy_mean_sqrt_m": self._mean(self.amcl_covariance_xy),
            "tf_lookup_samples": self.tf_samples,
            "tf_missing_samples": self.tf_missing_samples,
            "tf_missing_rate": self.tf_missing_samples / self.tf_samples if self.tf_samples else None,
            "ros_contact_messages": self._contact_counter.message_count,
        }
