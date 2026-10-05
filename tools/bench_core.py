"""Calculs du benchmark, sans aucune dépendance ROS.

Tout ce qui est mathématique ou statistique est ici, pour pouvoir être testé
sur n'importe quel ordinateur (voir test_bench_core.py).
"""

import csv
import math
import statistics

# Enveloppe réelle du robot dans le repère base_footprint, mesurée depuis
# l'URDF (voir le commentaire de costmap_common_params.yaml).
ROBOT_X_MIN = -0.288
ROBOT_X_MAX = 0.198
ROBOT_Y_HALF = 0.222

STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}
TERMINAL_STATUSES = (4, 5, 6)


def yaw_from_quaternion(x, y, z, w):
    """Angle de rotation autour de l'axe vertical (en radians)."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def normalize_angle(angle):
    """Ramène un angle dans l'intervalle [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_error(estimated, truth):
    """Écart entre deux poses (x, y, yaw) : (distance en m, écart d'angle en rad)."""
    xy_error = math.hypot(estimated[0] - truth[0], estimated[1] - truth[1])
    yaw_error = abs(normalize_angle(estimated[2] - truth[2]))
    return xy_error, yaw_error


def extract_pose(msg, name_keywords=('sitoe', 'base_footprint')):
    """Lit (x, y, yaw) dans différents types de messages de pose.

    Types gérés : Pose, PoseStamped, Odometry, PoseWithCovarianceStamped et
    TFMessage (on prend alors la transformée dont le nom d'enfant contient un des
    mots de name_keywords). Retourne None si rien d'exploitable.
    """
    transforms = getattr(msg, 'transforms', None)
    if transforms is not None:
        for item in transforms:
            child = (getattr(item, 'child_frame_id', '') or '').lower()
            if any(keyword in child for keyword in name_keywords):
                t = item.transform.translation
                q = item.transform.rotation
                return (t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        return None

    pose = msg
    for _ in range(2):  # PoseStamped : 1 niveau ; Odometry et PoseWithCovariance : 2 niveaux
        if hasattr(pose, 'pose'):
            pose = pose.pose
    if not hasattr(pose, 'position') or not hasattr(pose, 'orientation'):
        return None
    q = pose.orientation
    return (pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


def distance_to_robot_body(goal_x, goal_y, robot_x, robot_y, robot_yaw):
    """Distance entre le centre du but et le point le plus proche du robot.

    Le règlement dit que la tâche est finie quand N'IMPORTE QUELLE PARTIE du
    robot est dans le cercle vert. Cette distance doit donc être inférieure
    ou égale au rayon du cercle. Elle vaut 0 si le centre du but est sous le
    robot.
    """
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)

    # Position du but vue depuis le robot
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy

    # Point du rectangle du robot le plus proche du but
    nearest_x = min(max(local_x, ROBOT_X_MIN), ROBOT_X_MAX)
    nearest_y = min(max(local_y, -ROBOT_Y_HALF), ROBOT_Y_HALF)

    return math.hypot(local_x - nearest_x, local_y - nearest_y)


def is_real_contact(name1, name2, ignore_keywords, robot_keyword='sitoe'):
    """Dit si un contact signalé par Gazebo est une vraie collision.

    Ne comptent pas :
    - un contact du robot avec lui-même (les deux noms contiennent le nom du robot) ;
    - un contact avec le sol (les roues touchent toujours le sol).
    """
    n1 = (name1 or '').lower()
    n2 = (name2 or '').lower()

    if robot_keyword and robot_keyword in n1 and robot_keyword in n2:
        return False

    for keyword in ignore_keywords:
        if keyword and (keyword in n1 or keyword in n2):
            return False

    return True


class ContactCounter:
    """Compte des ÉPISODES de contact, pas des messages.

    Les capteurs publient à 5 Hz : un robot collé à un mur pendant 10 s donne
    50 messages mais reste UN seul contact. Deux messages séparés de plus de
    gap_sec secondes forment deux épisodes différents.
    """

    def __init__(self, gap_sec=1.0):
        self.gap_sec = gap_sec
        self.episodes = 0
        self.messages = 0
        self._last_time = None

    def add(self, sim_time):
        """Enregistre un contact. Retourne True si c'est un nouvel épisode."""
        new_episode = (
            self._last_time is None or sim_time - self._last_time > self.gap_sec
        )
        if new_episode:
            self.episodes += 1
        self._last_time = sim_time
        self.messages += 1
        return new_episode


def _values(runs, key):
    values = []
    for run in runs:
        value = run.get(key)
        if value is not None:
            values.append(value)
    return values


def _mean(values):
    if not values:
        return None
    return round(sum(values) / len(values), 2)


def _median(values):
    if not values:
        return None
    return round(statistics.median(values), 2)


def summarize_runs(runs):
    """Regroupe les runs par label et calcule les moyennes.

    Retourne une liste de dictionnaires, une ligne par label.
    """
    groups = {}
    for run in runs:
        groups.setdefault(run.get('label', '?'), []).append(run)

    rows = []
    for label in sorted(groups):
        items = groups[label]
        successes = [r for r in items if r.get('result') == 'SUCCESS']

        rows.append({
            'label': label,
            'runs': len(items),
            'success': len(successes),
            'success_rate': round(len(successes) / len(items), 2),
            'time_median_s': _median(_values(successes, 'time_sec')),
            'time_mean_s': _mean(_values(successes, 'time_sec')),
            'final_dist_mean_m': _mean(_values(items, 'final_center_distance')),
            'progress_mean_m': _mean(_values(items, 'progress_m')),
            'speed_mean_mps': _mean(_values(items, 'average_speed_mps')),
            'contacts_mean': _mean(_values(items, 'contacts_episodes')),
            'recoveries_mean': _mean(_values(items, 'recoveries')),
            'cov_max_mean': _mean(_values(items, 'amcl_cov_max_xy')),
            'loc_err_max_mean_m': _mean(_values(items, 'loc_error_max_xy')),
            'commits': ','.join(sorted({str(r.get('git_commit', '?')) for r in items})),
        })
    return rows


SUMMARY_COLUMNS = [
    'label', 'runs', 'success', 'success_rate', 'time_median_s', 'time_mean_s',
    'final_dist_mean_m', 'progress_mean_m', 'speed_mean_mps', 'contacts_mean',
    'recoveries_mean', 'cov_max_mean', 'loc_err_max_mean_m', 'commits',
]


def write_summary_csv(rows, path):
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def format_table(rows):
    """Texte aligné pour l'affichage dans le terminal."""
    if not rows:
        return 'Aucun résultat.'

    widths = {}
    for column in SUMMARY_COLUMNS:
        widths[column] = len(column)
        for row in rows:
            widths[column] = max(widths[column], len(str(row[column])))

    lines = []
    header = '  '.join(column.ljust(widths[column]) for column in SUMMARY_COLUMNS)
    lines.append(header)
    lines.append('-' * len(header))
    for row in rows:
        lines.append(
            '  '.join(str(row[column]).ljust(widths[column]) for column in SUMMARY_COLUMNS)
        )
    return '\n'.join(lines)
