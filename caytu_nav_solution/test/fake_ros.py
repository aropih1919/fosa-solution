"""Faux ROS 2 minimal pour tester task_solution.py sans simulateur.

Seul ce que task_solution.py utilise est imité : horloge simulée, paramètres,
TF du robot, serveur d'action NavigateToPose scénarisé, ComputePathToPose,
services d'effacement des costmaps et costmap globale. Les messages ont des
champs fixes (`__slots__`) : une faute de frappe sur un nom de champ fait
échouer le test, comme elle ferait échouer le vrai nœud.

Utilisation : `world, module = load_task_solution()`.
"""
import importlib
import math
import sys
import types


# --------------------------------------------------------------------------- #
# Messages
# --------------------------------------------------------------------------- #
class _Msg:
    __slots__ = ()

    def __init__(self, **values):
        for name in self.__slots__:
            setattr(self, name, self._default(name))
        for name, value in values.items():
            setattr(self, name, value)

    def _default(self, name):
        return 0.0


class Stamp(_Msg):
    __slots__ = ('sec', 'nanosec')

    def _default(self, name):
        return 0


class Header(_Msg):
    __slots__ = ('stamp', 'frame_id')

    def _default(self, name):
        return Stamp() if name == 'stamp' else ''


class Vector3(_Msg):
    __slots__ = ('x', 'y', 'z')


class Quaternion(_Msg):
    __slots__ = ('x', 'y', 'z', 'w')

    def _default(self, name):
        return 1.0 if name == 'w' else 0.0


class Pose(_Msg):
    __slots__ = ('position', 'orientation')

    def _default(self, name):
        return Vector3() if name == 'position' else Quaternion()


class PoseStamped(_Msg):
    __slots__ = ('header', 'pose')

    def _default(self, name):
        return Header() if name == 'header' else Pose()


class Twist(_Msg):
    __slots__ = ('linear', 'angular')

    def _default(self, name):
        return Vector3()


class Transform(_Msg):
    __slots__ = ('translation', 'rotation')

    def _default(self, name):
        return Vector3() if name == 'translation' else Quaternion()


class TransformStamped(_Msg):
    __slots__ = ('header', 'child_frame_id', 'transform')

    def _default(self, name):
        return {'header': Header(), 'child_frame_id': '', 'transform': Transform()}[name]


class LaserScan(_Msg):
    __slots__ = ('header', 'angle_min', 'angle_max', 'angle_increment', 'ranges',
                 'range_min', 'range_max')

    def _default(self, name):
        return {'header': Header(), 'ranges': []}.get(name, 0.0)


class MapInfo(_Msg):
    __slots__ = ('width', 'height', 'resolution', 'origin')

    def _default(self, name):
        return Pose() if name == 'origin' else 0


class OccupancyGrid(_Msg):
    __slots__ = ('header', 'info', 'data')

    def _default(self, name):
        return {'header': Header(), 'info': MapInfo(), 'data': []}[name]


class Path(_Msg):
    __slots__ = ('header', 'poses')

    def _default(self, name):
        return Header() if name == 'header' else []


class GoalStatus:
    STATUS_UNKNOWN, STATUS_ACCEPTED, STATUS_EXECUTING = 0, 1, 2
    STATUS_CANCELING, STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 3, 4, 5, 6


def _action(name, goal_slots, result_slots, constants):
    goal = type(f'{name}_Goal', (_Msg,), {'__slots__': goal_slots})
    result_attrs = {'__slots__': result_slots, 'NONE': 0}
    result_attrs.update(constants)
    result = type(f'{name}_Result', (_Msg,), result_attrs)
    return type(name, (), {'Goal': goal, 'Result': result})


NavigateToPose = _action('NavigateToPose', ('pose', 'behavior_tree'),
                         ('error_code', 'error_msg'), {})
NavigateToPose.Goal._default = lambda self, name: '' if name == 'behavior_tree' else 0.0
ComputePathToPose = _action(
    'ComputePathToPose', ('goal', 'start', 'planner_id', 'use_start'),
    ('path', 'error_code'),
    {'UNKNOWN': 200, 'INVALID_PLANNER': 201, 'TF_ERROR': 202, 'START_OUTSIDE_MAP': 203,
     'GOAL_OUTSIDE_MAP': 204, 'START_OCCUPIED': 205, 'GOAL_OCCUPIED': 206, 'TIMEOUT': 207,
     'NO_VALID_PATH': 208})
FollowPath = _action(
    'FollowPath', ('path',), ('error_code',),
    {'UNKNOWN': 100, 'INVALID_CONTROLLER': 101, 'TF_ERROR': 102, 'INVALID_PATH': 103,
     'PATIENCE_EXCEEDED': 104, 'FAILED_TO_MAKE_PROGRESS': 105, 'NO_VALID_CONTROL': 106,
     'CONTROLLER_TIMED_OUT': 107})
Spin = _action('Spin', ('target_yaw',), ('error_code',),
               {'UNKNOWN': 700, 'TIMEOUT': 701, 'TF_ERROR': 702, 'COLLISION_AHEAD': 703})
BackUp = _action('BackUp', ('target',), ('error_code',),
                 {'UNKNOWN': 710, 'TIMEOUT': 711, 'TF_ERROR': 712, 'INVALID_INPUT': 713,
                  'COLLISION_AHEAD': 714})


def _service(name, request_slots):
    request = type(f'{name}_Request', (_Msg,), {'__slots__': request_slots})
    return type(name, (), {'Request': request})


ClearEntireCostmap = _service('ClearEntireCostmap', ())
GetState = _service('GetState', ())
ClearCostmapExceptRegion = _service('ClearCostmapExceptRegion', ('reset_distance',))
ClearCostmapAroundRobot = _service('ClearCostmapAroundRobot', ('reset_distance',))


# --------------------------------------------------------------------------- #
# Temps
# --------------------------------------------------------------------------- #
class Duration:
    def __init__(self, nanoseconds=0, seconds=0.0):
        self.nanoseconds = int(nanoseconds + seconds * 1e9)


class Time:
    def __init__(self, nanoseconds=0, **_):
        self.nanoseconds = int(nanoseconds)

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return Stamp(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)

    @staticmethod
    def from_msg(stamp):
        return Time(stamp.sec * 10**9 + stamp.nanosec)


class Future:
    def __init__(self):
        self._done, self._value, self._error = False, None, None

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return self._value

    def resolve(self, value):
        self._done, self._value = True, value

    def fail(self, error):
        self._done, self._error = True, error


# --------------------------------------------------------------------------- #
# Monde simulé
# --------------------------------------------------------------------------- #
class World:
    """Horloge, robot et réponses scénarisées de Nav2."""

    def __init__(self):
        self.t = 100.0                      # secondes simulées
        self.dt = 0.05
        self.robot = [0.0, 0.0, 0.0]
        self.tf_ok = True
        self.nav_ready = True
        self.nav_script = []                # un dict par but NavigateToPose
        self.sent_goals = []                # (x, y) de chaque but accepté
        self.sent_trees = []                # arbre de comportement demandé avec chaque but
        self.sent_times = []                # instant (simulé) de chaque but accepté
        self.keeper_ready = False           # le nœud path_keeper répond-il ?
        # État de cycle de vie des nœuds de Nav2 quand nav_ready est faux
        # (1 = non configuré) ; actifs (3) quand nav_ready est vrai.
        self.lifecycle_state = 1
        self.state_requests = 0
        self.cancels = 0
        self.clears = []                    # (service, reset_distance)
        self.plan_requests = []             # (planificateur, x, y)
        self.plan_ok = lambda planner, x, y: True
        self.costmap = None                 # OccupancyGrid servi à la demande
        self.costmap_callbacks = []
        self.published = []                 # (topic, message)
        self.logs = []                      # (niveau, texte)
        self._events = []                   # (instant, fonction)
        self._last_costmap_pub = -1e9

    # -- outils de scénario -------------------------------------------------
    def log_text(self):
        return '\n'.join(text for _, text in self.logs)

    def at(self, delay, function):
        self._events.append((self.t + delay, function))

    def step(self):
        self.t += self.dt
        due = [e for e in self._events if e[0] <= self.t]
        self._events = [e for e in self._events if e[0] > self.t]
        for _, function in sorted(due, key=lambda e: e[0]):
            function()
        if self.costmap is not None and self.costmap_callbacks \
                and self.t - self._last_costmap_pub >= 0.5:
            self._last_costmap_pub = self.t
            self.costmap.header.stamp = Time(int(self.t * 1e9)).to_msg()
            for callback in list(self.costmap_callbacks):
                callback(self.costmap)

    def set_costmap(self, values, width, height, resolution, origin_x, origin_y):
        grid = OccupancyGrid()
        grid.info.width, grid.info.height, grid.info.resolution = width, height, resolution
        grid.info.origin.position.x, grid.info.origin.position.y = origin_x, origin_y
        grid.data = list(values)
        self.costmap = grid

    # -- serveurs d'action --------------------------------------------------
    def _navigate(self, goal, feedback_callback):
        handle = GoalHandle(self)
        target = (goal.pose.pose.position.x, goal.pose.pose.position.y)
        self.sent_goals.append(target)
        self.sent_trees.append(goal.behavior_tree)
        self.sent_times.append(self.t)
        step = self.nav_script.pop(0) if self.nav_script else {}
        if step.get('reject'):
            handle.accepted = False
            return handle
        duration = float(step.get('duration', 2.0))
        if step.get('feedback', True):
            k = 0.5
            while k < duration:
                self.at(k, lambda: None if handle.finished else feedback_callback(
                    types.SimpleNamespace(feedback=types.SimpleNamespace(distance_remaining=1.0))))
                k += 0.5
        if step.get('never'):
            return handle                   # Nav2 accepte puis ne répond plus

        def finish():
            if handle.finished:
                return
            move_to = step.get('move_to', 'target')
            if move_to == 'target':
                self.robot[0], self.robot[1] = target
            elif move_to is not None:
                self.robot[0], self.robot[1] = move_to
            status = getattr(GoalStatus, 'STATUS_' + step.get('status', 'SUCCEEDED'))
            handle.finish(status, int(step.get('error_code', 0)))
        self.at(duration, finish)
        return handle

    def _compute_path(self, goal):
        handle = GoalHandle(self)
        x, y = goal.goal.pose.position.x, goal.goal.pose.position.y
        self.plan_requests.append((goal.planner_id, x, y))
        ok = bool(self.plan_ok(goal.planner_id, x, y))
        result = ComputePathToPose.Result()
        result.path = Path()
        result.path.poses = [PoseStamped(), PoseStamped()] if ok else []
        status = GoalStatus.STATUS_SUCCEEDED if ok else GoalStatus.STATUS_ABORTED
        self.at(0.1, lambda: handle.finish(status, 0 if ok else 208, result))
        return handle


class GoalHandle:
    def __init__(self, world):
        self._world = world
        self.accepted = True
        self.finished = False
        self._result = Future()

    def finish(self, status, error_code=0, result=None):
        if self.finished:
            return
        self.finished = True
        if result is None:
            result = NavigateToPose.Result()
        result.error_code = error_code
        self._result.resolve(types.SimpleNamespace(status=status, result=result))

    def get_result_async(self):
        return self._result

    def cancel_goal_async(self):
        self._world.cancels += 1
        future = Future()

        def done():
            self.finish(GoalStatus.STATUS_CANCELED)
            future.resolve(True)
        self._world.at(0.1, done)
        return future


class ActionClient:
    def __init__(self, node, action_type, name):
        self._world, self._name = node.world, name

    def server_is_ready(self):
        if self._name == 'compute_path_stable':
            return self._world.keeper_ready
        return self._world.nav_ready

    def wait_for_server(self, timeout_sec=None):
        return self._world.nav_ready

    def send_goal_async(self, goal, feedback_callback=None):
        future = Future()
        if self._name == 'navigate_to_pose':
            handle = self._world._navigate(goal, feedback_callback or (lambda message: None))
        else:
            handle = self._world._compute_path(goal)
        self._world.at(0.05, lambda: future.resolve(handle))
        return future


# --------------------------------------------------------------------------- #
# Nœud
# --------------------------------------------------------------------------- #
class _Logger:
    def __init__(self, world):
        self._world = world

    def _add(self, level, text):
        self._world.logs.append((level, str(text)))

    def info(self, text, **_):
        self._add('info', text)

    def warn(self, text, **_):
        self._add('warn', text)

    warning = warn

    def error(self, text, **_):
        self._add('error', text)


class Parameter:
    class Type:
        BOOL, DOUBLE, STRING, INTEGER = 'bool', 'double', 'string', 'integer'

    def __init__(self, name, type_=None, value=None):
        self.name, self.value = name, value


class _Client:
    def __init__(self, world, name):
        self._world, self._name = world, name

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        if self._name.endswith('/get_state'):
            self._world.state_requests += 1
            state = 3 if self._world.nav_ready else self._world.lifecycle_state
            future.resolve(types.SimpleNamespace(current_state=types.SimpleNamespace(id=state)))
            return future
        self._world.clears.append((self._name, getattr(request, 'reset_distance', None)))
        return future


class _Publisher:
    def __init__(self, world, topic):
        self._world, self._topic = world, topic

    def publish(self, message):
        self._world.published.append((self._topic, message))


class Node:
    world = None                            # fixé par load_task_solution()

    def __init__(self, name, parameter_overrides=None, **_):
        self._name = name
        self._parameters = {'use_sim_time': True}
        for parameter in parameter_overrides or []:
            self._parameters[parameter.name] = parameter.value
        self._logger = _Logger(self.world)

    def declare_parameter(self, name, value=None, descriptor=None):
        self._parameters.setdefault(name, value)
        return Parameter(name, value=self._parameters[name])

    def get_parameter(self, name):
        return Parameter(name, value=self._parameters[name])    # KeyError si non déclaré

    def get_logger(self):
        return self._logger

    def get_clock(self):
        world = self.world
        return types.SimpleNamespace(now=lambda: Time(int(round(world.t * 1e9))))

    def create_publisher(self, message_type, topic, qos):
        return _Publisher(self.world, topic)

    def create_subscription(self, message_type, topic, callback, qos):
        if message_type is OccupancyGrid:
            self.world.costmap_callbacks.append(callback)
        return (topic, callback)

    def destroy_subscription(self, subscription):
        return True

    def create_client(self, service_type, name):
        return _Client(self.world, name)

    def destroy_node(self):
        return True


class TransformException(Exception):
    pass


class Buffer:
    world = None

    def lookup_transform(self, target, source, time):
        if not self.world.tf_ok:
            raise TransformException('pas de transformation')
        message = TransformStamped()
        x, y, yaw = self.world.robot
        message.transform.translation.x, message.transform.translation.y = x, y
        message.transform.rotation.z = math.sin(yaw / 2.0)
        message.transform.rotation.w = math.cos(yaw / 2.0)
        return message


# --------------------------------------------------------------------------- #
# Installation des faux modules
# --------------------------------------------------------------------------- #
def _module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    return module


def load_task_solution():
    """Importe task_solution.py contre les faux modules. Retourne (monde, module)."""
    world = World()
    Node.world = world
    Buffer.world = world

    def spin_once(node, timeout_sec=None):
        world.step()

    fakes = {
        'rclpy': _module('rclpy', ok=lambda: True, spin_once=spin_once,
                         init=lambda *a, **k: None, shutdown=lambda *a, **k: None),
        'rclpy.action': _module('rclpy.action', ActionClient=ActionClient),
        'rclpy.node': _module('rclpy.node', Node=Node),
        'rclpy.parameter': _module('rclpy.parameter', Parameter=Parameter),
        'rcl_interfaces': _module('rcl_interfaces'),
        'rcl_interfaces.msg': _module(
            'rcl_interfaces.msg', ParameterDescriptor=lambda **options: options),
        'rclpy.qos': _module(
            'rclpy.qos', QoSProfile=lambda **k: k, qos_profile_sensor_data='sensor',
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),
            DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1)),
        'rclpy.signals': _module('rclpy.signals',
                                 SignalHandlerOptions=types.SimpleNamespace(NO=0)),
        'rclpy.time': _module('rclpy.time', Time=Time),
        'lifecycle_msgs': _module('lifecycle_msgs'),
        'lifecycle_msgs.srv': _module('lifecycle_msgs.srv', GetState=GetState),
        'action_msgs': _module('action_msgs'),
        'action_msgs.msg': _module('action_msgs.msg', GoalStatus=GoalStatus),
        'geometry_msgs': _module('geometry_msgs'),
        'geometry_msgs.msg': _module('geometry_msgs.msg', PoseStamped=PoseStamped, Twist=Twist),
        'nav2_msgs': _module('nav2_msgs'),
        'nav2_msgs.action': _module(
            'nav2_msgs.action', NavigateToPose=NavigateToPose,
            ComputePathToPose=ComputePathToPose, FollowPath=FollowPath, Spin=Spin,
            BackUp=BackUp),
        'nav2_msgs.srv': _module(
            'nav2_msgs.srv', ClearEntireCostmap=ClearEntireCostmap,
            ClearCostmapExceptRegion=ClearCostmapExceptRegion,
            ClearCostmapAroundRobot=ClearCostmapAroundRobot),
        'nav_msgs': _module('nav_msgs'),
        'nav_msgs.msg': _module('nav_msgs.msg', OccupancyGrid=OccupancyGrid),
        'sensor_msgs': _module('sensor_msgs'),
        'sensor_msgs.msg': _module('sensor_msgs.msg', LaserScan=LaserScan),
        'tf2_ros': _module('tf2_ros', Buffer=Buffer, TransformException=TransformException,
                           TransformListener=lambda *a, **k: None),
        # Absent : task_solution.py doit s'en passer (contacts non comptés).
        'ros_gz_interfaces': None,
        'ros_gz_interfaces.msg': None,
    }
    name = 'caytu_nav_solution.task_solution'
    saved = {key: sys.modules.get(key) for key in list(fakes) + [name]}
    try:
        for key, module in fakes.items():
            sys.modules[key] = module
        sys.modules.pop(name, None)
        module = importlib.import_module(name)
    finally:
        for key, previous in saved.items():
            if previous is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = previous
    # Le temps « réel » du module suit l'horloge simulée : aucun test n'attend.
    module.time = types.SimpleNamespace(monotonic=lambda: world.t)
    module.rclpy = fakes['rclpy']
    return world, module
