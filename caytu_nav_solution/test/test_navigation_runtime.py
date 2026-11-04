"""Tests des briques extraites : chronomètre, bringup, but Nav2."""

import math
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace

from caytu_nav_solution.nav_math import Pose2D
from caytu_nav_solution.navigation_runtime import (
    BringupManager,
    construire_but,
    est_arrive,
    Nav2Navigator,
    secteur_arriere_est_libre,
    TaskTimer,
)
import pytest


class _FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return _FakeTime(self.nanoseconds - other.nanoseconds)


class _FakeClock:
    def __init__(self):
        self.nanoseconds = 0

    def now(self):
        return _FakeTime(self.nanoseconds)

    def advance(self, seconds):
        self.nanoseconds += int(seconds * 1e9)


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warn(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


class _FakeProcess:
    def __init__(self, pid=4242):
        self.pid = pid
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = 0
        return 0


class _ResistantProcess(_FakeProcess):
    def __init__(self):
        super().__init__()
        self.waits = 0

    def wait(self, timeout=None):
        self.waits += 1
        if self.waits < 3:
            raise subprocess.TimeoutExpired('ros2 launch', timeout)
        self.returncode = 0
        return 0


class _FakePopen:
    def __init__(self, process):
        self.process = process
        self.command = None
        self.kwargs = None

    def __call__(self, command, **kwargs):
        self.command = command
        self.kwargs = kwargs
        return self.process


def _navigation():
    return SimpleNamespace(
        bringup_package='caytu_nav_bringup',
        bringup_launch='solution_bringup.launch.py',
        bringup_stop_timeouts=(10.0, 5.0, 2.0),
    )


def test_le_chronometre_deduit_la_marge_et_signale_l_echeance():
    clock = _FakeClock()
    timer = TaskTimer(clock.now, 600.0, 1.0)

    assert timer.elapsed() == 0.0
    assert timer.remaining() == 599.0
    assert timer.start() is False          # horloge encore à zéro

    clock.advance(1.0)
    assert timer.start() is True
    clock.advance(10.0)
    assert timer.elapsed() == pytest.approx(10.0)
    assert timer.remaining() == pytest.approx(589.0)
    assert timer.expired() is False

    clock.advance(589.0)
    assert timer.expired() is True


def _scan_recul(ranges, angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                range_max=5.0):
    return SimpleNamespace(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.05,
        range_max=range_max,
    )


def _tf_identite(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=translation[0], y=translation[1]),
        rotation=SimpleNamespace(
            x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])))


def test_le_recul_refuse_si_un_obstacle_est_dans_le_secteur_arriere():
    ranges = [5.0] * 360
    ranges[0] = 0.5  # rayon centré vers l'arrière

    assert not secteur_arriere_est_libre(
        _scan_recul(ranges), _tf_identite(), 1.0, 0.7, 5, 0.2)


def test_le_recul_refuse_si_le_secteur_arriere_n_est_pas_couvert():
    scan = _scan_recul([5.0] * 30, angle_min=-0.5, angle_increment=0.03)

    assert not secteur_arriere_est_libre(
        scan, _tf_identite(), 1.0, 0.7, 5, 0.2)


def test_le_recul_accepte_un_secteur_complet_et_libre():
    assert secteur_arriere_est_libre(
        _scan_recul([5.0] * 360), _tf_identite(), 1.0, 0.7, 5, 0.2)


def test_le_recul_refuse_un_scan_dont_la_portee_est_trop_courte():
    assert not secteur_arriere_est_libre(
        _scan_recul([1.0] * 360, range_max=1.0),
        _tf_identite(translation=(0.2, 0.0, 0.0)), 1.0, 0.7, 5, 0.2)


def test_le_bringup_construit_une_commande_sans_shell_et_nettoie(tmp_path):
    process = _FakeProcess()
    popen = _FakePopen(process)
    manager = BringupManager(
        _navigation(), _FakeLogger(), popen=popen,
        killpg=lambda pgid, sig: None, getpgid=lambda pid: 99)
    carte = tmp_path / 'carte.yaml'

    manager.start(
        map_yaml=str(carte),
        use_sim_time=True,
        params_file='',
        controller='rpp',
        localization='odom_imu',
        profile='balanced',
        task_params_path='/tmp/task_params.yaml',
        spawn=Pose2D(1.5, -2.0, 0.5),
    )

    assert popen.kwargs == {'start_new_session': True}
    assert popen.command[:4] == [
        'ros2', 'launch', 'caytu_nav_bringup', 'solution_bringup.launch.py']
    assert 'use_sim_time:=true' in popen.command
    assert f'map:={carte}' in popen.command
    assert 'controller:=rpp' in popen.command
    assert 'localization:=odom_imu' in popen.command
    assert 'profile:=balanced' in popen.command
    assert 'task_params_file:=/tmp/task_params.yaml' in popen.command
    assert 'initial_pose_x:=1.5' in popen.command
    assert 'initial_pose_y:=-2.0' in popen.command
    assert manager.alive() is True

    params_file = next(
        argument.split(':=', 1)[1] for argument in popen.command
        if argument.startswith('params_file:='))
    dossier = Path(params_file).parent
    assert dossier.is_dir()

    manager.stop()
    assert dossier.is_dir() is False
    assert manager.alive() is False


def test_le_bringup_respecte_les_overrides_du_paquet_et_du_launch():
    popen = _FakePopen(_FakeProcess())
    manager = BringupManager(_navigation(), _FakeLogger(), popen=popen)

    manager.start(
        map_yaml='/tmp/carte.yaml', use_sim_time=False, params_file='',
        controller='rpp', localization='odom_imu', profile='balanced',
        task_params_path='/tmp/task_params.yaml', spawn=Pose2D(0.0, 0.0, 0.0),
        bringup_package='custom_bringup', bringup_launch='custom.launch.py')

    assert popen.command[2:4] == ['custom_bringup', 'custom.launch.py']
    manager.cleanup()


def test_l_arret_escalade_les_signaux_jusqu_a_sigkill():
    process = _ResistantProcess()
    popen = _FakePopen(process)
    signaux = []
    manager = BringupManager(
        _navigation(), _FakeLogger(), popen=popen,
        killpg=lambda pgid, sig: signaux.append(sig),
        getpgid=lambda pid: 99)

    manager.start(
        map_yaml='/tmp/carte.yaml', use_sim_time=False, params_file='',
        controller='rpp', localization='odom_imu', profile='balanced',
        task_params_path='/tmp/task_params.yaml', spawn=Pose2D(0.0, 0.0, 0.0))
    manager.stop()

    assert signaux == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]


def test_le_bringup_non_lance_reste_vivant_et_un_processus_mort_ne_l_est_plus():
    manager = BringupManager(_navigation(), _FakeLogger())
    assert manager.alive() is True

    process = _FakeProcess()
    manager = BringupManager(
        _navigation(), _FakeLogger(), popen=_FakePopen(process))
    manager.start(
        map_yaml='/tmp/carte.yaml', use_sim_time=False, params_file='',
        controller='rpp', localization='odom_imu', profile='balanced',
        task_params_path='/tmp/task_params.yaml', spawn=Pose2D(0.0, 0.0, 0.0))
    assert manager.alive() is True

    process.returncode = 1
    assert manager.alive() is False


def test_le_but_est_oriente_du_robot_vers_la_cible():
    but = construire_but(
        1.0, 0.0, 'map', None, Pose2D(0.0, 0.0, 0.0), None)
    assert but.pose.header.frame_id == 'map'
    assert but.pose.pose.position.x == 1.0
    assert but.pose.pose.orientation.z == pytest.approx(0.0)
    assert but.pose.pose.orientation.w == pytest.approx(1.0)

    but = construire_but(
        0.0, 1.0, 'map', None, Pose2D(0.0, 0.0, 0.0), None)
    assert but.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert but.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_le_rayon_d_arrivee_est_respecte():
    assert est_arrive(Pose2D(0.0, 0.0, 0.0), 0.1, 0.0, 0.2) is True
    assert est_arrive(Pose2D(0.0, 0.0, 0.0), 0.5, 0.0, 0.2) is False
    assert est_arrive(None, 0.0, 0.0, 0.2) is False


class _FakeFuture:
    def __init__(self, done, result=None):
        self._done = done
        self._result = result

    def done(self):
        return self._done

    def result(self):
        return self._result


class _FakeGoalHandle:
    def __init__(self, accepted=True, result=None):
        self.accepted = accepted
        self._result = result
        self.cancelled = False

    def get_result_async(self):
        return _FakeFuture(True, self._result)

    def cancel_goal_async(self):
        self.cancelled = True
        return _FakeFuture(True)


class _FakeNavClient:
    def __init__(self, handle):
        self.handle = handle
        self.sent = []

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent.append((goal, feedback_callback))
        return _FakeFuture(True, self.handle)


class _FakeRuntimeNode:
    def __init__(self):
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger


def _navigateur_sans_ros(timer, bringup):
    navigateur = object.__new__(Nav2Navigator)
    navigateur._timer = timer
    navigateur._bringup = bringup
    navigateur._spin = lambda timeout: None
    navigateur._is_ok = lambda: True
    return navigateur


def test_attendre_future_distingue_echeance_et_arret_du_bringup():
    timer = SimpleNamespace(remaining=lambda: 10.0)
    bringup = SimpleNamespace(alive=lambda: True)
    navigateur = _navigateur_sans_ros(timer, bringup)

    assert navigateur._attendre_future(_FakeFuture(True)) == 'ok'

    timer.remaining = lambda: -1.0
    assert navigateur._attendre_future(_FakeFuture(False)) == 'temps_ecoule'

    timer.remaining = lambda: 10.0
    bringup.alive = lambda: False
    assert navigateur._attendre_future(_FakeFuture(False)) == 'arret'


def test_le_navigateur_respecte_les_rayons_fournis_par_les_overrides_ros():
    navigation = SimpleNamespace(
        success_radius=0.2, arrival_recheck_radius=0.35)
    navigateur = object.__new__(Nav2Navigator)
    navigateur._navigation = navigation
    navigateur._success_radius = 0.3
    navigateur._arrival_recheck_radius = 0.4
    navigateur._pose_robot = lambda: Pose2D(0.25, 0.0, 0.0)

    assert navigateur.est_arrive(0.0, 0.0) is True
    assert navigateur._arrival_recheck_radius == 0.4


def test_le_navigateur_utilise_les_rayons_de_configuration_sans_override():
    navigation = SimpleNamespace(
        success_radius=0.2, arrival_recheck_radius=0.35)
    navigateur = object.__new__(Nav2Navigator)
    navigateur._navigation = navigation
    navigateur._success_radius = navigation.success_radius
    navigateur._arrival_recheck_radius = navigation.arrival_recheck_radius
    navigateur._pose_robot = lambda: Pose2D(0.25, 0.0, 0.0)

    assert navigateur.est_arrive(0.0, 0.0) is False
    assert navigateur._arrival_recheck_radius == 0.35


def _navigateur_pour_resultat(handle, pose):
    navigateur = object.__new__(Nav2Navigator)
    navigateur._client = _FakeNavClient(handle)
    navigateur._node = _FakeRuntimeNode()
    navigateur._navigation = SimpleNamespace(
        success_radius=0.2, arrival_recheck_radius=0.35,
        feedback_log_period_sec=5.0, cancel_timeout_sec=2.0)
    navigateur._success_radius = 0.2
    navigateur._arrival_recheck_radius = 0.35
    navigateur._timer = SimpleNamespace(elapsed=lambda: 1.0, remaining=lambda: 10.0)
    navigateur._bringup = SimpleNamespace(alive=lambda: True)
    navigateur._pose_robot = lambda: pose
    navigateur._spin = lambda timeout: None
    navigateur._is_ok = lambda: True
    navigateur._but_en_cours = None
    navigateur._dernier_log_feedback = 0.0
    navigateur.message_but = lambda *args: object()
    navigateur.recevoir_feedback = lambda message: None
    return navigateur


def test_le_navigateur_accepte_la_reussite_nav2_si_le_robot_est_proche():
    handle = _FakeGoalHandle(
        result=SimpleNamespace(status=4))  # GoalStatus.STATUS_SUCCEEDED
    navigateur = _navigateur_pour_resultat(handle, Pose2D(0.3, 0.0, 0.0))

    assert navigateur.naviguer_une_fois(0.0, 0.0, 'map', Pose2D(0, 0, 0)) == 'succes'
    assert len(navigateur._client.sent) == 1


def test_le_navigateur_renvoie_au_but_si_nav2_annonce_une_arrivee_trop_loin():
    handle = _FakeGoalHandle(
        result=SimpleNamespace(status=4))  # GoalStatus.STATUS_SUCCEEDED
    navigateur = _navigateur_pour_resultat(handle, Pose2D(0.5, 0.0, 0.0))

    assert navigateur.naviguer_une_fois(0.0, 0.0, 'map', Pose2D(0, 0, 0)) == 'reessayer'


def test_le_navigateur_signale_un_but_refuse_par_nav2():
    handle = _FakeGoalHandle(accepted=False)
    navigateur = _navigateur_pour_resultat(handle, Pose2D(0, 0, 0))

    assert navigateur.naviguer_une_fois(0.0, 0.0, 'map', Pose2D(0, 0, 0)) == 'refuse'
