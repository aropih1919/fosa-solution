"""Tests du chronomètre de course et du client Nav2."""

import math
from types import SimpleNamespace

from caytu_nav_solution.nav_math import Pose2D
from caytu_nav_solution.navigation_runtime import (
    ChronometreCourse,
    construire_but,
    est_arrive,
    NavigateurNav2,
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


def test_le_chronometre_deduit_la_marge_et_signale_l_echeance():
    clock = _FakeClock()
    chrono = ChronometreCourse(clock.now, 600.0, 1.0)

    assert chrono.ecoule() == 0.0
    assert chrono.restant() == 599.0
    assert chrono.demarrer() is False      # horloge encore à zéro

    clock.advance(1.0)
    assert chrono.demarrer() is True
    clock.advance(10.0)
    assert chrono.ecoule() == pytest.approx(10.0)
    assert chrono.restant() == pytest.approx(589.0)
    assert chrono.expire() is False

    clock.advance(589.0)
    assert chrono.expire() is True


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
    def __init__(self, handle, pret=True):
        self.handle = handle
        self.sent = []
        self.pret = pret

    def server_is_ready(self):
        return self.pret

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent.append((goal, feedback_callback))
        return _FakeFuture(True, self.handle)


class _FakeRuntimeNode:
    def __init__(self):
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger


def _navigateur_sans_ros(chronometre):
    navigateur = object.__new__(NavigateurNav2)
    navigateur._chronometre = chronometre
    navigateur._spin = lambda timeout: None
    navigateur._is_ok = lambda: True
    navigateur._interrompre = lambda: False
    return navigateur


def test_attendre_future_distingue_echeance_et_interruption():
    chrono = SimpleNamespace(restant=lambda: 10.0)
    navigateur = _navigateur_sans_ros(chrono)

    assert navigateur._attendre_future(_FakeFuture(True)) == 'ok'

    chrono.restant = lambda: -1.0
    assert navigateur._attendre_future(_FakeFuture(False)) == 'temps_ecoule'

    chrono.restant = lambda: 10.0
    assert navigateur._attendre_future(
        _FakeFuture(False), interrompre=lambda: True) == 'interrompu'


def test_le_navigateur_respecte_les_rayons_fournis_par_les_overrides_ros():
    navigation = SimpleNamespace(
        success_radius=0.2, arrival_recheck_radius=0.35)
    navigateur = object.__new__(NavigateurNav2)
    navigateur._navigation = navigation
    navigateur._success_radius = 0.3
    navigateur._arrival_recheck_radius = 0.4
    navigateur._pose_robot = lambda: Pose2D(0.25, 0.0, 0.0)

    assert navigateur.est_arrive(0.0, 0.0) is True
    assert navigateur._arrival_recheck_radius == 0.4


def test_le_navigateur_utilise_les_rayons_de_configuration_sans_override():
    navigation = SimpleNamespace(
        success_radius=0.2, arrival_recheck_radius=0.35)
    navigateur = object.__new__(NavigateurNav2)
    navigateur._navigation = navigation
    navigateur._success_radius = navigation.success_radius
    navigateur._arrival_recheck_radius = navigation.arrival_recheck_radius
    navigateur._pose_robot = lambda: Pose2D(0.25, 0.0, 0.0)

    assert navigateur.est_arrive(0.0, 0.0) is False
    assert navigateur._arrival_recheck_radius == 0.35


def _navigateur_pour_resultat(handle, pose):
    navigateur = object.__new__(NavigateurNav2)
    navigateur._client = _FakeNavClient(handle)
    navigateur._node = _FakeRuntimeNode()
    navigateur._navigation = SimpleNamespace(
        success_radius=0.2, arrival_recheck_radius=0.35,
        feedback_log_period_sec=5.0, cancel_timeout_sec=2.0)
    navigateur._success_radius = 0.2
    navigateur._arrival_recheck_radius = 0.35
    navigateur._chronometre = SimpleNamespace(
        ecoule=lambda: 1.0, restant=lambda: 10.0)
    navigateur._pose_robot = lambda: pose
    navigateur._spin = lambda timeout: None
    navigateur._is_ok = lambda: True
    navigateur._interrompre = lambda: False
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


def test_le_but_peut_imposer_un_cap():
    but = construire_but(
        1.0, 0.0, 'map', None, Pose2D(0.0, 0.0, 0.0), None, cap=1.0)
    assert but.pose.pose.orientation.z == pytest.approx(math.sin(0.5))
    assert but.pose.pose.orientation.w == pytest.approx(math.cos(0.5))


class _FakeGoalHandleEnCours(_FakeGoalHandle):
    """But accepté dont le résultat n'arrive jamais."""

    def get_result_async(self):
        return _FakeFuture(False)


def test_le_navigateur_s_interrompt_et_annule_le_but():
    handle = _FakeGoalHandleEnCours()
    navigateur = _navigateur_pour_resultat(handle, Pose2D(3.0, 0.0, 0.0))

    resultat = navigateur.naviguer_une_fois(
        0.0, 0.0, 'map', Pose2D(0, 0, 0), interrompre=lambda: True)

    assert resultat == 'interrompu'
    assert handle.cancelled is True


def test_un_arret_arbitre_annule_aussi_une_approche_nav2():
    poignee = _FakeGoalHandleEnCours()
    navigateur = _navigateur_pour_resultat(poignee, Pose2D(3.0, 0.0, 0.0))
    navigateur._interrompre = lambda: True
    assert navigateur.naviguer_une_fois(0.0, 0.0, 'map', Pose2D(0, 0, 0)) == 'interrompu'
    assert poignee.cancelled


def test_le_navigateur_transmet_le_cap_au_message():
    handle = _FakeGoalHandle(result=SimpleNamespace(status=4))
    navigateur = _navigateur_pour_resultat(handle, Pose2D(0.0, 0.0, 0.0))
    recus = []
    navigateur.message_but = lambda *args: recus.append(args) or object()

    navigateur.naviguer_une_fois(1.0, 2.0, 'map', Pose2D(0, 0, 0), cap=0.5)

    assert recus[0][-1] == 0.5


class _FakeSpinClient(_FakeNavClient):
    def __init__(self, handle, pret=True):
        super().__init__(handle)
        self.pret = pret

    def server_is_ready(self):
        return self.pret


def _navigateur_rotation(handle, pret=True):
    navigateur = _navigateur_pour_resultat(handle, Pose2D(0, 0, 0))
    navigateur._client_rotation = _FakeSpinClient(handle, pret)
    return navigateur


def test_la_rotation_nav2_reussit_ou_echoue_selon_le_statut():
    reussie = _navigateur_rotation(_FakeGoalHandle(result=SimpleNamespace(status=4)))
    assert reussie.tourner(6.28, 20.0) == 'succes'
    goal = reussie._client_rotation.sent[0][0]
    assert goal.target_yaw == pytest.approx(6.28)
    assert goal.time_allowance.sec == 20

    echouee = _navigateur_rotation(_FakeGoalHandle(result=SimpleNamespace(status=6)))
    assert echouee.tourner(6.28, 20.0) == 'echec'


def test_la_rotation_s_interrompt_et_signale_un_serveur_absent():
    handle = _FakeGoalHandleEnCours()
    navigateur = _navigateur_rotation(handle)
    assert navigateur.tourner(6.28, 20.0, interrompre=lambda: True) == 'interrompu'
    assert handle.cancelled is True

    absent = _navigateur_rotation(_FakeGoalHandle(), pret=False)
    assert absent.tourner(6.28, 20.0) == 'indisponible'


def test_sans_serveur_nav2_le_but_n_est_pas_envoye():
    navigateur = _navigateur_pour_resultat(_FakeGoalHandle(), Pose2D(0, 0, 0))
    navigateur._client.pret = False
    assert navigateur.naviguer_une_fois(1.0, 0.0, 'map', Pose2D(0, 0, 0)) == 'refuse'
    assert navigateur._client.sent == []


class _FakeEnvoiTardif(_FakeNavClient):
    """L'acceptation arrive après l'interruption (revue de code)."""

    def __init__(self, handle):
        super().__init__(handle)
        self.future = _FakeFuture(False, handle)

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent.append((goal, feedback_callback))
        return self.future


def test_un_but_accepte_apres_l_interruption_est_annule():
    handle = _FakeGoalHandleEnCours()
    navigateur = _navigateur_pour_resultat(handle, Pose2D(3.0, 0.0, 0.0))
    client = _FakeEnvoiTardif(handle)
    navigateur._client = client

    def accepter_au_premier_tour(timeout):
        client.future._done = True

    navigateur._spin = accepter_au_premier_tour
    resultat = navigateur.naviguer_une_fois(
        0.0, 0.0, 'map', Pose2D(0, 0, 0), interrompre=lambda: True)
    assert resultat == 'interrompu'
    assert handle.cancelled is True
