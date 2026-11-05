"""Tests des calculs purs : s'exécutent avec `pytest`, sans ROS ni Gazebo."""
import math
import random

import pytest

from caytu_nav_solution.nav_math import (
    GRAVITE,
    FusionOdomImu,
    Pose2D,
    accord_scan_carte,
    charger_carte_pgm,
    distances_sol,
    hauteur_lidar,
    inclinaison_depuis_acceleration,
    lacet_depuis_quaternion,
    normaliser_angle,
    quaternion_depuis_lacet,
    quaternion_depuis_roulis_tangage_lacet,
    separer_retours_sol,
)

DEPART = Pose2D(-0.200546, -7.485170, 1.571)        # task_params.yaml officiel
BUT = (-2.293350, 2.232090)


def test_pose_composer_et_inverse_coherents():
    a = Pose2D(1.0, -2.0, 0.7)
    b = Pose2D(0.3, 0.4, -1.2)
    identite = a.composer(a.inverse())
    assert identite.x == pytest.approx(0.0, abs=1e-12)
    assert identite.y == pytest.approx(0.0, abs=1e-12)
    assert identite.yaw == pytest.approx(0.0, abs=1e-12)
    retour = a.inverse().composer(a.composer(b))
    assert (retour.x, retour.y, retour.yaw) == pytest.approx((b.x, b.y, b.yaw))


def test_quaternions_aller_retour():
    for lacet in (-3.0, -1.0, 0.0, 0.5, 3.1):
        assert lacet_depuis_quaternion(
            *quaternion_depuis_lacet(lacet)) == pytest.approx(lacet)
    # Le lacet reste lisible quand le robot penche (cas réel : 2,5° de tangage).
    assert lacet_depuis_quaternion(
        *quaternion_depuis_roulis_tangage_lacet(0.01, 0.043, 1.2)
    ) == pytest.approx(1.2)


def test_premiere_mise_a_jour_place_map_sur_repere_gazebo():
    fusion = FusionOdomImu(DEPART)
    map_vers_odom = fusion.mettre_a_jour(Pose2D(0.0, 0.0, 0.0), lacet_imu=0.0)
    # L'origine de odom est le départ : map devient le repère Gazebo.
    assert (map_vers_odom.x, map_vers_odom.y, map_vers_odom.yaw) == pytest.approx(
        (DEPART.x, DEPART.y, DEPART.yaw))
    pose = fusion.get_pose()
    assert (pose.x, pose.y) == pytest.approx((DEPART.x, DEPART.y))


def _simuler_deplacement(fusion, segments, gain_cap, utiliser_imu,
                         bruit=0.0, graine=1):
    """Simule un robot réel et son odométrie dont le cap est multiplié par gain_cap.

    segments : liste de (distance, rotation) parcourus par petits pas.
    Retourne (pose vraie dans map, pose estimée).
    """
    generateur = random.Random(graine)
    vraie_pose = DEPART                # pose réelle dans Gazebo
    odom = Pose2D()                    # ce que publie le plugin DiffDrive
    lacet_vrai_relatif = 0.0           # ce que mesure l'IMU (relatif au départ)
    fusion.mettre_a_jour(odom, 0.0 if utiliser_imu else None)

    for distance, rotation in segments:
        nombre_pas = 200
        for _ in range(nombre_pas):
            ds = distance / nombre_pas
            dth = rotation / nombre_pas

            vraie_pose = Pose2D(
                vraie_pose.x + ds * math.cos(vraie_pose.yaw + dth / 2),
                vraie_pose.y + ds * math.sin(vraie_pose.yaw + dth / 2),
                normaliser_angle(vraie_pose.yaw + dth))
            lacet_vrai_relatif += dth

            dth_odom = dth * gain_cap
            odom = Pose2D(
                odom.x + ds * math.cos(odom.yaw + dth_odom / 2),
                odom.y + ds * math.sin(odom.yaw + dth_odom / 2),
                normaliser_angle(odom.yaw + dth_odom))

            imu = (normaliser_angle(lacet_vrai_relatif + generateur.gauss(0.0, bruit))
                   if utiliser_imu else None)
            map_vers_odom = fusion.mettre_a_jour(odom, imu)

            # La TF publiée doit toujours redonner la pose estimée.
            via_tf = map_vers_odom.composer(odom)
            pose = fusion.get_pose()
            assert (via_tf.x, via_tf.y) == pytest.approx(
                (pose.x, pose.y), abs=1e-9)

    return vraie_pose, fusion.get_pose()


SLALOM = [(2.0, 0.0), (0.0, 0.8), (1.5, 0.0), (0.0, -1.6), (2.0, 0.0),
          (0.0, 0.8), (2.5, 0.0), (1.0, 0.6), (1.5, -0.6)]


def test_cap_imu_corrige_erreur_rotation_odometrie():
    # Entraxe déclaré 0.3409 m pour 0.393 m réels : cap odométrique x1,153.
    gain = 0.39298 / 0.3409
    vraie, estimee = _simuler_deplacement(
        FusionOdomImu(DEPART), SLALOM, gain, utiliser_imu=True, bruit=2e-4)
    assert math.hypot(estimee.x - vraie.x, estimee.y - vraie.y) < 0.02
    assert abs(normaliser_angle(estimee.yaw - vraie.yaw)) < 0.005


def test_cap_odometrie_seul_rate_le_but():
    gain = 0.39298 / 0.3409
    vraie, estimee = _simuler_deplacement(
        FusionOdomImu(DEPART), SLALOM, gain, utiliser_imu=False)
    # Sans IMU, l'erreur dépasse la tolérance d'arrivée : d'où le choix de l'IMU.
    assert math.hypot(estimee.x - vraie.x, estimee.y - vraie.y) > 0.25


def test_fusion_bascule_puis_revient_sans_saut():
    fusion = FusionOdomImu(DEPART)
    fusion.mettre_a_jour(Pose2D(0.0, 0.0, 0.0), 0.0)
    fusion.mettre_a_jour(Pose2D(0.5, 0.0, 0.0), 0.0)
    assert fusion.utilise_imu()
    avant = fusion.get_pose()
    # IMU muette : cap odométrique.
    fusion.mettre_a_jour(Pose2D(0.5, 0.0, 0.1), None)
    assert not fusion.utilise_imu()
    assert fusion.get_pose().yaw == pytest.approx(
        normaliser_angle(avant.yaw + 0.1))
    # IMU de retour, autre référence : on se recale sans saut.
    fusion.mettre_a_jour(Pose2D(0.5, 0.0, 0.1), 2.5)
    assert fusion.utilise_imu()
    assert fusion.get_pose().yaw == pytest.approx(
        normaliser_angle(avant.yaw + 0.1))


def test_saut_odometrie_non_integre():
    fusion = FusionOdomImu(DEPART)
    fusion.mettre_a_jour(Pose2D(0.0, 0.0, 0.0), 0.0)
    # Réinitialisation du simulateur : ce n'est pas un déplacement.
    fusion.mettre_a_jour(Pose2D(5.0, 5.0, 0.0), 0.0)
    pose = fusion.get_pose()
    assert (pose.x, pose.y) == pytest.approx((DEPART.x, DEPART.y))


def test_inclinaison_accelerometre_correspond_geometrie():
    tangage = math.atan(0.0098 / 0.228)          # roulette 9,8 mm plus basse
    ax = -GRAVITE * math.sin(tangage)
    ay = 0.0
    az = GRAVITE * math.cos(tangage)
    roulis, mesure = inclinaison_depuis_acceleration(ax, ay, az)
    assert mesure == pytest.approx(tangage)
    assert roulis == pytest.approx(0.0)
    assert math.degrees(tangage) == pytest.approx(2.46, abs=0.01)


def test_ligne_sol_droite_devant():
    hauteur = hauteur_lidar(math.radians(3.5), 0.0481, 0.116)
    tangage_plan = math.radians(3.5) - 0.04363323129985824      # 1° vers le bas
    angles = [math.radians(a) for a in (-40, -20, 0, 20, 40, 120, 180)]
    sol = distances_sol(angles, tangage_plan, 0.0, hauteur)
    avant = [r * math.cos(a) for r, a in zip(sol[:5], angles[:5])]
    # À l'avant, le sol est une ligne droite perpendiculaire au cap...
    assert max(avant) - min(avant) < 1e-9
    assert avant[0] == pytest.approx(hauteur / math.sin(tangage_plan))
    # ...et les rayons arrière, qui montent, ne touchent jamais le sol.
    assert sol[5] == math.inf and sol[6] == math.inf


def test_lidar_horizontal_ne_voit_jamais_le_sol():
    sol = distances_sol([0.0, 1.0, 2.0, 3.0], 0.0, 0.0, 0.05)
    assert all(r == math.inf for r in sol)


def test_retours_sol_retires_obstacles_gardes():
    sol = [3.0, 3.0, 3.0, math.inf]
    distances = [3.02, 1.20, math.inf, 5.0]      # sol, obstacle, vide, mur arrière
    marquage, effacement, nombre = separer_retours_sol(
        distances, sol, portee_max=12.0)
    assert nombre == 1
    assert marquage == [math.inf, 1.20, math.inf, 5.0]
    assert effacement[0] == pytest.approx(2.1)   # libre jusqu'à 70 % du sol
    assert effacement[1:] == [1.20, math.inf, 5.0]


def _pgm_salle_carree(taille=40):
    """Salle carrée de 2 m (résolution 0.05) entourée d'un mur d'une cellule."""
    lignes = []
    for ligne in range(taille):
        lignes.append(bytes(
            0 if ligne in (0, taille - 1) or colonne in (0, taille - 1) else 254
            for colonne in range(taille)))
    return f'P5\n# test\n{taille} {taille}\n255\n'.encode() + b''.join(lignes)


def test_chargement_pgm_et_accord_scan():
    carte = charger_carte_pgm(_pgm_salle_carree(), 0.05, -1.0, -1.0)
    assert carte.largeur == 40 and carte.hauteur == 40
    assert carte.est_occupee_pres(-0.99, 0.0, 0)        # mur ouest
    assert not carte.est_occupee_pres(0.0, 0.0, 2)      # centre libre

    nombre_rayons = 180
    angle_min = -math.pi
    increment = 2 * math.pi / nombre_rayons

    def distance_mur(angle):
        return min(0.975 / max(abs(math.cos(angle)), 1e-9),
                   0.975 / max(abs(math.sin(angle)), 1e-9))

    distances = [distance_mur(angle_min + i * increment)
                 for i in range(nombre_rayons)]
    accord, utilises = accord_scan_carte(
        carte, Pose2D(0.0, 0.0, 0.0), angle_min, increment, distances)
    assert utilises == nombre_rayons and accord > 0.95

    # Même scan, mais robot supposé ailleurs : la carte ne colle plus.
    mauvais, _ = accord_scan_carte(
        carte, Pose2D(0.5, 0.4, 0.6), angle_min, increment, distances)
    assert mauvais < 0.5


def test_pgm_refuse_autres_formats():
    with pytest.raises(ValueError):
        charger_carte_pgm(b'P2\n2 2\n255\n0 0 0 0\n', 0.05, 0.0, 0.0)
