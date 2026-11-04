"""Tests de la configuration de course (phase réelle) et des profils."""

from pathlib import Path

import pytest

from caytu_config import ConfigurationError, load_config
from caytu_config.modeles_course import (
    CourseConfig,
    ObstacleSourceConfig,
    StartWordConfig,
)

CONFIG = Path(__file__).resolve().parents[1] / "config"
PROFIL_SIMULATION = CONFIG / "profils" / "simulation.yaml"


@pytest.fixture
def robot(monkeypatch):
    monkeypatch.delenv("FOSA_CONFIG_OVERRIDE", raising=False)
    monkeypatch.delenv("FOSA_DEPLOYMENT", raising=False)
    return load_config(CONFIG)


def _course_minimale(**surcharges):
    base = {
        "tag_ids": [1, 4, 7, 9],
        "order_topic": "/ordre",
        "order_type": "string",
        "start_topic": "/depart",
        "state_topic": "/etat", "events_topic": "/evenements", "map_topic": "/map",
        "referee": {
            "start_topic": "/parc/start", "start_type": "parc_msgs/msg/StartSignal",
            "start_method_command": 0, "start_method_voice": 1,
            "listen_start": True,
            "visit_topic": "/parc/tag_visit", "visit_type": "parc_msgs/msg/TagVisit",
            "finish_topic": "/parc/finish", "finish_type": "parc_msgs/msg/Finish",
            "finish_reason": 0, "run_status_topic": "/parc/run_status",
            "jpeg_quality": 85},
        "time_limit_sec": 600.0, "stop_margin_sec": 5.0,
        "nav2_ready_timeout_sec": 60.0,
        "visit_distance_m": 0.5, "approach_distance_m": 0.9,
        "distance_tolerance_m": 0.04, "lateral_tolerance_m": 0.06,
        "heading_tolerance_deg": 5.0, "stable_cycles": 5, "hold_sec": 5.0,
        "detection_max_age_sec": 0.6,
        "search_spin_time_sec": 25.0, "search_grid_step_m": 0.8,
        "search_clearance_m": 0.45, "search_visited_radius_m": 0.8,
        "front_guard_m": 0.10, "goal_xy_tolerance_m": 0.08,
        "goal_yaw_tolerance_deg": 7.0, "tag_lost_timeout_sec": 4.0,
        "preapproach_margin_m": 0.3, "search_spin_speed_rps": 0.4,
        "search_max_radius_m": 5.0, "unstuck_distance_m": 0.3,
        "unstuck_after_failures": 2,
        "alignment": {
            "linear_gain": 0.6, "angular_gain": 1.2, "max_linear_mps": 0.08,
            "max_angular_rps": 0.35, "min_linear_mps": 0.03,
            "min_angular_rps": 0.1, "precision_point_m": 0.025,
            "precision_heading_deg": 2.0, "max_distance_error_m": 0.75,
            "max_lateral_error_m": 0.30, "timeout_sec": 30.0,
            "max_renavigations": 2},
        "strategy": {
            "points_per_tag": 25.0, "travel_speed_mps": 0.15,
            "detour_factor": 1.4, "visit_time_sec": 45.0,
            "search_time_sec": 90.0, "unknown_leg_m": 2.5, "time_margin": 0.2,
            "success_known": 0.9, "success_unknown": 0.6, "failure_decay": 0.6,
            "min_skip_gain_points": 5.0, "replan_period_sec": 2.0},
        "preparation": {
            "directory": "~/.ros/fosa/preparation", "max_duration_sec": 720.0,
            "close_view_m": 1.3, "approach_attempts": 2},
    }
    base.update(surcharges)
    return base


def test_le_profil_par_defaut_est_le_robot_reel(robot):
    assert robot.system.deployment == "robot"
    assert robot.system.use_sim_time is False
    assert robot.system.base_frame == "base_link"
    assert robot.system.ros_domain_id == 0
    assert robot.navigation.odom_topic == "/botwheel_explorer/odom"
    assert robot.navigation.cmd_vel_topic == "/cmd_vel"
    assert robot.navigation.scan_topic == "/scan_filtered"
    assert robot.navigation.laser_self_filter is False


def test_les_tags_officiels_sont_configures(robot):
    tag = robot.perception.apriltag
    assert tag.family == "36h11"
    assert tag.size_m == pytest.approx(0.165)
    assert tag.ids == (1, 4, 7, 9)
    assert [camera.name for camera in tag.cameras] == ["camera1", "camera2"]
    assert tag.cameras[0].color_topic == "/camera1/camera1/color/image_raw"


def test_la_course_suit_le_reglement(robot):
    course = robot.course
    assert course.tag_ids == (1, 4, 7, 9)
    assert course.visit_distance_m == pytest.approx(0.5)
    assert course.time_limit_sec == pytest.approx(600.0)
    assert course.heading_tolerance_deg < 12.0


def test_l_arbitre_officiel_est_configure(robot):
    arbitre = robot.course.referee
    assert robot.course.order_topic == "/parc/tag_order"
    assert robot.course.order_type == "auto"
    assert arbitre.start_topic == "/parc/start"
    assert arbitre.visit_topic == "/parc/tag_visit"
    assert arbitre.finish_topic == "/parc/finish"
    assert (arbitre.start_method_command, arbitre.start_method_voice) == (0, 1)


def test_la_pre_approche_reste_a_portee_de_l_asservissement():
    base = _course_minimale()
    base["alignment"] = dict(base["alignment"], max_distance_error_m=0.5)
    with pytest.raises(ConfigurationError, match="max_distance_error_m"):
        CourseConfig.from_mapping(base)


def test_des_methodes_de_depart_identiques_sont_refusees():
    base = _course_minimale()
    base["referee"] = dict(base["referee"], start_method_voice=0)
    with pytest.raises(ConfigurationError, match="méthodes de départ"):
        CourseConfig.from_mapping(base)


def test_le_mot_de_depart_est_hors_ligne(robot):
    voix = robot.start_word
    assert voix.words == ("allez",)
    assert voix.audio_topic == "/speech_audio"
    assert voix.sample_width == 2


def test_le_profil_simulation_surcharge_les_interfaces(monkeypatch):
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(PROFIL_SIMULATION))
    simulation = load_config(CONFIG)
    assert simulation.system.deployment == "simulation"
    assert simulation.system.use_sim_time is True
    assert simulation.system.base_frame == "base_footprint"
    assert simulation.navigation.odom_topic == "/odom"
    assert simulation.navigation.laser_self_filter is True
    assert simulation.course.tag_ids == (1, 4, 7, 9)


def test_des_tags_en_double_sont_refuses():
    with pytest.raises(ConfigurationError, match="tag_ids"):
        CourseConfig.from_mapping(_course_minimale(tag_ids=[1, 1, 7, 9]))


def test_un_type_d_ordre_inconnu_est_refuse():
    with pytest.raises(ConfigurationError, match="order_type"):
        CourseConfig.from_mapping(_course_minimale(order_type="xml"))


def test_une_tolerance_de_cap_hors_reglement_est_refusee():
    with pytest.raises(ConfigurationError, match="heading_tolerance_deg"):
        CourseConfig.from_mapping(_course_minimale(heading_tolerance_deg=15.0))


def test_le_mot_de_depart_exige_au_moins_un_mot():
    with pytest.raises(ConfigurationError, match="words"):
        StartWordConfig.from_mapping({
            "enabled": True, "audio_topic": "/speech_audio",
            "mode": "utterance", "sample_rate": 16000, "channels": 1,
            "channel_index": 0, "sample_width": 2, "words": [],
            "restricted_grammar": False,
            "model_path": "~/modele", "min_confidence": 0.8})


def test_une_source_d_obstacles_incoherente_est_refusee():
    with pytest.raises(ConfigurationError, match="raytrace_max_range"):
        ObstacleSourceConfig.from_mapping({
            "name": "lidar", "topic": "/scan", "enabled": True,
            "obstacle_min_range": 0.1, "obstacle_max_range": 4.0,
            "raytrace_max_range": 3.0, "inf_is_valid": False,
            "global_costmap": True})


def test_les_tags_de_la_course_doivent_etre_detectes(tmp_path, monkeypatch):
    surcharge = tmp_path / "surcharge.yaml"
    surcharge.write_text("perception:\n  apriltag:\n    ids: [1, 4, 7]\n",
                         encoding="utf-8")
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(surcharge))
    with pytest.raises(ConfigurationError, match="tag_ids"):
        load_config(CONFIG)


def test_une_source_camera_ne_marque_pas_au_bout_du_scan(tmp_path, monkeypatch):
    surcharge = tmp_path / "surcharge.yaml"
    surcharge.write_text(
        "navigation:\n  obstacle_sources:\n    - name: camera1\n"
        "      topic: /fosa/scan_camera1\n      enabled: true\n"
        "      obstacle_min_range: 0.25\n      obstacle_max_range: 2.5\n"
        "      raytrace_max_range: 3.0\n      inf_is_valid: true\n"
        "      global_costmap: true\n", encoding="utf-8")
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(surcharge))
    with pytest.raises(ConfigurationError, match="obstacle_max_range"):
        load_config(CONFIG)


def test_les_cameras_detectent_les_obstacles_hauts(robot):
    noms = [scan.name for scan in robot.perception.depth_scans]
    assert noms == ["camera1", "camera2"]
    actives = {s.topic for s in robot.navigation.obstacle_sources if s.enabled}
    assert {s.output_topic for s in robot.perception.depth_scans} <= actives
