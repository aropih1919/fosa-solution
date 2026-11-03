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
        "state_topic": "/etat",
        "status": {
            "start_topic": "/s/depart", "visit_topic": "/s/tag",
            "image_topic": "/s/image", "finish_topic": "/s/fin"},
        "time_limit_sec": 600.0, "stop_margin_sec": 5.0,
        "nav2_ready_timeout_sec": 60.0,
        "visit_distance_m": 0.5, "approach_distance_m": 0.9,
        "distance_tolerance_m": 0.04, "lateral_tolerance_m": 0.06,
        "heading_tolerance_deg": 5.0, "stable_cycles": 5, "hold_sec": 5.0,
        "min_tag_budget_sec": 45.0, "detection_max_age_sec": 0.6,
        "search_spin_time_sec": 25.0, "search_grid_step_m": 0.8,
        "search_clearance_m": 0.45, "search_visited_radius_m": 0.8,
        "front_guard_m": 0.10,
        "alignment": {
            "linear_gain": 0.6, "angular_gain": 1.2, "max_linear_mps": 0.08,
            "max_angular_rps": 0.35, "min_linear_mps": 0.03,
            "min_angular_rps": 0.1, "timeout_sec": 20.0,
            "max_renavigations": 2},
        "slam_map_file": "", "memorized_tags_file": "",
        "tags_output_file": "~/.ros/fosa_tags_course.yaml",
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
    assert course.status.visit_topic


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
            "model_path": "~/modele", "min_confidence": 0.8})


def test_une_source_d_obstacles_incoherente_est_refusee():
    with pytest.raises(ConfigurationError, match="raytrace_max_range"):
        ObstacleSourceConfig.from_mapping({
            "name": "lidar", "topic": "/scan", "enabled": True,
            "obstacle_min_range": 0.1, "obstacle_max_range": 4.0,
            "raytrace_max_range": 3.0, "global_costmap": True})
