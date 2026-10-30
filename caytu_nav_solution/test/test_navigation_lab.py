"""Tests unitaires du chargeur, des métriques et des rapports du laboratoire."""

import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from caytu_nav_solution.benchmark_config import (
    AMCL_EKF_UNAVAILABLE,
    CONTROLLERS,
    PROFILES,
    ConfigurationError,
    build_experiment_config,
    deep_merge,
    validate_localization_map,
    validate_scan_map_agreement,
    validate_experiment_config,
    validate_selection,
    write_experiment_config,
)
from caytu_nav_solution.benchmark_metrics import (
    BenchmarkMetrics,
    CollisionEpisodeCounter,
    distance_between,
    pose_errors,
    travelled_distance,
)
from caytu_nav_solution.benchmark_report import (
    new_run_directory,
    write_report,
)
from caytu_nav_solution.compare_cli import load_runs, print_comparison
from caytu_nav_solution.nav_math import Pose2D


SOLUTION_DIR = Path(__file__).resolve().parents[1]
ROOT = SOLUTION_DIR.parent
CONFIG = ROOT / "caytu_nav_bringup" / "config"
MAP = ROOT / "caytu_nav_bringup" / "maps" / "cafe_map.yaml"
SPAWN = Pose2D(-0.200546, -7.485170, 1.571)
GOAL = Pose2D(-2.293350, 2.232090, 0.0)


@pytest.mark.parametrize("controller", CONTROLLERS)
@pytest.mark.parametrize("profile", PROFILES)
def test_every_controller_profile_and_odom_mode_loads(controller, profile):
    params = build_experiment_config(
        controller, "odom_imu", profile, SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert params["controller_server"]["ros__parameters"]["controller_plugins"] == ["FollowPath"]
    assert follow_path["plugin"]
    assert params["odom_imu_localizer"]["ros__parameters"]["use_imu_yaw"] is True


def test_balanced_profiles_retain_reference_values_for_existing_controllers():
    for controller, expected_file in (("rpp", "controller_rpp.yaml"),
                                      ("dwb", "controller_dwb.yaml")):
        config = build_experiment_config(
            controller, "odom_imu", "balanced", SPAWN,
            config_root=CONFIG, check_packages=False,
        )
        baseline = yaml.safe_load((CONFIG / "nav2_params.yaml").read_text())
        override = yaml.safe_load((CONFIG / expected_file).read_text())
        merged = deep_merge(baseline, override)
        assert config["controller_server"]["ros__parameters"]["FollowPath"] == \
            merged["controller_server"]["ros__parameters"]["FollowPath"]


def test_odom_only_reuses_localizer_without_imu_yaw():
    config = build_experiment_config(
        "rpp", "odom_only", "balanced", SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    params = config["odom_imu_localizer"]["ros__parameters"]
    assert params["use_imu_yaw"] is False
    assert params["odom_topic"] == "/odom"
    assert params["imu_topic"] == "/imu"


def test_amcl_config_gets_initial_pose_and_filtered_lidar_topic():
    config = build_experiment_config(
        "rpp", "amcl", "balanced", SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    params = config["amcl"]["ros__parameters"]
    assert params["scan_topic"] == "/scan_clean"
    assert params["set_initial_pose"] is True
    assert params["initial_pose"] == {
        "x": SPAWN.x, "y": SPAWN.y, "z": 0.0, "yaw": SPAWN.yaw,
    }
    assert "odom_imu_localizer" not in config


def test_amcl_ekf_is_explicitly_unavailable_until_tf_owner_is_verified():
    with pytest.raises(ConfigurationError, match="amcl_ekf est indisponible"):
        validate_selection("rpp", "amcl_ekf", "balanced", check_packages=False)
    assert "publisher" in AMCL_EKF_UNAVAILABLE


def test_profiles_keep_footprint_sensor_layers_and_collision_detection():
    for controller in CONTROLLERS:
        for profile in PROFILES:
            config = build_experiment_config(
                controller, "odom_imu", profile, SPAWN,
                config_root=CONFIG, check_packages=False,
            )
            for name in ("global_costmap", "local_costmap"):
                params = config[name][name]["ros__parameters"]
                assert params["footprint"]
                assert params["footprint_padding"] >= 0.02
                assert "inflation_layer" in params["plugins"]
                assert any("lidar_layer" in layer or "camera_layer" in layer
                           for layer in params["plugins"])
            if controller == "rpp":
                assert config["controller_server"]["ros__parameters"]["FollowPath"]\
                    ["use_collision_detection"] is True


def test_tight_clearance_is_a_simulation_profile_not_a_disabled_safety_mode():
    config = build_experiment_config(
        "rpp", "odom_imu", "tight_clearance", SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    follow_path = config["controller_server"]["ros__parameters"]["FollowPath"]
    assert local["inflation_layer"]["inflation_radius"] == pytest.approx(0.30)
    assert local["footprint_padding"] == pytest.approx(0.02)
    assert follow_path["use_collision_detection"] is True
    with pytest.raises(ConfigurationError, match="robot physique est désactivé"):
        validate_selection(
            "rpp", "odom_imu", "tight_clearance", environment="real")


def test_profile_speed_order_is_safe_balanced_fast():
    speeds = {}
    for profile in PROFILES:
        config = build_experiment_config(
            "rpp", "odom_imu", profile, SPAWN,
            config_root=CONFIG, check_packages=False,
        )
        speeds[profile] = config["controller_server"]["ros__parameters"]\
            ["FollowPath"]["desired_linear_vel"]
    assert speeds["safe"] < speeds["balanced"] < speeds["fast"]


def test_controller_specific_parameter_dependencies_are_validated():
    rpp = build_experiment_config(
        "rpp", "odom_imu", "balanced", SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    rpp["controller_server"]["ros__parameters"]["FollowPath"][
        "inflation_cost_scaling_factor"] = 9.0
    with pytest.raises(ConfigurationError, match="facteur d'inflation"):
        validate_experiment_config(rpp, "rpp", "odom_imu", "balanced")

    dwb = build_experiment_config(
        "dwb", "odom_imu", "balanced", SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    dwb["controller_server"]["ros__parameters"]["FollowPath"]["vx_samples"] = 1
    with pytest.raises(ConfigurationError, match="vx_samples"):
        validate_experiment_config(dwb, "dwb", "odom_imu", "balanced")


def test_rejects_incompatible_or_unknown_selection():
    with pytest.raises(ConfigurationError, match="Scénario indisponible"):
        validate_selection("rpp", "odom_imu", "balanced", scenario="dynamic_obstacles")
    with pytest.raises(ConfigurationError, match="robot physique est désactivé"):
        validate_selection("rpp", "odom_imu", "safe", environment="real")
    with pytest.raises(ConfigurationError, match="Profil inconnu"):
        validate_selection("rpp", "odom_imu", "turbo")


def test_yaml_files_parse_and_amcl_map_contains_spawn_goal_with_clearance():
    for path in CONFIG.rglob("*.yaml"):
        assert yaml.safe_load(path.read_text(encoding="utf-8")) is not None, path
    assert validate_localization_map(MAP, SPAWN, GOAL) == str(MAP.resolve())


def test_amcl_map_validation_rejects_missing_and_out_of_map_poses(tmp_path):
    with pytest.raises(ConfigurationError, match="pas libre|hors de la carte|sort de la carte"):
        validate_localization_map(MAP, Pose2D(100.0, 100.0, 0.0), GOAL)
    with pytest.raises(FileNotFoundError):
        validate_localization_map(tmp_path / "missing.yaml", SPAWN, GOAL)


@pytest.mark.parametrize("agreement,used", [(0.0, 200), (0.8, 29)])
def test_amcl_rejects_unmatched_or_insufficient_initial_scan(agreement, used):
    with pytest.raises(ConfigurationError, match="AMCL ne démarre pas"):
        validate_scan_map_agreement(agreement, used, 0.35)
    validate_scan_map_agreement(0.5, 30, 0.35)


def test_config_export_is_complete_yaml(tmp_path):
    config = build_experiment_config(
        "mppi", "odom_imu", "balanced", SPAWN,
        config_root=CONFIG, check_packages=False,
    )
    path = write_experiment_config(config, tmp_path / "run" / "config_used.yaml")
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert loaded["controller_server"]["ros__parameters"]["FollowPath"]\
        ["plugin"] == "nav2_mppi_controller::MPPIController"
    assert loaded["local_costmap"]["local_costmap"]["ros__parameters"]\
        ["footprint_padding"] == 0.02


def test_map_and_localizer_launch_have_one_map_to_odom_producer():
    solution_launch = (ROOT / "caytu_nav_bringup" / "launch" /
                       "solution_bringup.launch.py").read_text(encoding="utf-8")
    nav_launch = (ROOT / "caytu_nav_bringup" / "launch" /
                  "navigation.launch.py").read_text(encoding="utf-8")
    assert 'if localization == "amcl":' in solution_launch
    assert 'return []' in solution_launch
    assert solution_launch.count('executable="odom_imu_localizer"') == 1
    assert 'lifecycle_nodes.insert(1, "amcl")' in nav_launch
    assert 'package="nav2_amcl"' in nav_launch
    assert '"amcl_ekf"' not in solution_launch + nav_launch


def test_distance_and_pose_error_calculations():
    assert distance_between(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)
    position, angle = pose_errors((0, 0, math.pi - 0.1), (0.3, 0.4, -math.pi + 0.1))
    assert position == pytest.approx(0.5)
    assert angle == pytest.approx(0.2)


def test_distance_is_unavailable_until_a_pose_sample_arrives():
    assert travelled_distance(False, False, 0.0, 0.0) == (None, None)
    assert travelled_distance(False, True, 4.2, 1.7) == (1.7, "/odom")
    assert travelled_distance(True, True, 3.4, 1.7) == (3.4, "gazebo_truth_pose")


def test_collision_counter_groups_repeated_contacts_and_ignores_ground():
    def contact(a, b):
        return SimpleNamespace(
            collision1=SimpleNamespace(name=a),
            collision2=SimpleNamespace(name=b),
        )

    counter = CollisionEpisodeCounter(episode_gap_sec=0.5)
    collision = contact("robot::bumper", "world::table")
    ground = contact("robot::wheel", "world::ground")
    counter.update("/base_collisions", [collision, ground], now=1.0)
    counter.update("/base_collisions", [collision], now=1.2)
    counter.update("/base_collisions", [collision], now=2.0)
    assert counter.episodes == 2
    assert counter.message_count == 3


def test_run_ids_json_csv_and_comparison_filters(tmp_path):
    run_id, run_dir = new_run_directory(tmp_path, year=2026)
    assert run_id == "FOSA-2026-001"
    metadata = {
        "run_id": run_id, "controller": "rpp", "localization": "odom_imu",
        "profile": "balanced", "scenario": "full_course",
    }
    config = write_experiment_config({"controller_server": {}}, run_dir / "config_used.yaml")
    metrics = {
        "mission_success": True, "collision_count": 0,
        "execution_time_sec": 42.0, "travelled_distance_m": 10.0,
    }
    write_report(run_dir, metadata, metrics, [{"time_sec": 0.0, "truth_x": 1.0}])
    assert json.loads((run_dir / "summary.json").read_text())["metrics"]["collision_count"] == 0
    assert list(csv.DictReader((run_dir / "metrics.csv").open()))[0]["truth_x"] == "1.0"
    assert config.is_file()
    assert len(load_runs(tmp_path, controller="rpp")) == 1
    assert load_runs(tmp_path, controller="dwb") == []
    assert new_run_directory(tmp_path, year=2026)[0] == "FOSA-2026-002"


def test_empty_comparison_has_no_runs(capsys):
    assert load_runs("/path/that/does/not/exist") == []
    print_comparison([])
    assert "Aucune expérience" in capsys.readouterr().out


def test_gitignore_excludes_generated_results():
    assert "results/" in (ROOT / ".gitignore").read_text(encoding="utf-8")


@pytest.mark.parametrize(
    "ros_args, expected",
    [([], True), (["--ros-args", "-p", "use_sim_time:=false"], False)],
)
def test_benchmark_defaults_to_sim_time_and_respects_override(ros_args, expected):
    import rclpy
    from caytu_nav_solution.benchmark_cli import BenchmarkRunner

    rclpy.init(args=ros_args)
    node = BenchmarkRunner()
    try:
        assert node.get_parameter("use_sim_time").value is expected
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_live_monitor_exposes_goal_path_motion_and_localization_state():
    import rclpy

    rclpy.init(args=[])
    metrics = BenchmarkMetrics("odom_imu", GOAL.x, GOAL.y)
    try:
        metrics.odom_pose = (SPAWN.x, SPAWN.y, SPAWN.yaw)
        metrics._latest_linear_speed = 0.2
        metrics._latest_angular_speed = 0.1
        metrics._latest_command_linear = 0.25
        metrics._latest_command_angular = 0.0
        metrics.nav_goal_state = "navigation active"
        metrics._last_local_plan_stamp = 1.0
        metrics._local_plan_pose_count = 12
        metrics._global_plan_stamp = 1.0
        metrics._global_plan_pose_count = 42
        line = metrics.live_status("navigation")
        assert line.startswith("[MONITOR navigation ")
        assert "\n" not in line
        assert "navigation active" in line
        assert "path=/plan:reçu (42 poses)" in line
        assert "local:reçu (12 poses)" in line
        assert "odom v/w=0.20/+0.10m/s,rad/s" in line
        assert "loc=attente" in line
        assert "odom=(" in line and "but=N/A" in line
        metrics.estimate = (1.0, SPAWN.x, SPAWN.y, SPAWN.yaw)
        metrics.localization_samples = 1
        localized_line = metrics.live_status("navigation")
        assert "map=(" in localized_line and "but=" in localized_line
    finally:
        metrics.destroy_node()
        rclpy.shutdown()
