from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / "config"


def load_yaml(name):
    with (CONFIG_DIR / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_all_navigation_yaml_files_parse():
    for path in CONFIG_DIR.glob("*.yaml"):
        with path.open(encoding="utf-8") as stream:
            assert yaml.safe_load(stream) is not None, path.name


def test_costmap_node_paths_and_integer_window_size():
    global_params = load_yaml("global_costmap_params.yaml")
    local_params = load_yaml("local_costmap_params.yaml")

    global_costmap = global_params["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    local_costmap = local_params["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]

    assert global_costmap["plugins"] == [
        "static_layer",
        "obstacle_layer",
        "inflation_layer",
    ]
    assert local_costmap["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert type(local_costmap["width"]) is int
    assert type(local_costmap["height"]) is int
    assert global_costmap["inflation_layer"]["inflation_radius"] >= 0.40
    assert local_costmap["inflation_layer"]["inflation_radius"] >= 0.40


def test_common_costmap_parameters_target_both_internal_nodes():
    common = load_yaml("costmap_common_params.yaml")

    for node_pattern in ("/**/global_costmap", "/**/local_costmap"):
        params = common[node_pattern]["ros__parameters"]
        assert params["robot_base_frame"] == "base_footprint"
        assert params["obstacle_layer"]["observation_sources"] == "scan"
        scan = params["obstacle_layer"]["scan"]
        assert scan["topic"] == "/scan"
        assert scan["sensor_frame"] == "lidar_link"
        assert scan["marking"] is True
        assert scan["clearing"] is True
        assert scan["obstacle_min_range"] >= 0.35
        assert scan["raytrace_min_range"] == 0.0


def test_jazzy_planner_plugin_and_only_smac2d_parameters():
    planner = load_yaml("planner_server_params.yaml")["planner_server"][
        "ros__parameters"
    ]["GridBased"]

    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    for hybrid_only_parameter in (
        "motion_model_for_search",
        "angle_quantization_bins",
        "analytic_expansion_ratio",
        "analytic_expansion_max_length",
        "minimum_turning_radius",
    ):
        assert hybrid_only_parameter not in planner


def test_dwb_kinematics_and_critics_are_consistent():
    controller = load_yaml("controller_server_params.yaml")["controller_server"][
        "ros__parameters"
    ]["FollowPath"]

    assert controller["plugin"] == "dwb_core::DWBLocalPlanner"
    assert controller["min_vel_y"] == 0.0
    assert controller["max_vel_y"] == 0.0
    assert controller["vy_samples"] == 1
    assert "min_vel_theta" not in controller
    assert controller["min_speed_xy"] >= 0.0
    assert controller["min_speed_theta"] >= 0.0

    critics = controller["critics"]
    assert "ObstacleFootprint" in critics
    assert "PathDist" in critics
    for configured_critic in (
        "ObstacleFootprint",
        "PathAlign",
        "GoalAlign",
        "PathDist",
        "GoalDist",
        "PreferForward",
        "RotateToGoal",
        "Oscillation",
    ):
        assert configured_critic in critics


def test_launch_loads_all_johny_parameter_files():
    launch_file = (PACKAGE_DIR / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    for name in (
        "costmap_common_params.yaml",
        "global_costmap_params.yaml",
        "local_costmap_params.yaml",
        "planner_server_params.yaml",
        "controller_server_params.yaml",
    ):
        assert name in launch_file
