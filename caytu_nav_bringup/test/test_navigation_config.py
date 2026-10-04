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

    # La camera basse n'alimente que la costmap locale : un faux obstacle de sol
    # y est temporaire, alors qu'il resterait marque dans la globale.
    assert global_costmap["plugins"] == [
        "static_layer",
        "obstacle_layer",
        "top_camera_layer",
        "inflation_layer",
    ]
    assert local_costmap["plugins"] == [
        "obstacle_layer",
        "top_camera_layer",
        "bottom_camera_layer",
        "inflation_layer",
    ]
    assert type(local_costmap["width"]) is int
    assert type(local_costmap["height"]) is int
    # Le footprint physique protège le châssis ; ces rayons conservent ensuite
    # une marge sans fermer les couloirs de la carte du stade.
    assert global_costmap["inflation_layer"]["inflation_radius"] == 0.30
    assert local_costmap["inflation_layer"]["inflation_radius"] == 0.25


def test_common_costmap_parameters_target_both_internal_nodes():
    common = load_yaml("costmap_common_params.yaml")

    for node_pattern in ("/**/global_costmap", "/**/local_costmap"):
        params = common[node_pattern]["ros__parameters"]
        assert params["robot_base_frame"] == "base_footprint"
        assert params["footprint"] == (
            "[ [0.22, 0.25], [0.22, -0.25], "
            "[-0.31, -0.25], [-0.31, 0.25] ]"
        )
        assert params["footprint_padding"] == 0.02
        # Le lidar est seul dans sa couche : le clearing des cameras ne doit pas
        # pouvoir effacer ses obstacles.
        assert params["obstacle_layer"]["observation_sources"] == "scan"
        scan = params["obstacle_layer"]["scan"]
        # /scan est brut et contient l'auto-détection du châssis ; Nav2 doit
        # uniquement consommer le flux produit par laser_filters.
        assert scan["topic"] == "/scan_filtered"
        assert scan["sensor_frame"] == "lidar_link"
        assert scan["marking"] is True
        assert scan["clearing"] is True
        assert scan["obstacle_min_range"] >= 0.35
        assert scan["raytrace_min_range"] == 0.0


def test_each_camera_has_its_own_obstacle_layer():
    common = load_yaml("costmap_common_params.yaml")

    expected = {
        "/**/global_costmap": {"top_camera_layer": ("top_cam", "/top_camera_scan")},
        "/**/local_costmap": {
            "top_camera_layer": ("top_cam", "/top_camera_scan"),
            "bottom_camera_layer": ("bottom_cam", "/bottom_camera_scan"),
        },
    }
    for node_pattern, layers in expected.items():
        params = common[node_pattern]["ros__parameters"]
        assert "bottom_camera_layer" in params or node_pattern.endswith("global_costmap")
        for layer_name, (source, topic) in layers.items():
            layer = params[layer_name]
            assert layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
            assert layer["combination_method"] == 1
            assert layer["observation_sources"] == source
            observation = layer[source]
            assert observation["topic"] == topic
            assert observation["data_type"] == "LaserScan"
            assert observation["marking"] is True
            # Un scan de camera a z = 0 : une borne basse >= 0 le rejetterait.
            assert observation["min_obstacle_height"] < 0.0
            # Evite que la couche passe "not current" si un nuage est en retard.
            assert observation["expected_update_rate"] == 0.0


def test_every_costmap_plugin_is_fully_defined():
    common = load_yaml("costmap_common_params.yaml")
    for file_name, outer, node_pattern in (
        ("global_costmap_params.yaml", "global_costmap", "/**/global_costmap"),
        ("local_costmap_params.yaml", "local_costmap", "/**/local_costmap"),
    ):
        own = load_yaml(file_name)[outer][outer]["ros__parameters"]
        shared = common[node_pattern]["ros__parameters"]
        for layer_name in own["plugins"]:
            layer = own.get(layer_name) or shared.get(layer_name)
            assert layer is not None, f"{outer}: couche {layer_name} non definie"
            assert "plugin" in layer, f"{outer}: {layer_name} sans plugin"
            for source in str(layer.get("observation_sources", "")).split():
                assert source in layer, f"{outer}: source {source} non definie"


def test_camera_scan_nodes_filter_floor_noise():
    launch_file = (PACKAGE_DIR / "launch" / "solution_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    # Garde-fou : ne pas redescendre sous ces seuils sans avoir verifie dans RViz
    # que le sol n'y produit pas de points fantomes.
    assert '"min_height": 0.30' in launch_file  # camera haute
    assert '"min_height": 0.25' in launch_file  # camera basse
    assert "/top_camera_scan" in launch_file
    assert "/bottom_camera_scan" in launch_file


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


def test_navigation_launch_uses_package_relative_bt_path():
    launch_file = (PACKAGE_DIR / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "get_package_share_directory" in launch_file
    assert "navigate_bounded_recovery.xml" in launch_file
    assert "/home/" not in launch_file


def test_solution_bringup_starts_navigation_prerequisites():
    launch_file = (PACKAGE_DIR / "launch" / "solution_bringup.launch.py").read_text(
        encoding="utf-8"
    )

    for executable in (
        "scan_to_scan_filter_chain",
        "map_server",
        "amcl",
        "localization_watchdog",
    ):
        assert executable in launch_file
