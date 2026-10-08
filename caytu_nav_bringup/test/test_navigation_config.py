"""Garde-fous sur la configuration : chaque test protège un défaut déjà rencontré."""
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / "config"
LAUNCH_DIR = PACKAGE_DIR / "launch"
MAPS_DIR = PACKAGE_DIR / "maps"

# task_params.yaml officiel (repère Gazebo).
SPAWN = (-0.200546, -7.485170)
GOAL = (-2.293350, 2.232090)


def load_yaml(name):
    with (CONFIG_DIR / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def nav2():
    return load_yaml("nav2_params.yaml")


def costmap(name):
    return nav2()[name][name]["ros__parameters"]


def test_all_yaml_files_parse():
    for path in list(CONFIG_DIR.glob("*.yaml")) + list(MAPS_DIR.glob("*.yaml")):
        with path.open(encoding="utf-8") as stream:
            assert yaml.safe_load(stream) is not None, path.name


def test_no_amcl_slam_or_watchdog_left():
    # La localisation est assurée par odom_imu_localizer : plus de carte SLAM,
    # plus d'AMCL, plus de seuil de covariance (cause de « localisation perdue »).
    assert not (CONFIG_DIR / "amcl_params.yaml").exists()
    assert not (CONFIG_DIR / "slam_toolbox_params.yaml").exists()
    for launch_file in LAUNCH_DIR.glob("*.py"):
        text = launch_file.read_text(encoding="utf-8")
        assert "nav2_amcl" not in text
        assert "localization_watchdog" not in text
        assert "/home/" not in text


def test_every_costmap_plugin_and_source_is_defined():
    for name in ("global_costmap", "local_costmap"):
        params = costmap(name)
        assert params["robot_base_frame"] == "base_footprint"
        for layer_name in params["plugins"]:
            layer = params.get(layer_name)
            assert layer is not None, f"{name}: couche {layer_name} non définie"
            assert "plugin" in layer, f"{name}: {layer_name} sans plugin"
            for source in str(layer.get("observation_sources", "")).split():
                assert source in layer, f"{name}: source {source} non définie"
                assert layer[source]["data_type"] == "LaserScan"
                # Une couche en retard ne doit pas bloquer le contrôleur quand
                # la simulation tourne au ralenti.
                assert layer[source]["expected_update_rate"] == 0.0


def test_costmap_layout():
    # Caméra basse dans la costmap locale seulement : disposition de la version
    # de référence.
    assert costmap("global_costmap")["plugins"] == [
        "static_layer", "lidar_layer", "top_camera_layer", "inflation_layer"]
    assert costmap("local_costmap")["plugins"] == [
        "lidar_layer", "top_camera_layer", "bottom_camera_layer", "inflation_layer"]
    local = costmap("local_costmap")
    assert local["rolling_window"] is True
    assert type(local["width"]) is int and type(local["height"]) is int
    assert costmap("global_costmap")["global_frame"] == "map"
    assert local["global_frame"] == "odom"


def test_lidar_layer_never_uses_raw_or_floor_returns():
    for name in ("global_costmap", "local_costmap"):
        layer = costmap(name)["lidar_layer"]
        assert layer["observation_sources"] == "lidar_mark lidar_clear"
        mark, clear = layer["lidar_mark"], layer["lidar_clear"]
        # /scan brut contient le robot lui-même, /scan_filtered contient le sol.
        assert mark["topic"] == "/scan_clean"
        assert clear["topic"] == "/scan_clear"
        assert mark["marking"] is True and mark["clearing"] is False
        assert clear["marking"] is False and clear["clearing"] is True
        assert mark["obstacle_min_range"] >= 0.35


def test_camera_layers_cannot_mark_empty_rays():
    # Avec inf_is_valid, Nav2 remplace un rayon vide par un point à range_max
    # du scan. Si obstacle_max_range n'est pas plus petit, ce point devient un
    # obstacle fantôme (défaut de la version précédente : 3.0 pour 3.0).
    launch = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    assert "TOP_CAMERA_RANGE = 2.5" in launch and "CAMERA_CLEAR_MARGIN = 0.5" in launch
    assert '"range_max": reach + CAMERA_CLEAR_MARGIN' in launch and '"range_max": 1.2' in launch
    scan_range_max = {"/top_camera_scan": 2.5 + 0.5, "/bottom_camera_scan": 1.2}
    seen = 0
    for name in ("global_costmap", "local_costmap"):
        params = costmap(name)
        for layer_name in ("top_camera_layer", "bottom_camera_layer"):
            if layer_name not in params["plugins"]:
                continue
            layer = params[layer_name]
            assert layer["combination_method"] == 1
            source = layer[layer["observation_sources"]]
            assert source["inf_is_valid"] is True
            assert source["marking"] is True and source["clearing"] is True
            assert source["obstacle_max_range"] < scan_range_max[source["topic"]]
            assert source["raytrace_max_range"] <= scan_range_max[source["topic"]]
            if source["topic"] == "/top_camera_scan":
                # Même portée de marquage dans le launch et dans les deux costmaps.
                assert source["obstacle_max_range"] == 2.5
            # Les points d'un scan caméra ont z ~ 0 dans la costmap.
            assert source["min_obstacle_height"] < 0.0
            seen += 1
    assert seen == 3        # caméra haute : globale + locale ; caméra basse : locale


def test_inflation_gives_a_cost_slope_beyond_the_robot():
    inscribed_radius = 0.24
    for name in ("global_costmap", "local_costmap"):
        inflation = costmap(name)["inflation_layer"]
        assert inflation["inflation_radius"] >= inscribed_radius + 0.25
    rpp = nav2()["controller_server"]["ros__parameters"]["FollowPath"]
    assert rpp["inflation_cost_scaling_factor"] == costmap("local_costmap")[
        "inflation_layer"]["cost_scaling_factor"]


def test_camera_scans_are_filtered_in_the_level_frame():
    text = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    assert 'LEVEL_FRAME = "base_footprint_level"' in text
    assert text.count('"target_frame": LEVEL_FRAME') == 2
    assert text.count("pointcloud_to_laserscan_node") == 2
    # Bruit de profondeur Gazebo : 0.10 m sur la hauteur. Ne pas redescendre
    # sous 0.60 m pour la caméra haute sans revérifier sur sol dégagé.
    assert '"min_height": 0.60' in text
    assert '"min_height": 0.25' in text
    for node in ("scan_to_scan_filter_chain", "lidar_floor_filter",
                 "odom_imu_localizer", "pointcloud_to_laserscan_node"):
        assert node in text
    assert "TimerAction" not in text      # le temps de la tâche court dès le lancement


def test_navigation_launch_starts_everything_in_one_lifecycle_manager():
    text = (LAUNCH_DIR / "navigation.launch.py").read_text(encoding="utf-8")
    for executable in ("map_server", "controller_server", "planner_server",
                       "behavior_server", "bt_navigator", "lifecycle_manager"):
        assert executable in text
    assert "/robot_base_controller/cmd_vel_unstamped" in text
    assert "navigate_bounded_recovery.xml" in text
    assert "get_package_share_directory" in text
    # Un seul fichier de paramètres Nav2 : pas d'ordre de chargement à respecter.
    assert "nav2_params.yaml" in text and "controller_file" not in text


def test_controller_is_consistent_with_the_robot_and_the_scoring():
    common = nav2()["controller_server"]["ros__parameters"]
    assert common["enable_stamped_cmd_vel"] is False
    assert common["controller_plugins"] == ["FollowPath"]
    # Critère PARC n° 2 : distance finale au centre du but.
    assert common["goal_checker"]["xy_goal_tolerance"] <= 0.15
    assert common["goal_checker"]["yaw_goal_tolerance"] > 3.14

    rpp = common["FollowPath"]
    assert rpp["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
    assert not (rpp["use_rotate_to_heading"] and rpp["allow_reversing"])
    assert rpp["rotate_to_heading_angular_vel"] <= 1.0      # limite du plugin DiffDrive
    assert rpp["use_collision_detection"] is True
    assert rpp["desired_linear_vel"] <= 1.5                 # limite du plugin DiffDrive
    assert rpp["rotate_to_heading_min_angle"] == 0.6
    assert common["failure_tolerance"] == 1.0


def test_planner_is_smac_2d_alone():
    planner = nav2()["planner_server"]["ros__parameters"]
    # Smac Hybrid a été essayé puis retiré : mesuré plus lent dans Gazebo, avec
    # un arrêt tardif devant les obstacles.
    assert planner["planner_plugins"] == ["GridBased"]
    assert planner["GridBased"]["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["GridBased"]["cost_travel_multiplier"] == 3.0
    assert "Hybrid" not in (CONFIG_DIR / "nav2_params.yaml").read_text(
        encoding="utf-8").split("planner_plugins")[1]


def _read_pgm(path):
    data = path.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos)
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[pos + 1:pos + 1 + width * height]
    assert tokens[0] == b"P5" and len(pixels) == width * height
    return width, height, pixels


def test_cafe_map_is_in_the_gazebo_frame_and_keeps_spawn_and_goal_free():
    meta = yaml.safe_load((MAPS_DIR / "cafe_map.yaml").read_text(encoding="utf-8"))
    assert meta["mode"] == "trinary" and meta["resolution"] == 0.05
    width, height, pixels = _read_pgm(MAPS_DIR / meta["image"])
    origin_x, origin_y = meta["origin"][0], meta["origin"][1]

    def value(x, y):
        col = int((x - origin_x) / meta["resolution"])
        row = height - 1 - int((y - origin_y) / meta["resolution"])
        assert 0 <= col < width and 0 <= row < height, "point hors de la carte"
        return pixels[row * width + col]

    # Le spawn et le but officiels doivent être dans la zone libre, avec au
    # moins 0.5 m sans mur autour : la carte est bien dans le repère Gazebo.
    for x, y in (SPAWN, GOAL):
        for dx in (-0.5, 0.0, 0.5):
            for dy in (-0.5, 0.0, 0.5):
                assert value(x + dx, y + dy) == 254
    # Les murs longs du café sont à x = -4.93 et x = +4.34 dans Gazebo.
    assert any(value(-4.93 + d, -6.0) == 0 for d in (-0.1, -0.05, 0.0, 0.05, 0.1))
    assert any(value(4.34 + d, -6.0) == 0 for d in (-0.1, -0.05, 0.0, 0.05, 0.1))


# Nœuds fournis par nav2_behavior_tree (Jazzy) ou par BehaviorTree.CPP.
KNOWN_BT_NODES = {
    "root", "BehaviorTree", "RecoveryNode", "PipelineSequence", "RoundRobin",
    "RateController", "ControllerSelector", "PlannerSelector", "ComputePathToPose",
    "FollowPath", "IsPathValid", "GlobalUpdatedGoal", "WouldAPlannerRecoveryHelp",
    "WouldAControllerRecoveryHelp", "ClearEntireCostmap", "ClearCostmapExceptRegion",
    "Spin", "Wait", "BackUp",
    "Sequence", "Fallback", "ReactiveSequence", "Inverter",
}


def test_behavior_trees_are_well_formed_and_use_known_nodes():
    import xml.dom.minidom

    for name in ("navigate_bounded_recovery.xml", "navigate_keep_path.xml"):
        path = PACKAGE_DIR / "behavior_trees" / name
        document = xml.dom.minidom.parseString(path.read_bytes())
        tags = {node.tagName for node in document.getElementsByTagName("*")}
        assert tags <= KNOWN_BT_NODES, f"{name}: nœuds inconnus {tags - KNOWN_BT_NODES}"
        text = path.read_text(encoding="utf-8")
        assert 'main_tree_to_execute="MainTree"' in text
        assert "local_costmap/clear_entirely_local_costmap" in text
        assert 'default_controller="FollowPath"' in text
        assert 'default_planner="GridBased"' in text


def test_trees_never_wipe_the_whole_global_costmap():
    # Un plateau de table n'est vu que par les caméras, vers l'avant. Effacer
    # toute la costmap globale quand une table est sur le côté du robot ferait
    # passer le chemin suivant à travers elle : seule la zone lointaine est
    # effacée (carré de 5 m gardé autour du robot).
    for name in ("navigate_bounded_recovery.xml", "navigate_keep_path.xml"):
        text = (PACKAGE_DIR / "behavior_trees" / name).read_text(encoding="utf-8")
        assert "global_costmap/clear_entirely_global_costmap" not in text
        assert text.count("global_costmap/clear_except_global_costmap") == 2
        assert text.count('reset_distance="5.0"') == 2


def test_trees_use_the_single_configured_planner():
    import xml.dom.minidom

    plugins = nav2()["planner_server"]["ros__parameters"]["planner_plugins"]
    for name in ("navigate_bounded_recovery.xml", "navigate_keep_path.xml"):
        path = PACKAGE_DIR / "behavior_trees" / name
        document = xml.dom.minidom.parseString(path.read_bytes())
        nodes = document.getElementsByTagName("ComputePathToPose")
        assert [node.getAttribute("planner_id") for node in nodes] == ["{selected_planner}"]
        assert nodes[0].getAttribute("error_code_id") == "{compute_path_error_code}"
        selector = document.getElementsByTagName("PlannerSelector")[0]
        # Un planificateur nommé dans un arbre mais absent de la configuration
        # ferait échouer toute planification.
        assert selector.getAttribute("default_planner") in plugins


def test_recovery_ladder_is_bounded_and_ordered():
    import xml.dom.minidom

    for name in ("navigate_bounded_recovery.xml", "navigate_keep_path.xml"):
        path = PACKAGE_DIR / "behavior_trees" / name
        document = xml.dom.minidom.parseString(path.read_bytes())
        round_robin = document.getElementsByTagName("RoundRobin")[0]
        actions = [child.tagName for child in round_robin.childNodes if child.nodeType == 1]
        assert actions == ["Sequence", "Wait", "Spin", "BackUp"]
        top = document.getElementsByTagName("RecoveryNode")[0]
        assert top.getAttribute("name") == "NavigateRecovery"
        # Une tentative par action de récupération, puis Nav2 rend la main.
        assert int(top.getAttribute("number_of_retries")) == len(actions)


def test_tree_timeouts_tolerate_a_loaded_machine():
    bt = nav2()["bt_navigator"]["ros__parameters"]
    # À 20 ms (valeur par défaut de Nav2), une réponse lente d'un serveur fait
    # échouer un nœud de l'arbre : chemin déclaré invalide, ou navigation
    # abandonnée sans récupération.
    assert bt["default_server_timeout"] >= 200
    assert bt["default_cancel_timeout"] >= 200
    assert bt["wait_for_service_timeout"] >= 1000


def test_nodes_are_restarted_if_they_die():
    bringup = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    navigation = (LAUNCH_DIR / "navigation.launch.py").read_text(encoding="utf-8")
    # 2 filtres du lidar, 2 caméras, chemin stabilisé. La localisation n'est
    # pas relancée : elle repartirait du point de départ.
    assert bringup.count("**RESPAWN") == 5
    # carte, contrôleur, planificateur, behaviors, BT. Le lifecycle manager
    # n'est pas relancé : il referait un démarrage complet.
    assert navigation.count("**RESPAWN") == 5
    for text in (bringup, navigation):
        assert '"respawn": True' in text and '"respawn_delay": 2.0' in text
    assert '"attempt_respawn_reconnection": True' in navigation


def test_both_trees_replan_periodically_like_the_reference_version():
    # Replanification périodique à 2 Hz : conduite de la version mesurée à
    # 2 min 33 s. « Seulement si le chemin est invalide » a été mesuré plus
    # lent (le robot avançait trop avant de s'arrêter puis de pivoter).
    for name in ("navigate_bounded_recovery.xml", "navigate_keep_path.xml"):
        text = (PACKAGE_DIR / "behavior_trees" / name).read_text(encoding="utf-8")
        assert '<RateController hz="2.0">' in text and "IsPathValid" not in text


def _tree_nodes(name):
    """Liste (balise, attributs) de tous les nœuds d'un arbre, dans l'ordre."""
    import xml.dom.minidom

    document = xml.dom.minidom.parseString((PACKAGE_DIR / "behavior_trees" / name).read_bytes())
    return [(node.tagName, dict(node.attributes.items()))
            for node in document.getElementsByTagName("*")]


def test_keep_path_tree_differs_from_the_default_tree_by_one_attribute():
    default, keeper = _tree_nodes("navigate_bounded_recovery.xml"), _tree_nodes(
        "navigate_keep_path.xml")
    assert len(default) == len(keeper)
    differences = [(a, b) for a, b in zip(default, keeper) if a != b]
    assert len(differences) == 1
    (tag, plain), (_, stable) = differences[0]
    assert tag == "ComputePathToPose" and "server_name" not in plain
    assert stable == {**plain, "server_name": "compute_path_stable"}


def test_path_keeper_is_launched_and_reads_the_planner_settings():
    bringup = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    assert 'executable="path_keeper"' in bringup
    assert 'LaunchConfiguration("path_keeper").perform(context).lower() != "true"' in bringup
    # path_keeper lit la costmap globale publiée : elle doit l'être en entier
    # et aussi souvent qu'elle est mise à jour.
    params = costmap("global_costmap")
    assert params["always_send_full_costmap"] is True
    assert params["publish_frequency"] >= params["update_frequency"]


def test_drive_arguments_are_declared_empty_and_forwarded():
    bringup = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    navigation = (LAUNCH_DIR / "navigation.launch.py").read_text(encoding="utf-8")
    for name in ("cruise_speed", "turn_speed", "approach_speed", "approach_distance",
                 "curve_radius", "camera_range"):
        assert f'"{name}"' in bringup and f'"{name}"' in navigation
    # Sans argument, nav2_params.yaml est utilisé tel quel (aucune copie).
    assert "return source" in navigation
    assert '"axle_offset", default_value=""' in bringup
    assert '"keeper_gain", default_value=""' in bringup


def test_tried_settings_exist_in_the_controller_configuration():
    rpp = nav2()["controller_server"]["ros__parameters"]["FollowPath"]
    for key in ("desired_linear_vel", "rotate_to_heading_angular_vel",
                "min_approach_linear_velocity", "approach_velocity_scaling_dist",
                "regulated_linear_scaling_min_radius"):
        assert isinstance(rpp[key], float), key
    # Arrivée : assez lente pour s'arrêter dans la tolérance du but.
    tolerance = nav2()["controller_server"]["ros__parameters"]["goal_checker"][
        "xy_goal_tolerance"]
    assert rpp["min_approach_linear_velocity"] ** 2 / (2 * 1.0) < tolerance
    # Le robot doit pouvoir s'arrêter (1 m/s²) avant la fin de la zone qu'il voit.
    assert rpp["desired_linear_vel"] ** 2 / (2 * 1.0) < 0.5
