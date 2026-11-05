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
    for path in list(CONFIG_DIR.rglob("*.yaml")) + list(MAPS_DIR.rglob("*.yaml")):
        with path.open(encoding="utf-8") as stream:
            assert yaml.safe_load(stream) is not None, path.name


def test_amcl_is_optional_and_never_runs_with_the_custom_localizer():
    # AMCL remplace le localisateur Fosa ; les deux ne publient pas map -> odom.
    assert not (CONFIG_DIR / "slam_toolbox_params.yaml").exists()
    solution = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    navigation = (LAUNCH_DIR / "navigation.launch.py").read_text(encoding="utf-8")
    assert 'if localisation == "amcl":' in solution
    assert 'return []' in solution
    assert 'package="nav2_amcl"' in navigation
    assert 'noeuds_cycle_vie.insert(1, "amcl")' in navigation
    assert "localization_watchdog" not in solution + navigation
    assert "/home/" not in solution + navigation


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
    scan_range_max = {"/top_camera_scan": 3.0, "/bottom_camera_scan": 1.2}
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
            # Les points d'un scan caméra ont z ~ 0 dans la costmap.
            assert source["min_obstacle_height"] < 0.0
            seen += 1
    assert seen == 3


def test_inflation_gives_a_cost_slope_beyond_the_robot():
    inscribed_radius = 0.24
    for name in ("global_costmap", "local_costmap"):
        inflation = costmap(name)["inflation_layer"]
        assert inflation["inflation_radius"] >= inscribed_radius + 0.25
    rpp = load_yaml("controller_rpp.yaml")["controller_server"]["ros__parameters"]["FollowPath"]
    assert rpp["inflation_cost_scaling_factor"] == costmap("local_costmap")[
        "inflation_layer"]["cost_scaling_factor"]


def test_camera_scans_are_filtered_in_the_level_frame():
    text = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    filtres = (LAUNCH_DIR / "robot_interfaces.launch.py").read_text(encoding="utf-8")
    # Le repère horizontal publié par le filtre sol est celui des caméras.
    assert "REPERE_HORIZONTAL = CONFIG.perception.top_scan.target_frame" in text
    assert "LEVEL_FRAME = load_config().perception.top_scan.target_frame" in filtres
    assert "parameters=[_parametres_scan(scan, temps_simulation)]" in text
    # Les seuils de hauteur viennent de caytu_config, pas du fichier launch.
    from caytu_config import load_config
    configuration = load_config()
    assert configuration.perception.top_scan.min_height == 0.60
    assert configuration.perception.bottom_scan.min_height == 0.25
    for node in ("localisateur_odom_imu", "pointcloud_to_laserscan_node",
                 "robot_interfaces.launch.py", "localization_launch.py"):
        assert node in text
    for node in ("scan_to_scan_filter_chain", "filtre_sol_lidar"):
        assert node in filtres
    for fichier in ("solution_bringup.launch.py", "robot_interfaces.launch.py"):
        contenu = (LAUNCH_DIR / fichier).read_text(encoding="utf-8")
        assert "TimerAction" not in contenu  # le temps de la tâche court dès le lancement


def test_navigation_launch_starts_everything_in_one_lifecycle_manager():
    text = (LAUNCH_DIR / "navigation.launch.py").read_text(encoding="utf-8")
    for executable in ("map_server", "controller_server", "planner_server",
                       "behavior_server", "bt_navigator", "lifecycle_manager"):
        assert executable in text
    assert "TOPIC_VITESSE = CONFIG.navigation.cmd_vel_topic" in text
    # Robot réel : solution_bringup passe le topic du profil.
    assert '"cmd_vel_topic", default_value=TOPIC_VITESSE' in text
    assert text.count('remappings=[("cmd_vel", topic_vitesse)]') == 2
    from caytu_config import load_config
    assert load_config().navigation.cmd_vel_topic == (
        "/robot_base_controller/cmd_vel_unstamped")
    assert "navigate_bounded_recovery.xml" in text
    assert "get_package_share_directory" in text
    assert 'choices=["rpp", "dwb", "mppi"]' in text


def test_controller_files_exist_and_are_consistent():
    common = nav2()["controller_server"]["ros__parameters"]
    assert common["enable_stamped_cmd_vel"] is False
    assert common["controller_plugins"] == ["FollowPath"]
    # Critère PARC n° 2 : distance finale au centre du but.
    assert common["goal_checker"]["xy_goal_tolerance"] <= 0.15
    assert common["goal_checker"]["yaw_goal_tolerance"] > 3.14

    rpp = load_yaml("controller_rpp.yaml")["controller_server"]["ros__parameters"]["FollowPath"]
    assert rpp["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
    assert not (rpp["use_rotate_to_heading"] and rpp["allow_reversing"])
    assert rpp["rotate_to_heading_angular_vel"] <= 1.0      # limite du plugin DiffDrive

    dwb = load_yaml("controller_dwb.yaml")["controller_server"]["ros__parameters"]["FollowPath"]
    assert dwb["plugin"] == "dwb_core::DWBLocalPlanner"
    assert dwb["debug_trajectory_details"] is False
    assert dwb["max_vel_y"] == 0.0 and dwb["vy_samples"] == 1
    for critic in dwb["critics"]:
        assert critic in ("RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign",
                          "PathAlign", "PathDist", "GoalDist", "PreferForward")

    mppi = load_yaml("controllers/mppi.yaml")["controller_server"]["ros__parameters"][
        "FollowPath"]
    assert mppi["plugin"] == "nav2_mppi_controller::MPPIController"
    assert mppi["motion_model"] == "DiffDrive"
    assert mppi["visualize"] is False
    assert mppi["batch_size"] <= 1000
    assert mppi["vx_max"] <= 1.5 and mppi["wz_max"] <= 1.0


def test_planner_is_smac_2d_without_hybrid_parameters():
    planner = nav2()["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    for hybrid_only in ("motion_model_for_search", "angle_quantization_bins",
                        "analytic_expansion_ratio", "minimum_turning_radius"):
        assert hybrid_only not in planner


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


def test_behavior_tree_only_uses_known_recovery_services():
    text = (PACKAGE_DIR / "behavior_trees" / "navigate_bounded_recovery.xml").read_text(
        encoding="utf-8")
    assert 'main_tree_to_execute="MainTree"' in text
    assert "local_costmap/clear_entirely_local_costmap" in text
    assert "global_costmap/clear_entirely_global_costmap" in text
    assert 'default_controller="FollowPath"' in text
    assert 'default_planner="GridBased"' in text


# --------------------------------------------------------------------------- #
# Cartographie manuelle (slam_toolbox) et localisation sur la carte enregistrée
# --------------------------------------------------------------------------- #
def slam(name):
    return load_yaml(name)["slam_toolbox"]["ros__parameters"]


def test_slam_toolbox_uses_the_filtered_lidar_and_the_navigation_frames():
    resolution = costmap("global_costmap")["resolution"]
    lidar_min = costmap("local_costmap")["lidar_layer"]["lidar_mark"]["obstacle_min_range"]
    for name, mode in (("slam_mapping.yaml", "mapping"),
                       ("slam_localization.yaml", "localization")):
        params = slam(name)
        assert params["mode"] == mode
        # /scan contient le robot, /fosa/scan_filtered contient encore le sol.
        assert params["scan_topic"] == "/scan_clean"
        assert params["base_frame"] == "base_footprint"
        assert (params["map_frame"], params["odom_frame"]) == ("map", "odom")
        assert params["resolution"] == resolution
        assert params["min_laser_range"] >= lidar_min
        # RPLIDAR C1 : 12 m de portée ; l'arène fait 5 m x 5 m.
        assert 7.1 <= params["max_laser_range"] <= 12.0
        assert params["transform_publish_period"] > 0.0
        # La carte et la pose de départ sont ajoutées au lancement.
        for key in ("map_file_name", "map_start_pose", "map_start_at_dock"):
            assert key not in params, f"{name}: {key} écrit en dur"
    assert slam("slam_mapping.yaml")["do_loop_closing"] is True


def test_mapping_launch_starts_slam_toolbox_session_and_shared_filters():
    text = (LAUNCH_DIR / "mapping.launch.py").read_text(encoding="utf-8")
    for needed in ("robot_interfaces.launch.py", "online_async_launch.py", "slam_mapping.yaml",
                   "mapping_session", "tf_bag_relay", "--topics"):
        assert needed in text
    # Les nœuds de navigation ne tournent pas pendant la cartographie manuelle.
    for absent in ("localisateur_odom_imu", "navigation.launch.py", "controller_server"):
        assert absent not in text
    rviz = (PACKAGE_DIR / "rviz" / "mapping.rviz").read_text(encoding="utf-8")
    assert yaml.safe_load(rviz)["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"


def test_slam_localization_keeps_map_server_as_the_only_map_publisher():
    text = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    # Nav2 lit la carte (retouchée) de map_server ; celle de slam_toolbox est déviée.
    assert 'SetRemap(src="/map", dst="/slam_map")' in text
    assert 'choices=["odom_imu", "odom_only", "amcl", "slam", "slam_live"]' in text
    # Nav2 attend la première position (sur événement) : sinon la costmap
    # globale abandonne son activation en temps simulé. Sauf avec AMCL, que
    # Nav2 lance lui-même.
    assert 'executable="wait_for_tf"' in text and "OnProcessExit" in text
    assert 'if localisation == "amcl":\n        return [navigation]' in text



# --------------------------------------------------------------------------- #
# Profils de robot (config/robots/) : Gazebo phase 1 et robot réel de Dakar
# --------------------------------------------------------------------------- #
def profile(name):
    with (CONFIG_DIR / "robots" / f"{name}.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


PROFILE_KEYS = ("robot", "cmd_vel_topic", "odom_topic", "imu_topic", "scan_topic",
                "base_footprint", "laser_filter_file", "floor_filter", "cameras",
                "collision_cameras", "record_topics", "record_camera_topics")


def test_profiles_are_complete_and_point_to_existing_files():
    for name in ("sim", "parc", "real", "real_remote"):
        data = profile(name)
        for key in PROFILE_KEYS:
            assert key in data, f"{name}: {key} manquant"
        assert data["robot"] == name
        assert (CONFIG_DIR / data["laser_filter_file"]).is_file()
        for camera in data["cameras"]:
            assert data["cameras"][camera]["cloud"].endswith("/points")
        assert "/tf" in data["record_topics"] and "/tf_static" in data["record_topics"]
        assert data["scan_topic"] in data["record_topics"]
        assert data["odom_topic"] in data["record_topics"]


def test_sim_profile_matches_caytu_config():
    # En simulation, caytu_config reste la référence : le profil sim ne doit
    # pas s'en écarter.
    from caytu_config import load_config
    configuration = load_config()
    sim = profile("sim")
    assert sim["cmd_vel_topic"] == configuration.navigation.cmd_vel_topic
    assert sim["scan_topic"] == configuration.navigation.scan_topic
    assert sim["cameras"]["top"]["cloud"] == configuration.perception.top_scan.cloud_topic
    assert sim["cameras"]["bottom"]["cloud"] == (
        configuration.perception.bottom_scan.cloud_topic)
    localisateur = load_yaml("localization/odom_imu.yaml")[
        "localisateur_odom_imu"]["ros__parameters"]
    assert (sim["odom_topic"], sim["imu_topic"]) == (
        localisateur["odom_topic"], localisateur["imu_topic"])
    assert sim["base_footprint"]["publish"] is False
    assert sim["laser_filter_file"] == "laser_filter_params.yaml"
    assert sim["floor_filter"] == {}
    assert sim["cameras"]["top"]["cloud"] == "/top_camera_depth/points"
    assert "depth_image" not in sim["cameras"]["top"]


def test_real_profile_follows_the_official_topic_list():
    # Relevé officiel PARC du 2 novembre 2026.
    real = profile("real")
    assert real["cmd_vel_topic"] == "/cmd_vel"
    assert real["odom_topic"] == "/botwheel_explorer/odom"
    assert real["imu_topic"] == ""                     # pas d'IMU de base
    assert real["scan_topic"] == "/scan"
    # L'arbre TF réel s'arrête à base_link : on ajoute base_footprint dessous.
    assert real["base_footprint"]["publish"] is True
    assert real["base_footprint"]["parent"] == "base_link"
    assert real["floor_filter"]["imu_topic"] == ""
    assert real["cameras"]["top"]["depth_image"].endswith("/depth/image_rect_raw")
    assert real["cameras"]["bottom"]["depth_image"].endswith("/depth/image_rect_raw")
    for topic in ("/scan", "/botwheel_explorer/odom", "/tf", "/tf_static"):
        assert topic in real["record_topics"]


def test_solution_never_publishes_on_organizer_topics():
    reserved = set(profile("real")["reserved_topics"])
    for must_be_reserved in ("/scan_filtered", "/goal", "/botwheel_explorer/cmd_vel",
                             "/stamped_vel", "/camera1/scan", "/camera2/scan"):
        assert must_be_reserved in reserved
    published = {"/fosa/scan_filtered", "/scan_clean", "/scan_clear", "/top_camera_scan",
                 "/bottom_camera_scan", "/robot_tilt", "/localization_pose", "/map",
                 "/slam_map", "/slam_map_metadata"}
    # Topics publiés par la perception et l'assistant (caytu_config).
    from caytu_config import load_config
    configuration = load_config()
    perception, dialogue = configuration.perception, configuration.dialogue
    published.update({perception.entities_topic, perception.tags_topic,
                      perception.person_events_topic, perception.top_scan.scan_topic,
                      perception.bottom_scan.scan_topic, dialogue.transcript_topic,
                      dialogue.response_topic, dialogue.state_topic})
    for name in ("sim", "parc", "real", "real_remote"):
        data = profile(name)
        published.add(data["cmd_vel_topic"])
        published.update(cam["cloud"] for cam in data["cameras"].values())
    assert not published & reserved, published & reserved
    # Notre laser_filters ne sort jamais sur /scan_filtered.
    interfaces = (LAUNCH_DIR / "robot_interfaces.launch.py").read_text(encoding="utf-8")
    assert 'FILTERED_SCAN = "/fosa/scan_filtered"' in interfaces
    assert '("scan_filtered", FILTERED_SCAN)' in interfaces
    for launch_file in LAUNCH_DIR.glob("*.py"):
        assert '"/scan_filtered"' not in launch_file.read_text(encoding="utf-8")


def test_real_laser_filter_removes_the_robot_itself():
    with (CONFIG_DIR / "laser_filter_params_real.yaml").open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["scan_to_scan_filter_chain"]["ros__parameters"]
    filters = [params[key] for key in sorted(params) if key.startswith("filter")]
    assert [f["type"] for f in filters] == ["laser_filters/LaserScanRangeFilter",
                                            "laser_filters/LaserScanBoxFilter"]
    lower = filters[0]["params"]["lower_threshold"]
    # Sous les seuils de slam_toolbox et de la couche lidar de Nav2 (0,35 m).
    assert 0.0 < lower < slam("slam_mapping.yaml")["min_laser_range"]
    # Le lidar voit les roues et la carrosserie (0,17-0,28 m dans le simulateur
    # officiel) : la boîte couvre toute l'emprise de Nav2, marge comprise.
    boite = filters[1]["params"]
    assert boite["box_frame"] == "base_footprint" and boite["invert"] is False
    with (CONFIG_DIR / "nav2_params.yaml").open(encoding="utf-8") as stream:
        nav2 = yaml.safe_load(stream)
    emprise = yaml.safe_load(nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
    xs, ys = [p[0] for p in emprise], [p[1] for p in emprise]
    assert boite["min_x"] < min(xs) and boite["max_x"] > max(xs)
    assert boite["min_y"] < min(ys) and boite["max_y"] > max(ys)


def test_imu_corrected_slam_odometry_is_an_opt_in_trial():
    # Par défaut slam_toolbox lit odom ; imu_odom:=true essaie fosa_odom (roues
    # + cap de l'IMU), seulement si le profil a une IMU de base.
    text = (LAUNCH_DIR / "slam_odometry.launch.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("imu_odom", default_value="false")' in text
    assert 'imu_odom and profile.get("imu_topic")' in text
    for launch_file in ("mapping.launch.py", "solution_bringup.launch.py"):
        content = (LAUNCH_DIR / launch_file).read_text(encoding="utf-8")
        assert '"imu_odom", default_value="false"' in content
    assert 'executable="localisateur_odom_imu"' in text
    assert '"use_task_spawn": False' in text and '"odom_frame": "odom"' in text
    assert '"transform_tolerance": 0.0' in text
    # Le correcteur n'est lancé qu'avec slam_toolbox, jamais avec la
    # localisation de la phase 1 (deux parents pour odom sinon).
    for launch_file in ("mapping.launch.py", "solution_bringup.launch.py"):
        assert "slam_odometry.launch.py" in (LAUNCH_DIR / launch_file).read_text(encoding="utf-8")
    bringup = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    debut = bringup.index("def _creer_localisation_slam")
    fin = bringup.index("def _creer_localisation_et_navigation")
    assert "slam_odometry.launch.py" in bringup[debut:fin]
    assert "slam_odometry.launch.py" not in bringup[:debut] + bringup[fin:]
    assert "slam_odometry" not in (LAUNCH_DIR / "robot_interfaces.launch.py").read_text(
        encoding="utf-8")


def test_remote_profile_is_the_real_robot_without_cameras():
    # Solution sur le portable : mêmes interfaces que le robot réel, mais pas
    # d'images de profondeur à travers le Wi-Fi.
    real, remote = profile("real"), profile("real_remote")
    assert remote["cameras"] == {}
    for key in PROFILE_KEYS:
        if key not in ("robot", "cameras", "collision_cameras"):
            assert remote[key] == real[key], key
    assert remote["collision_cameras"] is False         # pas de caméras à distance
    assert remote["reserved_topics"] == real["reserved_topics"]
    text = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    assert "if not cameras:" in text
    assert 'ROBOTS = ["sim", "parc", "real", "real_remote"]' in text
    for launch_file in ("robot_interfaces.launch.py", "slam_odometry.launch.py",
                        "mapping.launch.py"):
        assert '"real_remote"]' in (LAUNCH_DIR / launch_file).read_text(encoding="utf-8")


def test_real_robot_config_overlay_matches_the_real_profile(monkeypatch):
    # caytu_config/config/robot_reel.yaml (FOSA_CONFIG_OVERRIDE) et le profil
    # real doivent donner les mêmes topics au robot réel.
    from caytu_config import load_config
    dossier = PACKAGE_DIR.parent / "caytu_config" / "config"
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(dossier / "robot_reel.yaml"))
    configuration = load_config(dossier)
    real = profile("real")
    assert configuration.system.use_sim_time is False
    assert configuration.system.ros_domain_id == 0
    assert configuration.navigation.cmd_vel_topic == real["cmd_vel_topic"]
    assert configuration.perception.top_scan.cloud_topic == real["cameras"]["top"]["cloud"]
    assert configuration.perception.bottom_scan.cloud_topic == (
        real["cameras"]["bottom"]["cloud"])
    assert configuration.perception.top_depth_topic == real["cameras"]["top"]["cloud"]
    haute = real["cameras"]["top"]["depth_image"].split("/")[1]      # camera2
    assert configuration.perception.top_color_topic.startswith(f"/{haute}/")


# --------------------------------------------------------------------------- #
# Course de la phase finale
# --------------------------------------------------------------------------- #
def test_course_launch_starts_the_whole_race_without_moving_before_the_start():
    text = (LAUNCH_DIR / "course.launch.py").read_text(encoding="utf-8")
    for needed in ("solution_bringup.launch.py", "tags.launch.py", "collision_monitor",
                   "mot_depart", "course_tags", '"slam_live"', '"localization": "slam"'):
        assert needed in text, needed
    # Le départ n'est jamais automatique en course.
    assert 'DeclareLaunchArgument(\n            "depart_immediat", default_value="false"' in text
    assert 'DeclareLaunchArgument(\n            "ordre", default_value=""' in text
    tags = (LAUNCH_DIR / "tags.launch.py").read_text(encoding="utf-8")
    for needed in ("apriltag_ros", "apriltag_node", "tag_detector", "registre_tags"):
        assert needed in tags
    mapping = (LAUNCH_DIR / "mapping.launch.py").read_text(encoding="utf-8")
    assert "tags.launch.py" in mapping and '"recharger": "false"' in mapping
    reperage = (LAUNCH_DIR / "reperage_tags.launch.py").read_text(encoding="utf-8")
    assert '"localization": "slam"' in reperage and '"recharger": "false"' in reperage


def _points(polygone):
    return yaml.safe_load(polygone["points"])


def test_safety_zone_stops_before_contact_but_lets_the_robot_reach_the_tag():
    from caytu_config import load_config
    course = load_config().course
    params = load_yaml("collision_monitor.yaml")["collision_monitor"]["ros__parameters"]
    arret = _points(params["ZoneArret"])
    ralenti = _points(params["ZoneRalenti"])
    empreinte = yaml.safe_load(nav2()["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    avant_robot = max(x for x, _ in empreinte)
    avant_arret = max(x for x, _ in arret)
    # Arrêt avant le contact, mais le support du tag (à 0,5 m du centre)
    # reste hors de la zone d'arrêt.
    assert avant_robot < avant_arret < course.visit_distance_m - 0.1
    assert max(abs(y) for _, y in arret) > max(abs(y) for _, y in empreinte)
    assert min(x for x, _ in arret) < min(x for x, _ in empreinte)
    assert max(x for x, _ in ralenti) > avant_arret
    assert params["ZoneArret"]["action_type"] == "stop"
    assert params["observation_sources"] == ["lidar"]       # caméras ajoutées au lancement
    assert params["lidar"]["topic"] == "/scan_clean"


def test_race_topics_never_collide_with_organizer_topics():
    from caytu_config import load_config
    course = load_config().course
    reserved = set(profile("real")["reserved_topics"])
    ours = {course.start_topic, course.status_topic, course.visited_tag_topic,
            course.visit_image_topic, course.tags_topic, "/fosa/cmd_vel_brut",
            "/fosa/tags_vus", "/fosa/tags_marqueurs"}
    assert not ours & reserved
    assert course.visit_distance_m == 0.5 and course.yaw_tolerance_deg <= 12.0
    assert tuple(course.expected_ids) == (1, 4, 7, 9)


def test_parc_profile_is_the_real_robot_in_simulated_time():
    # Simulateur officiel du 5 novembre : topics du vrai robot, base_footprint
    # déjà dans l'URDF, camera1 en haut.
    parc, real = profile("parc"), profile("real")
    for key in ("cmd_vel_topic", "odom_topic", "imu_topic", "scan_topic",
                "laser_filter_file", "floor_filter", "cameras", "reserved_topics"):
        assert parc[key] == real[key], key
    assert parc["base_footprint"]["publish"] is False
    # Simulateur souvent lent : caméras hors du garde-fou ; vrai robot : dedans.
    assert parc["collision_cameras"] is False and real["collision_cameras"] is True
    assert 'garde_fou_cameras' in (LAUNCH_DIR / "course.launch.py").read_text(encoding="utf-8")
    assert real["cameras"]["top"]["depth_image"].startswith("/camera1/")
    assert real["cameras"]["bottom"]["depth_image"].startswith("/camera2/")
    for launch_file in ("course.launch.py", "reperage_tags.launch.py"):
        texte = (LAUNCH_DIR / launch_file).read_text(encoding="utf-8")
        assert 'robot in ("sim", "parc")' in texte
    assert 'robot == "parc"' in (LAUNCH_DIR / "mapping.launch.py").read_text(encoding="utf-8")


def test_sim_parc_overlay_matches_real_overlay_except_time_and_tag_size(monkeypatch):
    from caytu_config import load_config
    dossier = PACKAGE_DIR.parent / "caytu_config" / "config"
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(dossier / "robot_reel.yaml"))
    reel = load_config()
    monkeypatch.setenv("FOSA_CONFIG_OVERRIDE", str(dossier / "sim_parc.yaml"))
    parc = load_config()
    assert parc.system.use_sim_time is True and reel.system.use_sim_time is False
    assert parc.perception.apriltag.size_m == 0.12
    assert parc.navigation.cmd_vel_topic == reel.navigation.cmd_vel_topic == "/cmd_vel"
    for champ in ("top_color_topic", "top_info_topic", "bottom_color_topic",
                  "top_depth_topic", "bottom_depth_topic"):
        assert getattr(parc.perception, champ) == getattr(reel.perception, champ), champ
    assert reel.perception.top_color_topic == "/camera1/camera1/color/image_raw"


def test_every_launch_file_compiles():
    # Les autres tests lisent les launch en texte : une parenthèse en trop
    # passerait inaperçue jusqu'au ros2 launch.
    import py_compile
    for fichier in LAUNCH_DIR.glob("*.launch.py"):
        py_compile.compile(str(fichier), doraise=True)


def test_controller_reads_the_robot_odometry_of_the_profile():
    # Sans odométrie, RPP croit le robot immobile : rotation sur place bornée à
    # max_angular_accel x dt (0,08 rad/s), progress checker en échec, recovery.
    solution = (LAUNCH_DIR / "solution_bringup.launch.py").read_text(encoding="utf-8")
    navigation = (LAUNCH_DIR / "navigation.launch.py").read_text(encoding="utf-8")
    assert '"odom_topic": topic_odom' in solution
    assert 'str(profil["odom_topic"])' in solution
    assert 'odometrie = {"odom_topic": topic_odom}' in navigation
    assert navigation.count("temps_simulation, odometrie]") == 2
    assert "parametres_communs + [odometrie, {" in navigation
