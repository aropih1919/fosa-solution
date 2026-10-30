"""Interface terminal et lancement d'une expérience Fosa reproducible."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter

from caytu_nav_solution.benchmark_config import (
    CONTROLLERS,
    PROFILES,
    ConfigurationError,
    build_experiment_config,
    localization_availability,
    package_is_installed,
    validate_selection,
    write_experiment_config,
)
from caytu_nav_solution.benchmark_metrics import BenchmarkMetrics
from caytu_nav_solution.benchmark_report import (
    make_metadata,
    new_run_directory,
    print_run_report,
    write_report,
)
from caytu_nav_solution.task_params import load_task_params


CONTROLLER_PACKAGES = {
    "rpp": "nav2_regulated_pure_pursuit_controller",
    "dwb": "nav2_dwb_controller",
    "mppi": "nav2_mppi_controller",
}
LOCALIZATION_LABELS = {
    "odom_imu": "ODOM_IMU — roues + cap IMU",
    "odom_only": "ODOM_ONLY — odométrie sans cap IMU (diagnostic)",
    "amcl": "AMCL — lidar filtré + carte statique",
    "amcl_ekf": "AMCL_EKF — indisponible tant que le conflit TF n'est pas levé",
}
PROFILE_LABELS = {
    "ultra_safe": "Très prudent — vitesse minimale, marge renforcée",
    "safe": "Prudent — vitesse réduite, marge renforcée",
    "balanced": "Équilibré — configuration de référence",
    "fast": "Rapide — vitesse accrue",
    "ultra_fast": "Très rapide — essai agressif en simulation",
    "tight_clearance": "Marge minimale — test de proximité en simulation",
}


def _scenario_registry():
    config_path = Path(get_package_share_directory("caytu_nav_bringup")) / "config" / "scenarios.yaml"
    with config_path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)["scenarios"]


def _controller_availability():
    return {
        name: package_is_installed(package)
        for name, package in CONTROLLER_PACKAGES.items()
    }


def _prompt_choice(title, choices, labels, available=None):
    available = available or {key: True for key in choices}
    print(f"\n{title}")
    for number, key in enumerate(choices, 1):
        suffix = "" if available.get(key, False) else " — INDISPONIBLE"
        reason = ""
        if not available.get(key, False):
            reason = f" ({labels.get(key, '')})"
        print(f"[{number}] {labels.get(key, key)}{suffix}{reason}")
    while True:
        answer = input("Choix : ").strip()
        if answer.isdigit() and 1 <= int(answer) <= len(choices):
            key = choices[int(answer) - 1]
            if available.get(key, False):
                return key
            print("Cette option ne peut pas être lancée dans cet environnement.")
        else:
            print(f"Entrez un nombre entre 1 et {len(choices)}.")


def _stop_process(process, timeout=12.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=4.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def _print_monitor(metrics, phase):
    """Réactualise l'état compact sur la ligne courante du terminal."""
    print(f"\r{metrics.live_status(phase)}", end="", flush=True)


class BenchmarkRunner(Node):
    def __init__(self):
        super().__init__("fosa_benchmark")
        # rclpy declares use_sim_time itself with default False. For this lab,
        # simulation is the default; preserve an explicit CLI override.
        if "use_sim_time" not in self._parameter_overrides:
            self.set_parameters([Parameter("use_sim_time", value=True)])
        self.declare_parameter("controller", "")
        self.declare_parameter("localization", "")
        self.declare_parameter("profile", "")
        self.declare_parameter("scenario", "full_course")
        self.declare_parameter("environment", "simulation")
        self.declare_parameter("results_dir", str(Path.cwd() / "results"))
        self.declare_parameter("task_params_file", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("start_simulation", True)
        self.declare_parameter("sensor_startup_timeout_sec", 180.0)

    def select(self):
        values = {
            "controller": str(self.get_parameter("controller").value).lower(),
            "localization": str(self.get_parameter("localization").value).lower(),
            "profile": str(self.get_parameter("profile").value).lower(),
            "scenario": str(self.get_parameter("scenario").value).lower(),
            "environment": str(self.get_parameter("environment").value).lower(),
        }
        interactive = any(not values[name] for name in
                          ("controller", "localization", "profile"))
        self._interactive_selection = interactive

        controller_available = _controller_availability()
        localization_available = localization_availability()
        if not values["controller"]:
            labels = {"rpp": "RPP", "dwb": "DWB", "mppi": "MPPI"}
            values["controller"] = _prompt_choice(
                "1. CONTROLLER", CONTROLLERS, labels, controller_available)
        if not values["localization"]:
            labels = {key: LOCALIZATION_LABELS[key] for key in LOCALIZATION_LABELS}
            values["localization"] = _prompt_choice(
                "2. LOCALIZATION", tuple(localization_available), labels,
                {key: item["available"] for key, item in localization_available.items()},
            )
        if not values["profile"]:
            values["profile"] = _prompt_choice(
                "3. PROFIL", PROFILES, PROFILE_LABELS)

        scenarios = _scenario_registry()
        if interactive:
            scenario_keys = tuple(scenarios)
            scenario_labels = {
                key: ("Full course — official task" if key == "full_course"
                      else f"{key} — {item['reason']}")
                for key, item in scenarios.items()
            }
            values["scenario"] = _prompt_choice(
                "4. SCENARIO", scenario_keys, scenario_labels,
                {key: item["available"] for key, item in scenarios.items()},
            )
        elif values["scenario"] not in scenarios:
            raise ConfigurationError(f"Scénario inconnu : {values['scenario']}")
        elif not scenarios[values["scenario"]]["available"]:
            raise ConfigurationError(scenarios[values["scenario"]]["reason"])

        validate_selection(**values)
        return values

    def run(self):
        selection = self.select()
        print("\n" + "=" * 48)
        print("FOSA NAVIGATION LAB — CONFIGURATION")
        print("=" * 48)
        print(f"Controller   : {selection['controller'].upper()}")
        print(f"Localization : {selection['localization'].upper()}")
        print(f"Profil       : {PROFILE_LABELS[selection['profile']]}")
        print(f"Scenario     : {selection['scenario']}")
        print("=" * 48)
        if sys.stdin.isatty() and self._interactive_selection:
            answer = input("[1] START EXPERIMENT    [2] CANCEL\nChoix : ").strip()
            if answer != "1":
                print("Expérience annulée.")
                return 0

        task_file = str(self.get_parameter("task_params_file").value)
        if not task_file and not package_is_installed("parc_robot_bringup"):
            raise ConfigurationError(
                "Le package parc_robot_bringup est absent et aucun task_params_file "
                "n'a été fourni. Installez le package officiel ou passez "
                "-p task_params_file:=/chemin/task_params.yaml.")
        task = load_task_params(task_file or None)
        bringup_share = Path(get_package_share_directory("caytu_nav_bringup"))
        map_file = str(self.get_parameter("map_yaml").value)
        if not map_file:
            map_file = str(bringup_share / "maps" / "cafe_map.yaml")
        use_sim_time = bool(self.get_parameter("use_sim_time").value)

        config = build_experiment_config(
            selection["controller"],
            selection["localization"],
            selection["profile"],
            task.spawn,
            use_sim_time=use_sim_time,
        )
        run_id, run_dir = new_run_directory(self.get_parameter("results_dir").value)
        config_path = write_experiment_config(config, run_dir / "config_used.yaml")
        metadata = make_metadata(
            run_id,
            selection,
            map_file,
            task,
            config_path,
            Path.cwd(),
            use_sim_time=use_sim_time,
        )
        (run_dir / "metadata.json").write_text(
            __import__("json").dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

        metrics = BenchmarkMetrics(
            selection["localization"], task.goal_x, task.goal_y, use_sim_time)
        metrics.mark_experiment_start()
        simulation_process = None
        task_process = None
        simulation_log = (run_dir / "simulation.log").open("w", encoding="utf-8")
        task_log = (run_dir / "task_solution.log").open("w", encoding="utf-8")
        mission_success = False
        exit_code = None
        error_message = None
        monitor_active = False
        try:
            if self.get_parameter("start_simulation").value:
                if not package_is_installed("parc_robot_bringup"):
                    raise ConfigurationError(
                        "Le package parc_robot_bringup n'est pas installé : le monde "
                        "officiel full_course ne peut pas être lancé.")
                simulation_process = subprocess.Popen(
                    ["ros2", "launch", "parc_robot_bringup", "task.launch.py"],
                    stdout=simulation_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )

            self.get_logger().info(f"Run {run_id}: attente de /odom et /scan...")
            timeout = float(self.get_parameter("sensor_startup_timeout_sec").value)
            deadline = time.monotonic() + timeout
            last_resource_sample = 0.0
            last_monitor_output = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(metrics, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_resource_sample >= 1.0:
                    metrics.sample_resources()
                    metrics.check_tf()
                    last_resource_sample = now
                if now - last_monitor_output >= 1.0:
                    _print_monitor(metrics, "préparation")
                    monitor_active = True
                    last_monitor_output = now
                if metrics.odom_pose is not None and "/scan" in metrics.scan_topics_received:
                    break
                if simulation_process is not None and simulation_process.poll() is not None:
                    raise RuntimeError(
                        "Le launch du monde officiel s'est arrêté avant l'arrivée des capteurs. "
                        f"Consultez {run_dir / 'simulation.log'}.")
            else:
                raise TimeoutError(
                    f"Pas de /odom et /scan après {timeout:.0f} s. "
                    f"Consultez {run_dir / 'simulation.log'}.")

            metrics.mark_mission_start()
            if monitor_active:
                print(flush=True)
                monitor_active = False
            task_command = [
                "ros2", "run", "caytu_nav_solution", "task_solution.py", "--ros-args",
                "-p", f"controller:={selection['controller']}",
                "-p", f"localization:={selection['localization']}",
                "-p", f"profile:={selection['profile']}",
                "-p", f"nav2_params_file:={config_path}",
                "-p", f"task_params_file:={task.path}",
                "-p", f"map_yaml:={map_file}",
                "-p", "static_map:=auto",
                "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            ]
            task_process = subprocess.Popen(
                task_command,
                stdout=task_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.get_logger().info(f"Mission démarrée; logs: {run_dir / 'task_solution.log'}")
            last_resource_sample = 0.0
            last_monitor_output = 0.0
            while rclpy.ok() and task_process.poll() is None:
                rclpy.spin_once(metrics, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_resource_sample >= 1.0:
                    metrics.sample_resources()
                    metrics.check_tf()
                    last_resource_sample = now
                if now - last_monitor_output >= 1.0:
                    _print_monitor(metrics, "navigation")
                    monitor_active = True
                    last_monitor_output = now
            exit_code = task_process.poll()
            mission_success = exit_code == 0
            if not mission_success:
                error_message = f"task_solution s'est terminé avec le code {exit_code}."
        except KeyboardInterrupt:
            error_message = "Interruption utilisateur."
            self.get_logger().warn(error_message)
            _stop_process(task_process)
            exit_code = task_process.poll() if task_process else None
        except Exception as error:  # Preserve a report for failed experiments too.
            error_message = str(error)
            self.get_logger().error(error_message)
            exit_code = task_process.poll() if task_process else None
        finally:
            _stop_process(task_process)
            _stop_process(simulation_process, timeout=30.0)
            metrics.mark_finished()
            if monitor_active:
                print(flush=True)
            simulation_log.close()
            task_log.close()
            summary_metrics = metrics.summary(mission_success, exit_code, error_message)
            summary = write_report(run_dir, metadata, summary_metrics, metrics.sample_rows)
            metrics.destroy_node()
            print_run_report(summary)
            self.get_logger().info(f"Résultats enregistrés dans {run_dir}")
        return 0 if mission_success else 1


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkRunner()
    code = 1
    try:
        code = node.run()
    except (ConfigurationError, RuntimeError, OSError, ValueError) as error:
        node.get_logger().error(str(error))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
