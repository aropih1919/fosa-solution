"""Identifiants et fichiers de résultat reproductibles pour une expérience."""

import csv
import hashlib
import json
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path


PROFILE_LABELS = {
    "ultra_safe": "Très prudent",
    "safe": "Prudent",
    "balanced": "Équilibré",
    "fast": "Rapide",
    "ultra_fast": "Très rapide",
    "tight_clearance": "Marge minimale (simulation)",
}


def new_run_directory(results_dir, year=None):
    results_dir = Path(results_dir).expanduser().resolve()
    year = int(year or datetime.now(timezone.utc).year)
    results_dir.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(rf"^FOSA-{year}-(\d+)$")
    existing = [int(match.group(1)) for path in results_dir.iterdir()
                if path.is_dir() and (match := pattern.match(path.name))]
    number = max(existing, default=0) + 1
    while True:
        run_id = f"FOSA-{year}-{number:03d}"
        run_dir = results_dir / run_id
        try:
            run_dir.mkdir()
            return run_id, run_dir
        except FileExistsError:
            number += 1


def _git_value(project_root, *args):
    try:
        result = subprocess.run(
            ["git", "-C", str(project_root), *args],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return result.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def _package_version(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory
        package_xml = Path(get_package_share_directory(package_name)) / "package.xml"
        root = ET.parse(package_xml).getroot()
        version = root.findtext("version")
        return version.strip() if version else None
    except (ImportError, LookupError, OSError, ET.ParseError):
        return None


def make_metadata(run_id, selection, map_path, task_params, config_path,
                  project_root, use_sim_time=True):
    config_bytes = Path(config_path).read_bytes()
    return {
        "run_id": run_id,
        "date_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": _git_value(project_root, "rev-parse", "HEAD"),
        "git_branch": _git_value(project_root, "branch", "--show-current"),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "nav2_version": _package_version("nav2_bringup"),
        "environment": selection["environment"],
        "scenario": selection["scenario"],
        "controller": selection["controller"],
        "localization": selection["localization"],
        "profile": selection["profile"],
        "profile_label": PROFILE_LABELS.get(selection["profile"], selection["profile"]),
        "map": str(Path(map_path).resolve()),
        "map_id": Path(map_path).stem,
        "task_params_file": task_params.path,
        "initial_pose": {
            "frame": "map",
            "x": float(task_params.spawn.x),
            "y": float(task_params.spawn.y),
            "yaw": float(task_params.spawn.yaw),
        },
        "goal": {"frame": "map", "x": float(task_params.goal_x),
                 "y": float(task_params.goal_y)},
        "use_sim_time": bool(use_sim_time),
        "config_file": "config_used.yaml",
        "results_directory": str(Path(config_path).parent.resolve()),
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "collision_metric_source": "ros_gz_interfaces/Contacts; null if no topic messages",
        "distance_metric_source": "Gazebo /sitoe_robot/pose when received, otherwise /odom",
        "real_robot_validation": False,
    }


def write_report(run_dir, metadata, metrics, samples):
    run_dir = Path(run_dir)
    summary = {"run_id": metadata["run_id"], "metadata": metadata, "metrics": metrics}
    (run_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    columns = [
        "time_sec", "truth_x", "truth_y", "truth_yaw", "estimate_x", "estimate_y",
        "estimate_yaw", "odom_x", "odom_y", "minimum_obstacle_distance_m",
    ]
    with (run_dir / "metrics.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(samples)
    return summary


def print_run_report(summary):
    metadata = summary["metadata"]
    metrics = summary["metrics"]

    def show(value, suffix=""):
        if value is None:
            return "N/A"
        if isinstance(value, bool):
            return "oui" if value else "non"
        if isinstance(value, float):
            return f"{value:.3f}{suffix}"
        return f"{value}{suffix}"

    lines = [
        "=" * 48,
        "FOSA NAVIGATION LAB — RAPPORT",
        "=" * 48,
        f"Run ID          : {metadata['run_id']}",
        f"Date UTC        : {metadata['date_utc']}",
        f"Git commit      : {show(metadata.get('git_commit'))}",
        f"Environment     : {show(metadata.get('environment'))}",
        f"Scenario        : {show(metadata.get('scenario'))}",
        f"Controller      : {show(metadata.get('controller'))}",
        f"Localization    : {show(metadata.get('localization'))}",
        f"Profil          : {show(metadata.get('profile_label', metadata.get('profile')))}",
        "-" * 48,
        f"Mission success : {show(metrics.get('mission_success'))}",
        f"Collisions      : {show(metrics.get('collision_count'))}",
        f"Execution time  : {show(metrics.get('execution_time_sec'), ' s')}",
        f"Travelled dist. : {show(metrics.get('travelled_distance_m'), ' m')}",
        f"Final goal dist.: {show(metrics.get('final_goal_distance_m'), ' m')}",
        f"Min obstacle    : {show(metrics.get('minimum_obstacle_distance_m'), ' m')}",
        f"Localization init: {show(metrics.get('localization_initialization_time_sec'), ' s')}",
        f"Position RMSE   : {show(metrics.get('position_rmse_m'), ' m')}",
        f"Controller rate : {show(metrics.get('controller_rate_hz'), ' Hz')}",
        f"Results         : {show(metadata.get('results_directory'))}",
        "=" * 48,
    ]
    print("\n".join(lines))
