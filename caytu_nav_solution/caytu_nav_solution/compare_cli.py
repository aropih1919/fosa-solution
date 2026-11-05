"""Affiche une comparaison sans score pondéré ni classement de vitesse seul."""

import argparse
import json
import math
from pathlib import Path


def _sort_key(run):
    metrics = run["metrics"]
    collisions = metrics.get("collision_count")
    runtime = metrics.get("execution_time_sec")
    distance = metrics.get("travelled_distance_m")
    return (
        collisions is None,
        collisions if collisions is not None else math.inf,
        not bool(metrics.get("mission_success")),
        runtime if runtime is not None else math.inf,
        distance if distance is not None else math.inf,
    )


def load_runs(results_dir, controller=None, localization=None, profile=None, scenario=None):
    runs = []
    for path in sorted(Path(results_dir).expanduser().glob("FOSA-*/summary.json")):
        try:
            run = json.loads(path.read_text(encoding="utf-8"))
            metadata = run["metadata"]
            if controller and metadata.get("controller") != controller:
                continue
            if localization and metadata.get("localization") != localization:
                continue
            if profile and metadata.get("profile") != profile:
                continue
            if scenario and metadata.get("scenario") != scenario:
                continue
            run["_path"] = str(path.parent)
            runs.append(run)
        except (OSError, json.JSONDecodeError, KeyError, TypeError):
            continue
    return sorted(runs, key=_sort_key)


def _cell(value, suffix=""):
    if value is None:
        return "N/A"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.2f}{suffix}"
    return f"{value}{suffix}"


def print_comparison(runs):
    headers = ("Run", "Controller", "Localization", "Profile", "Scenario",
               "Success", "Collisions", "Time", "Distance")
    rows = []
    for run in runs:
        meta, metrics = run["metadata"], run["metrics"]
        rows.append((
            meta.get("run_id", "N/A"),
            meta.get("controller", "N/A"),
            meta.get("localization", "N/A"),
            meta.get("profile_label", meta.get("profile", "N/A")),
            meta.get("scenario", "N/A"),
            _cell(metrics.get("mission_success")),
            _cell(metrics.get("collision_count")),
            _cell(metrics.get("execution_time_sec"), " s"),
            _cell(metrics.get("travelled_distance_m"), " m"),
        ))
    widths = [max([len(headers[i]), *(len(row[i]) for row in rows)])
              for i in range(len(headers))]
    print("  ".join(headers[i].ljust(widths[i]) for i in range(len(headers))))
    print("  ".join("-" * width for width in widths))
    for row in rows:
        print("  ".join(row[i].ljust(widths[i]) for i in range(len(headers))))
    print("\nTri : d'abord collisions connues, puis réussite, temps et distance. Aucun score pondéré.")
    if not rows:
        print("Aucune expérience ne correspond aux filtres.")


def main(args=None):
    parser = argparse.ArgumentParser(description="Compare les expériences Fosa Navigation Lab.")
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--controller", choices=("rpp", "dwb", "mppi"))
    parser.add_argument("--localization", choices=("odom_imu", "odom_only", "amcl", "amcl_ekf"))
    parser.add_argument(
        "--profile", choices=(
            "ultra_safe", "safe", "balanced", "fast", "ultra_fast", "tight_clearance"))
    parser.add_argument("--scenario")
    options = parser.parse_args(args=args)
    runs = load_runs(
        options.results_dir,
        controller=options.controller,
        localization=options.localization,
        profile=options.profile,
        scenario=options.scenario,
    )
    print_comparison(runs)
    return 0
