#!/usr/bin/env python3
"""Essais en serie : outil, jamais lance par la solution.

Pour chaque essai, dans cet ordre exact :
1. task_params.yaml officiel sauvegarde en .bak puis reecrit selon le scenario.
2. ros2 launch parc_robot_bringup task.launch.py (nouveau groupe de session).
3. Attente /clock puis /odom (180 s max, sinon sim_failed).
4. Boites SDF statiques via ros_gz_sim create.
5. ros2 run caytu_nav_solution task_solution.py avec report_dir dedie.
6. gz model -m sitoe_robot -p pour la distance reelle au but.
7. SIGINT au groupe simu, 15 s, puis SIGKILL, controle pgrep -f "gz sim".
8. Lecture du JSON de rapport et ajout aux resultats.
"""

import argparse
import csv
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

WORLD_Z = 0.213


def official_params_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory("parc_robot_bringup"), "config", "task_params.yaml")


def read_official(path: str):
    import yaml
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)["/**"]["ros__parameters"]
    return (float(data["x"]), float(data["y"]), float(data["yaw"]),
            float(data["goal_x"]), float(data["goal_y"]))


def resolve_scenario(name: str, spec: dict, official):
    ox, oy, oyaw, ogx, ogy = official
    if spec.get("use_official"):
        spawn, goal = (ox, oy, oyaw), (ogx, ogy)
    elif spec.get("swap_official"):
        spawn, goal = (ogx, ogy, float(spec.get("yaw", oyaw))), (ox, oy)
    else:
        sx, sy, syaw = spec["spawn"]
        gx, gy = spec["goal"]
        spawn, goal = (float(sx), float(sy), float(syaw)), (float(gx), float(gy))
    return spawn, goal, spec.get("boxes", [])


def write_params(path: str, spawn, goal) -> None:
    import yaml
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    params = data["/**"]["ros__parameters"]
    params["x"], params["y"], params["yaw"] = spawn
    params["goal_x"], params["goal_y"] = goal
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream)


def wait_topic(topic: str, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        left = end - time.monotonic()
        try:
            done = subprocess.run(
                ["ros2", "topic", "echo", topic, "--once"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=min(10.0, left))
            if done.returncode == 0:
                return True
        except subprocess.TimeoutExpired:
            continue
    return False


def box_sdf(width: float, depth: float, height: float) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.9"><model name="box"><static>true</static>
<link name="link"><collision name="c"><geometry><box><size>{width} {depth} {height}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{width} {depth} {height}</size></box></geometry></visual>
</link></model></sdf>
"""


def real_distance(goal) -> object:
    try:
        out = subprocess.run(
            ["gz", "model", "-m", "sitoe_robot", "-p"],
            capture_output=True, text=True, timeout=10.0)
        x = y = None
        for line in out.stdout.splitlines():
            parts = line.strip().split()
            if len(parts) >= 3 and parts[0] in ("x:", "y:"):
                pass
        nums = []
        for token in out.stdout.replace(":", " ").split():
            try:
                nums.append(float(token))
            except ValueError:
                continue
        if len(nums) >= 2:
            x, y = nums[0], nums[1]
            return math.hypot(goal[0] - x, goal[1] - y)
    except Exception:
        pass
    return None


def run_trial(scenario: str, spawn, goal, boxes, out_dir: str, index: int) -> dict:
    trial_dir = os.path.join(out_dir, f"{scenario}_{index:02d}")
    os.makedirs(trial_dir, exist_ok=True)
    params_path = official_params_path()
    backup = params_path + ".bak"
    shutil.copy(params_path, backup)
    sim = None

    def restore() -> None:
        if os.path.exists(backup):
            shutil.move(backup, params_path)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: restore())
    try:
        # 1. Scenario dans task_params officiel.
        if not (len(spawn) == 3 and len(goal) == 2):
            pass
        write_params(params_path, spawn, goal)
        # 2. Simulation dans son groupe de session.
        sim = subprocess.Popen(
            ["ros2", "launch", "parc_robot_bringup", "task.launch.py"],
            start_new_session=True)
        # 3. Horloge puis odometrie.
        if not wait_topic("/clock", 180.0) or not wait_topic("/odom", 180.0):
            return {"scenario": scenario, "result": "sim_failed"}
        # 4. Boites statiques.
        with tempfile.TemporaryDirectory(prefix="fosa_boxes_") as tmp:
            for k, (bx, by, w, d, h) in enumerate(boxes):
                sdf_path = os.path.join(tmp, f"box_{k}.sdf")
                with open(sdf_path, "w", encoding="utf-8") as stream:
                    stream.write(box_sdf(w, d, h))
                subprocess.run(
                    ["ros2", "run", "ros_gz_sim", "create", "-file", sdf_path,
                     "-name", f"box_{k}", "-x", str(bx), "-y", str(by),
                     "-z", str(WORLD_Z + h / 2.0)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(1.0)
            # 5. Solution officielle avec son dossier de rapport.
            done = subprocess.run(
                ["ros2", "run", "caytu_nav_solution", "task_solution.py",
                 "--ros-args", "-p", f"report_dir:={trial_dir}"])
            code = done.returncode
        # 6. Pose reelle Gazebo, seuls outil et moniteur y ont droit.
        real_d = real_distance(goal)
        # 7. Arret propre de la simulation.
        try:
            os.killpg(os.getpgid(sim.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            sim.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        subprocess.run(["pgrep", "-f", "gz sim"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # 8. Rapport JSON de l'essai.
        found = sorted(glob.glob(os.path.join(trial_dir, "run_*.json")))
        report = {}
        if found:
            with open(found[-1], encoding="utf-8") as stream:
                report = json.load(stream)
        return {
            "scenario": scenario,
            "result": report.get("result", f"exit_{code}"),
            "sim_duration": report.get("sim_duration_s", ""),
            "final_est": report.get("final_distance_m", ""),
            "final_real": real_d if real_d is not None else "",
            "contacts": report.get("contacts", 0),
        }
    finally:
        restore()
        if sim is not None and sim.poll() is None:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--scenarios", default="")
    parser.add_argument("--out", default="benchmark_results")
    args = parser.parse_args()
    import yaml
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "benchmark_scenarios.yaml"), encoding="utf-8") as stream:
        all_specs = yaml.safe_load(stream)["scenarios"]
    wanted = args.scenarios.split(",") if args.scenarios else list(all_specs)
    wanted = [w for w in wanted if w in all_specs]
    os.makedirs(args.out, exist_ok=True)
    official = read_official(official_params_path())
    rows = []
    for name in wanted:
        for i in range(args.runs):
            spawn, goal, boxes = resolve_scenario(name, all_specs[name], official)
            rows.append(run_trial(name, spawn, goal, boxes, args.out, i))
    csv_path = os.path.join(args.out, "benchmark_results.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["scenario", "result", "sim_duration",
                                "final_est", "final_real", "contacts"])
        writer.writeheader()
        writer.writerows(rows)
    # Agregats en une passe, sans tri.
    agg = {}
    for row in rows:
        entry = agg.setdefault(row["scenario"], {
            "n": 0, "ok": 0, "sum_dur": 0.0, "n_dur": 0,
            "min_dur": None, "max_dur": None,
            "sum_est": 0.0, "n_est": 0, "sum_real": 0.0, "n_real": 0,
            "contacts": 0})
        entry["n"] += 1
        if str(row["result"]) == "success":
            entry["ok"] += 1
        try:
            dur = float(row["sim_duration"])
            entry["sum_dur"] += dur
            entry["n_dur"] += 1
            entry["min_dur"] = dur if entry["min_dur"] is None else min(entry["min_dur"], dur)
            entry["max_dur"] = dur if entry["max_dur"] is None else max(entry["max_dur"], dur)
        except (TypeError, ValueError):
            pass
        for key, out in (("final_est", "est"), ("final_real", "real")):
            try:
                val = float(row[key])
                entry[f"sum_{out}"] += val
                entry[f"n_{out}"] += 1
            except (TypeError, ValueError):
                pass
        try:
            entry["contacts"] += int(row["contacts"])
        except (TypeError, ValueError):
            pass
    md_path = os.path.join(args.out, "benchmark_results.md")
    with open(md_path, "w", encoding="utf-8") as stream:
        for name, e in agg.items():
            avg_dur = e["sum_dur"] / e["n_dur"] if e["n_dur"] else 0.0
            avg_est = e["sum_est"] / e["n_est"] if e["n_est"] else 0.0
            avg_real = e["sum_real"] / e["n_real"] if e["n_real"] else 0.0
            stream.write(f"## {name}\n")
            stream.write(f"reussites: {e['ok']}/{e['n']}\n")
            stream.write(f"duree sim: moy {avg_dur:.1f} min {e['min_dur']} max {e['max_dur']}\n")
            stream.write(f"distance estimee moy: {avg_est:.2f} reelle moy: {avg_real:.2f}\n")
            stream.write(f"contacts: {e['contacts']}\n\n")


if __name__ == "__main__":
    main()
