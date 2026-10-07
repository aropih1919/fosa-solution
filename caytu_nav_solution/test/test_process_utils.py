"""Tests de la recherche de lanceurs avec un faux /proc."""
import os

from caytu_nav_solution.process_utils import find_stale_bringups


def _write_cmdline(root, pid, text):
    folder = os.path.join(root, str(pid))
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "cmdline"), "wb") as stream:
        stream.write(text)


def test_matching_launcher_found(tmp_path):
    root = str(tmp_path)
    _write_cmdline(root, "123",
                   b"python3\0/opt/ros/jazzy/bin/ros2\0launch\0caytu_nav_bringup\0"
                   b"solution_bringup.launch.py\0map:=x")
    assert find_stale_bringups(
        "caytu_nav_bringup", "solution_bringup.launch.py", 999, root) == [123]


def test_own_pid_ignored(tmp_path):
    root = str(tmp_path)
    _write_cmdline(root, "123",
                   b"python3\0/opt/ros/jazzy/bin/ros2\0launch\0caytu_nav_bringup\0"
                   b"solution_bringup.launch.py\0map:=x")
    assert find_stale_bringups(
        "caytu_nav_bringup", "solution_bringup.launch.py", 123, root) == []


def test_other_launcher_ignored(tmp_path):
    root = str(tmp_path)
    _write_cmdline(root, "456", b"ros2\0launch\0other_pkg\0other.launch.py\0")
    assert find_stale_bringups(
        "caytu_nav_bringup", "solution_bringup.launch.py", 999, root) == []


def test_non_numeric_and_missing_cmdline_ignored(tmp_path):
    root = str(tmp_path)
    os.makedirs(os.path.join(root, "self"), exist_ok=True)
    os.makedirs(os.path.join(root, "789"), exist_ok=True)
    assert find_stale_bringups(
        "caytu_nav_bringup", "solution_bringup.launch.py", 999, root) == []


def test_missing_proc_root_returns_empty():
    assert find_stale_bringups(
        "caytu_nav_bringup", "solution_bringup.launch.py", 999,
        "/proc-inexistant-fosa") == []
