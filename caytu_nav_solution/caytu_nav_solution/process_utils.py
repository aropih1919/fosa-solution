"""Recherche des lanceurs de la solution restés en vie (sans dépendance ROS)."""

import os
from typing import List


def find_stale_bringups(package: str, launch_file: str, own_pid: int,
                        proc_root: str = '/proc') -> List[int]:
    """PID des processus `ros2 launch <package> <launch_file>` encore présents.

    Un seul parcours de proc_root : O(nombre de processus).
    """
    found = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit() or int(entry) == own_pid:
            continue
        try:
            with open(os.path.join(proc_root, entry, 'cmdline'), 'rb') as stream:
                args = stream.read().split(b'\0')
        except OSError:
            continue
        words = [a.decode('utf-8', 'replace') for a in args if a]
        if 'launch' in words and package in words and launch_file in words:
            found.append(int(entry))
    return sorted(found)
