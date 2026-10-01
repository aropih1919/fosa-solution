#!/usr/bin/env python3
"""Vérifie que l'intégration caméra est cohérente dans caytu_nav_bringup.
Usage :  python3 check_camera_integration.py ~/ros2_ws/src/solutions
Reproduit l'ordre de chargement de navigation.launch.py (le dernier fichier gagne)."""
import fnmatch, sys
from pathlib import Path
import yaml

root = Path(sys.argv[1] if len(sys.argv) > 1 else '.').expanduser()
cfg = root / 'caytu_nav_bringup' / 'config'
ok_all = True

def check(cond, msg, detail=''):
    global ok_all
    ok_all &= bool(cond)
    print(('  OK   ' if cond else '  ECHEC ') + msg + (f'  -> {detail}' if detail and not cond else ''))

def load(name):
    with (cfg / name).open(encoding='utf-8') as f:
        return yaml.safe_load(f)

def params_for(node, files):
    """Fusion façon ROS 2 : clés '/**/x' ou 'x:'/'x:' imbriquées, dernier fichier gagnant (fusion profonde)."""
    out = {}
    def merge(dst, src):
        for k, v in src.items():
            if isinstance(v, dict) and isinstance(dst.get(k), dict):
                merge(dst[k], v)
            else:
                dst[k] = v
    ns, name = node.strip('/').split('/')
    for fn in files:
        d = load(fn)
        for key, val in d.items():
            keyfull = key
            hit = None
            if key.startswith('/') and fnmatch.fnmatch(node, key.replace('/**', '/*') if key.startswith('/**') else key):
                hit = val
            elif key == ns and isinstance(val, dict) and name in val:
                hit = val[name]
            elif key.startswith('/**/') and node.endswith(key[3:]):
                hit = val
            if hit and 'ros__parameters' in hit:
                merge(out, hit['ros__parameters'])
    return out

print('1) YAML valides')
for f in sorted(cfg.glob('*.yaml')):
    try: yaml.safe_load(f.read_text(encoding='utf-8')); check(True, f.name)
    except Exception as e: check(False, f.name, str(e)[:120])

# ordre de chargement de navigation.launch.py
ctrl_files = ['nav2_params.yaml', 'costmap_common_params.yaml', 'local_costmap_params.yaml', 'controller_server_params.yaml']
plan_files = ['nav2_params.yaml', 'costmap_common_params.yaml', 'global_costmap_params.yaml', 'planner_server_params.yaml']

for label, node, files in (('global', '/global_costmap/global_costmap', plan_files),
                           ('local', '/local_costmap/local_costmap', ctrl_files)):
    print(f'\n2) costmap {label} (paramètres effectivement reçus par {node})')
    p = params_for(node, files)
    plugins = p.get('plugins', [])
    check('camera_layer' in plugins, f'camera_layer dans plugins {plugins}')
    check(plugins.index('camera_layer') < plugins.index('inflation_layer') if 'camera_layer' in plugins and 'inflation_layer' in plugins else False,
          'camera_layer AVANT inflation_layer')
    cl = p.get('camera_layer') or {}
    check(cl.get('plugin') == 'nav2_costmap_2d::ObstacleLayer', 'camera_layer.plugin = ObstacleLayer', cl.get('plugin'))
    check(cl.get('observation_sources') == 'camera_scan', 'observation_sources = camera_scan', cl.get('observation_sources'))
    s = cl.get('camera_scan') or {}
    check(s.get('topic') == '/top_camera/obstacles_scan', 'topic = /top_camera/obstacles_scan', s.get('topic'))
    check(s.get('sensor_frame') == 'top_camera_link', 'sensor_frame = top_camera_link', s.get('sensor_frame'))
    check(s.get('data_type') == 'LaserScan', 'data_type = LaserScan', s.get('data_type'))
    check(s.get('inf_is_valid') is True, 'inf_is_valid = true (sinon impossible d\'effacer)', s.get('inf_is_valid'))
    check(s.get('marking') is True and s.get('clearing') is True, 'marking et clearing actifs')
    check(isinstance(s.get('obstacle_max_range'), (int, float)) and s['obstacle_max_range'] < 4.0,
          'obstacle_max_range < range_max du scan (4.0)', s.get('obstacle_max_range'))
    check(isinstance(s.get('raytrace_min_range'), (int, float)) and s['raytrace_min_range'] >= 1.0,
          'raytrace_min_range >= 1.0 (angle mort proche)', s.get('raytrace_min_range'))
    ol = p.get('obstacle_layer', {})
    check(ol.get('observation_sources') == 'scan' and (ol.get('scan') or {}).get('topic') == '/scan_filtered',
          'obstacle_layer LiDAR inchangée (/scan_filtered)')

print('\n3) launch et packages')
bl = (root / 'caytu_nav_bringup/launch/solution_bringup.launch.py').read_text(encoding='utf-8')
check('perception.launch.py' in bl, 'solution_bringup inclut perception.launch.py')
import re
m = re.search(r'LaunchDescription\(\s*\[(.*?)\]\s*\)\s*$', bl, re.S)
lst = m.group(1) if m else ''
inc = [n for n in ('perception', 'delayed_perception') if re.search(r'\b' + n + r'\b', lst)]
check(bool(inc), 'perception (ou delayed_perception) figure dans la LaunchDescription', 'défini mais jamais ajouté à la liste -> ne démarrera pas')
check((root / 'caytu_nav_bringup/launch/perception.launch.py').exists(), 'perception.launch.py présent')
pkg = (root / 'caytu_nav_bringup/package.xml').read_text(encoding='utf-8')
check('caytu_camera_perception' in pkg, 'package.xml bringup dépend de caytu_camera_perception')
check('ros_gz_bridge' in pkg, 'package.xml bringup dépend de ros_gz_bridge')
cp = root / 'caytu_camera_perception'
for rel in ('package.xml', 'setup.py', 'setup.cfg', 'resource/caytu_camera_perception', 'config/camera_obstacles.yaml',
            'caytu_camera_perception/scan_projection.py', 'caytu_camera_perception/camera_obstacle_node.py',
            'caytu_camera_perception/camera_perception_watchdog.py'):
    check((cp / rel).exists(), f'caytu_camera_perception/{rel}')
ts = (root / 'caytu_nav_solution/caytu_nav_solution/task_solution.py').read_text(encoding='utf-8')
check('/camera_perception_ready' in ts, 'task_solution.py écoute /camera_perception_ready (patch des 2 signaux)')

print('\nRESULTAT :', 'TOUT EST COHERENT' if ok_all else 'DES POINTS A CORRIGER (lignes ECHEC)')
sys.exit(0 if ok_all else 1)
