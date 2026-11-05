#!/usr/bin/env python3
"""Diagnostic de course Fosa : pourquoi Nav2 part en recovery, ou le robot ne bouge pas.

À lancer dans un 2e terminal pendant que course.launch.py tourne, AVANT
`ros2 run caytu_nav_solution depart` (même environnement : setup.bash, env.sh) :

    ros2 run caytu_nav_solution diagnostic          # version déployée (deploy.sh)
    python3 diagnostic_course.py                    # ou le fichier seul, sans recompiler
    options : --duree 600  --periode 5  --bag  --sortie DOSSIER  --temps auto|sim|reel

Ctrl-C arrête et affiche le BILAN (« PISTES »). Le script ne fait qu'ÉCOUTER :
aucune publication (ni /parc/*, ni odométrie, ni commande), pas même /rosout.

Ce qu'il suit, étage par étage :
  commandes     Nav2 (/fosa/cmd_vel_brut) -> notre garde-fou -> /cmd_vel ->
                sécurité des organisateurs -> /botwheel_explorer/cmd_vel -> odométrie ;
  garde-fou     état de nav2_collision_monitor, points du lidar et des caméras
                dans ses zones et dans l'emprise du robot (repère base_footprint) ;
  costmaps      coût sous le robot (locale, globale), obstacle létal le plus proche ;
  Nav2          arbre de comportement (Spin, BackUp, Wait, Clear...), buts abandonnés,
                messages WARN/ERROR expliqués (Failed to make progress, Start occupied...),
                cycle de vie et paramètres des nœuds ;
  localisation  sauts de map -> odom, TF manquantes, progrès (0,25 m en 15 s).

Sorties dans $FOSA_ROOT/diag (sinon ~/teams/Fosa/diag, sinon ~/fosa_diag) :
  diag_<date>.log     tout ce qui s'affiche, plus le détail ;
  diag_<date>.jsonl   une mesure par période (pour l'analyse) ;
  bag_<date>/         avec --bag : enregistrement léger, sans images.
"""
import argparse
import ast
import collections
import datetime
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np

# ------------------------------------------------------------------ réglages
BASE = 'base_footprint'
CARTE = 'map'
ODOM = 'odom'

# Valeurs de collision_monitor.yaml et nav2_params.yaml ; relues sur les nœuds.
ZONES_DEFAUT = {
    'ZoneArret': ([(0.30, 0.29), (0.30, -0.29), (-0.36, -0.29), (-0.36, 0.29)], 'stop'),
    'ZoneRalenti': ([(0.45, 0.33), (0.45, -0.33), (0.22, -0.33), (0.22, 0.33)], 'slowdown'),
}
EMPRISE_DEFAUT = [(0.22, 0.25), (0.22, -0.25), (-0.31, -0.25), (-0.31, 0.25)]
MARGE_EMPRISE_DEFAUT = 0.02
PROGRES_RAYON, PROGRES_DELAI = 0.25, 15.0       # progress_checker (nav2_params.yaml)

TOPIC_BRUT = '/fosa/cmd_vel_brut'               # Nav2 + course -> garde-fou
TOPIC_SORTIE = '/cmd_vel'                       # garde-fou -> robot
TOPIC_BASE = '/botwheel_explorer/cmd_vel'       # sécurité des organisateurs -> base
AUTRES_COMMANDES = ['/safe_vel', '/auto_vel', '/stamped_vel']
TOPIC_ODOM = '/botwheel_explorer/odom'
SCANS = ['/scan', '/fosa/scan_filtered', '/scan_clean', '/top_camera_scan', '/bottom_camera_scan']
DEBITS = ['/scan_clear', '/map', '/plan', '/fosa/top_camera/points', '/fosa/bottom_camera/points',
          '/fosa/tags_vus', '/odom', '/clock']
COSTMAPS = {'locale': '/local_costmap/costmap', 'globale': '/global_costmap/costmap'}
NOEUDS_CYCLE_VIE = ['/map_server', '/controller_server', '/planner_server', '/behavior_server',
                    '/bt_navigator', '/collision_monitor']
PUBLIEURS_ATTENDUS = {
    TOPIC_BRUT: {'controller_server', 'behavior_server', 'course_tags', 'velocity_smoother'},
    TOPIC_SORTIE: {'collision_monitor'},
}
TYPES_TWIST = ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped')
SEUIL_V, SEUIL_W = 0.01, 0.02                   # commande « non nulle »

# Noms de navigate_bounded_recovery.xml ; ClearLocal/GlobalCostmap sont dans ClearCostmaps.
BT_RECOVERIES = {'Spin', 'BackUp', 'Wait', 'DriveOnHeading', 'ClearCostmaps',
                 'ClearGlobalCostmapPlannerRecovery', 'ClearLocalCostmapControllerRecovery'}
BT_ECHECS = {'ComputePathToPose', 'FollowPath'}
ETATS_BUT = {0: 'inconnu', 1: 'accepté', 2: 'EN COURS', 3: 'annulation', 4: 'RÉUSSI',
             5: 'ANNULÉ', 6: 'ABANDONNÉ'}
ETATS_GARDE_FOU = {0: 'libre', 1: 'ARRÊT', 2: 'ralenti', 3: 'approche', 4: 'limite'}
INFO_SUIVIS = ('course_tags', 'bt_navigator', 'behavior_server', 'collision_monitor',
               'lifecycle_manager', 'controller_server', 'planner_server', 'slam_toolbox',
               'localisateur', 'mot_depart', 'referee', 'map_server')

# Messages de Nav2 et de la course -> clé d'explication.
INDICES = [
    (r'failed to make progress', 'pas_de_progres'),
    (r'start.{0,40}(occupied|lethal|obstacle|outside)|starting point.{0,30}lethal', 'depart_occupe'),
    (r'goal.{0,40}(occupied|lethal|obstacle|outside)', 'but_occupe'),
    (r'no valid (trajector|control)|optimizer fail|fail(ed)? to compute path|nocontrol',
     'mppi_sans_trajectoire'),
    (r'collision ahead|in collision|collision detected', 'collision'),
    (r'fail(ed)? to create (a )?plan|no valid path|could not find a path|0 poses|planning algorithm',
     'pas_de_chemin'),
    (r'sensor origin|out of (the )?(map )?bounds|outside (of )?the (costmap|map)', 'hors_costmap'),
    (r'extrapolat|timed out waiting for transform|could not transform|lookup would require'
     r'|message filter dropping|tf_old_data|invalid frame', 'tf'),
    (r'costmap timed out|timed out waiting for (the )?costmap|costmap.{0,20}not (been )?updated',
     'costmap_lente'),
    (r'control loop missed its desired rate|missed its desired rate', 'controleur_lent'),
    (r'invalid source|source .{0,40}(not published|out of date|not valid)|une source n.a pas',
     'garde_fou_source'),
    (r'garde-fou : arr', 'garde_fou_obstacle'),
    (r'patience exceeded', 'patience'),
    (r'goal (was )?aborted|navigation (failed|aborted)|aborting|goal failed', 'but_abandonne'),
]

EXPLICATIONS = {
    'garde_fou_obstacle': (
        'Notre garde-fou (collision_monitor) arrête le robot',
        "Des points du lidar ou des caméras sont dans ZoneArret (liste ci-dessus, repère "
        "base_footprint, x vers l'avant). S'ils sont collés au robot (|x| < 0,36 m, |y| < 0,30 m), "
        "c'est le robot lui-même : élargir le LaserScanBoxFilter "
        "(caytu_nav_bringup/config/laser_filter_params_real.yaml) ou réduire ZoneArret "
        "(collision_monitor.yaml). Caméra en cause : garde_fou_cameras:=false. Essai sans "
        "garde-fou : securite:=false (prudence)."),
    'garde_fou_source': (
        "Garde-fou : une source n'a pas de données fraîches (« invalid source »)",
        "collision_monitor arrête tout si /scan_clean ou une caméra a plus de 1 s de retard. Voir "
        "la ligne « débits ». Relancer avec garde_fou_cameras:=false (lidar seul)."),
    'garde_fou_bloque': (
        "Nav2 commande mais rien ne sort sur /cmd_vel",
        "collision_monitor reçoit /fosa/cmd_vel_brut mais publie zéro sans état ARRÊT connu : "
        "vérifier son cycle de vie (ligne « nœuds ») et /fosa/garde_fou_etat."),
    'garde_fou_muet': (
        "Personne ne publie /cmd_vel",
        "collision_monitor absent ou inactif : les commandes de Nav2 n'atteignent pas le robot."),
    'organisateurs': (
        "La sécurité des organisateurs bloque /cmd_vel",
        "/cmd_vel bouge mais /botwheel_explorer/cmd_vel reste à zéro : leur obstacle_avoidance "
        "juge un obstacle trop proche. Plus de marge aux murs (inflation), ou voir avec eux."),
    'base_immobile': (
        "La base reçoit des commandes mais l'odométrie ne bouge pas",
        "Arrêt d'urgence, moteurs, batterie, ou odométrie figée : "
        "ros2 topic echo /botwheel_explorer/odom pendant une commande."),
    'nav2_muet': (
        "Nav2 navigue mais ne commande rien",
        "Le contrôleur échoue ou attend : lire les messages controller_server / planner_server "
        "(MPPI sans trajectoire, costmap en retard, TF)."),
    'costmap_locale_letal': (
        "Obstacle létal dans l'emprise du robot (costmap locale)",
        "MPPI ne trouve plus de trajectoire sûre -> FollowPath échoue -> recovery. Souvent le "
        "robot vu par ses capteurs (retours lidar du châssis, caméra basse qui voit le sol) : "
        "voir « retours_robot » et les points listés."),
    'depart_occupe': (
        "Le centre du robot est un obstacle dans la costmap globale",
        "Le planificateur refuse (« Start occupied ») -> recovery. Taches de la carte sous le "
        "robot : edit_map <carte> --erase autour de la pose ; ou robot marqué par son lidar."),
    'but_occupe': (
        "Le but est dans un obstacle",
        "La pose visée tombe dans un mur ou une zone gonflée : vérifier tags.yaml et la carte."),
    'saut_localisation': (
        "Localisation : map -> odom saute",
        "slam_toolbox recale brutalement : carte différente du terrain, ou odométrie qui dérive "
        "(rotations sur place). Chemin et costmap globale se décalent -> échecs, recovery."),
    'tf_manquante': (
        "TF map -> base_footprint introuvable",
        "La localisation (slam_toolbox) ne publie pas map -> odom, ou odom -> base manque."),
    'pas_de_progres': (
        "Pas de progrès : moins de 0,25 m en 15 s pendant la navigation",
        "Le progress checker déclare l'échec (« Failed to make progress ») -> recovery. Voir à "
        "quel étage de la ligne « commandes » la vitesse tombe à zéro."),
    'retours_robot': (
        "Les capteurs voient le robot lui-même",
        "Des points sont DANS l'emprise du robot après filtrage : le filtre boîte du lidar ne "
        "les retire pas (ou c'est une caméra). Ils bloquent le garde-fou et la costmap locale."),
    'capteur_absent': (
        "Un capteur attendu n'est publié par personne",
        "Lidar filtré (/scan_clean) ou odométrie absents : costmaps et garde-fou sans données."),
    'capteur_muet': (
        "Un capteur existe mais n'envoie plus rien",
        "Topic présent sans message pendant la période : nœud bloqué, DDS, ou QoS."),
    'odom_controleur': (
        "controller_server lit une odométrie que personne ne publie",
        "MPPI croit le robot toujours immobile (vitesse 0) : suivi dégradé. Correctif : "
        "odom_topic: /botwheel_explorer/odom pour controller_server dans nav2_params.yaml."),
    'conflit_commande': (
        "Un autre nœud publie aussi la commande du robot",
        "Commandes contradictoires (robot saccadé ou arrêté) : arrêter ce nœud (téléop, essai)."),
    'cycle_vie': (
        "Un nœud Nav2 n'est pas actif",
        "Nav2 incomplet : relancer course.launch.py ; voir le journal du nœud cité."),
    'temps': (
        "Horloge : use_sim_time incohérent",
        "Un nœud attend /clock (use_sim_time true) sans simulateur : son temps reste à 0 "
        "(TF et minuteurs bloqués). Sur le robot, tout doit être en temps réel."),
    'costmap_figee': (
        "Une costmap n'est plus publiée",
        "La costmap n'a pas bougé depuis plusieurs secondes : nœud bloqué, TF ou capteurs."),
    'mppi_sans_trajectoire': (
        "MPPI ne trouve aucune trajectoire sans collision",
        "Obstacles trop près du robot dans la costmap locale (souvent le robot lui-même)."),
    'collision': ("Collision annoncée devant le robot", "Obstacle dans la costmap locale."),
    'pas_de_chemin': (
        "Le planificateur ne trouve pas de chemin",
        "But inaccessible dans la costmap globale (murs fantômes, porte fermée, but dans un mur)."),
    'hors_costmap': (
        "Robot ou capteur hors des limites de la costmap",
        "Localisation hors carte, ou costmap trop petite."),
    'tf': (
        "Problème de TF (retard, extrapolation)",
        "Données capteur ignorées : horloges, CPU saturé, ou TF publiée en retard."),
    'costmap_lente': ("Costmap en retard", "CPU saturé ou capteurs trop lents."),
    'controleur_lent': ("Contrôleur trop lent", "CPU saturé : MPPI n'atteint pas sa fréquence."),
    'patience': ("Patience du contrôleur dépassée", "Le suivi a échoué trop longtemps."),
    'but_abandonne': ("But abandonné par Nav2", "Voir les messages juste avant."),
}


# ------------------------------------------------------------------ calculs purs
def polygone_convexe(xs, ys, poly):
    """Masque des points (xs, ys) dans le polygone convexe poly (sens quelconque)."""
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    if xs.size == 0 or len(poly) < 3:
        return np.zeros(xs.shape, dtype=bool)
    produits = []
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        produits.append((x2 - x1) * (ys - y1) - (y2 - y1) * (xs - x1))
    produits = np.stack(produits)
    return np.all(produits >= 0, axis=0) | np.all(produits <= 0, axis=0)


def elargir(poly, marge):
    """Comme nav2 padFootprint : chaque sommet s'écarte de la marge en x et en y."""
    return [(x + math.copysign(marge, x), y + math.copysign(marge, y)) for x, y in poly]


def cercle(rayon, n=16):
    return [(rayon * math.cos(2 * math.pi * k / n), rayon * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def matrice_quaternion(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lacet(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def ecart_angle(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def points_scan(ranges, angle_min, increment, range_min, range_max, rotation, translation):
    """Points valides d'un LaserScan dans le repère cible : xs, ys, distances, angles."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(r.size) * increment
    ok = np.isfinite(r) & (r >= max(range_min, 0.01)) & (r <= range_max)
    r, a = r[ok], a[ok]
    p = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])
    q = rotation @ p + np.asarray(translation, dtype=float).reshape(3, 1)
    return q[0], q[1], r, a


def analyser_points(xs, ys, r, a, zones, emprise, n_proches=5):
    """Points dans chaque zone et dans l'emprise ; les plus proches parmi les gênants."""
    xs, ys = np.asarray(xs), np.asarray(ys)
    resultat = {'zones': {}, 'emprise': 0, 'proches': [], 'devant': None}
    genants = np.zeros(xs.shape, dtype=bool)
    for nom, (poly, action) in zones.items():
        masque = polygone_convexe(xs, ys, poly)
        resultat['zones'][nom] = int(masque.sum())
        if action == 'stop':
            genants |= masque
    dans_emprise = polygone_convexe(xs, ys, emprise)
    resultat['emprise'] = int(dans_emprise.sum())
    genants |= dans_emprise
    if genants.any():
        idx = np.nonzero(genants)[0]
        ordre = idx[np.argsort(np.hypot(xs[idx], ys[idx]))][:n_proches]
        resultat['proches'] = [(round(float(xs[k]), 3), round(float(ys[k]), 3),
                                round(math.degrees(float(a[k])), 1), round(float(r[k]), 3))
                               for k in ordre]
    devant = (xs > 0) & (np.abs(ys) < 0.35)
    if devant.any():
        resultat['devant'] = round(float(xs[devant].min()), 3)
    return resultat


def analyser_costmap(valeurs, resolution, origine, pose, emprise, rayon_recherche=1.0):
    """Coûts d'une OccupancyGrid de costmap (0..100, 99 inscrit, 100 létal, -1 inconnu).

    valeurs : tableau (hauteur, largeur) ; origine : (ox, oy, lacet) ; pose : (x, y, lacet)
    du robot dans le repère de la grille ; emprise : polygone dans le repère du robot.
    """
    haut, larg = valeurs.shape
    ox, oy, olacet = origine
    x, y, cap = pose
    c_o, s_o = math.cos(-olacet), math.sin(-olacet)

    def cellules(px, py):
        mx = x + math.cos(cap) * px - math.sin(cap) * py
        my = y + math.sin(cap) * px + math.cos(cap) * py
        dx, dy = mx - ox, my - oy
        i = np.floor((c_o * dx - s_o * dy) / resolution).astype(int)
        j = np.floor((s_o * dx + c_o * dy) / resolution).astype(int)
        return i, j

    ex, ey = [p[0] for p in emprise], [p[1] for p in emprise]
    pas = resolution / 2.0
    gx, gy = np.meshgrid(np.arange(min(ex), max(ex) + 1e-9, pas),
                         np.arange(min(ey), max(ey) + 1e-9, pas))
    gx, gy = gx.ravel(), gy.ravel()
    garde = polygone_convexe(gx, gy, emprise)
    i, j = cellules(gx[garde], gy[garde])
    dedans = (i >= 0) & (i < larg) & (j >= 0) & (j < haut)
    cellules_uniques = sorted(set(zip(i[dedans].tolist(), j[dedans].tolist())))
    v = np.array([valeurs[jj, ii] for ii, jj in cellules_uniques], dtype=int)
    connus = v[v >= 0]
    resultat = {
        'max_emprise': int(connus.max()) if connus.size else None,
        'letal_emprise': int((v >= 100).sum()),
        'inscrit_emprise': int((v == 99).sum()),
        'inconnu_emprise': int((v < 0).sum()),
        'hors_grille': bool((~dedans).any()),
        'centre': None, 'letal_proche': None,
    }
    ic, jc = cellules(np.array([0.0]), np.array([0.0]))
    ic, jc = int(ic[0]), int(jc[0])
    if 0 <= ic < larg and 0 <= jc < haut:
        resultat['centre'] = int(valeurs[jc, ic])
    n = int(math.ceil(rayon_recherche / resolution))
    i0, i1, j0, j1 = max(ic - n, 0), min(ic + n + 1, larg), max(jc - n, 0), min(jc + n + 1, haut)
    if i0 < i1 and j0 < j1:
        jj, ii = np.nonzero(valeurs[j0:j1, i0:i1] >= 100)
        if ii.size:
            cx, cy = (ii + i0 + 0.5) * resolution, (jj + j0 + 0.5) * resolution
            mx = ox + math.cos(olacet) * cx - math.sin(olacet) * cy
            my = oy + math.sin(olacet) * cx + math.cos(olacet) * cy
            dx, dy = mx - x, my - y
            rx = math.cos(-cap) * dx - math.sin(-cap) * dy
            ry = math.sin(-cap) * dx + math.cos(-cap) * dy
            d = np.hypot(rx, ry)
            k = int(np.argmin(d))
            if d[k] <= rayon_recherche:
                resultat['letal_proche'] = (round(float(d[k]), 2), round(float(rx[k]), 2),
                                            round(float(ry[k]), 2))
    return resultat


def stats_commandes(echantillons):
    """echantillons : [(t, v, w)] -> part non nulle, moyennes et maxima (None si vide)."""
    if not echantillons:
        return None
    v = np.array([e[1] for e in echantillons])
    w = np.array([e[2] for e in echantillons])
    non_nul = (np.abs(v) > SEUIL_V) | (np.abs(w) > SEUIL_W)
    return {'n': int(v.size), 'part': round(float(non_nul.mean()), 2),
            'v': round(float(v.mean()), 3), 'w': round(float(w.mean()), 3),
            'v_max': round(float(np.abs(v).max()), 3), 'w_max': round(float(np.abs(w).max()), 3)}


def normaliser(texte):
    return re.sub(r'-?\d+(\.\d+)?(e-?\d+)?', '#', texte)[:160]


def indice_message(texte):
    bas = texte.lower()
    for motif, cle in INDICES:
        if re.search(motif, bas):
            return cle
    return None


def parse_polygone(valeur):
    """Paramètre Nav2 de polygone : chaîne « [[x, y], ...] » ou liste plate [x1, y1, ...]."""
    if isinstance(valeur, str):
        if not valeur.strip():
            return None
        points = ast.literal_eval(valeur)
    else:
        plats = list(valeur)
        points = [plats[k:k + 2] for k in range(0, len(plats) - 1, 2)]
    points = [(float(p[0]), float(p[1])) for p in points]
    return points if len(points) >= 3 else None


def diagnostiquer(p):
    """Mesures d'une période -> [(clé, détail)] : ce qui empêche le robot d'avancer."""
    verdicts = []
    nav = p['nav']['actif']
    chaine, cmd = p['chaine'], p['cmd']
    brut = cmd.get(chaine['brut']) if chaine.get('brut') else None
    sortie = cmd.get(chaine['sortie']) if chaine.get('sortie') else None
    base = cmd.get(chaine['base']) if chaine.get('base') else None
    odom = p['odom']

    def bouge(st):
        return st is not None and st['part'] > 0.3

    def immobile(st):
        return st is None or st['part'] < 0.1

    etats = p['garde_fou']['durees']
    total = sum(etats.values())
    if not total:                    # aucun changement d'état : l'état courant vaut pour tout
        etats, total = {p['garde_fou']['actuel']: 1.0}, 1.0
    arret = sum(d for k, d in etats.items() if k.startswith('ARRÊT') and 'invalid' not in k)
    source = sum(d for k, d in etats.items() if 'invalid source' in k)
    arret = arret / total if total else 0.0
    source = source / total if total else 0.0
    if source > 0.2:
        verdicts.append(('garde_fou_source', f'{source:.0%} du temps'))
    if chaine.get('brut') and bouge(brut) and immobile(sortie) and source <= 0.2:
        if arret > 0.2:
            zones = []
            for topic, sc in p['scans'].items():
                if topic in p['sources_garde_fou'] and sc.get('analyses'):
                    pire = max(sc['zones'].items(), key=lambda kv: kv[1], default=(None, 0))
                    if pire[1]:
                        zones.append(f'{topic} : {pire[1]} pts dans {pire[0]}')
            verdicts.append(('garde_fou_obstacle', f'{arret:.0%} du temps ; ' + (
                ', '.join(zones) or 'points non vus ici (caméra ?)')))
        elif sortie is None and not chaine.get('sortie_publiee'):
            verdicts.append(('garde_fou_muet', ''))
        else:
            verdicts.append(('garde_fou_bloque', ''))
    entree_base = sortie if chaine.get('sortie') else brut
    if chaine.get('base') and bouge(entree_base) and immobile(base):
        verdicts.append(('organisateurs', ''))
    finale = base if chaine.get('base') else entree_base
    if (bouge(finale) and odom is not None and odom['v'] < 0.01 and odom['w'] < 0.02
            and odom.get('deplacement', 0.0) < 0.02 and odom.get('rotation', 0.0) < 2.0):
        verdicts.append(('base_immobile', f"odom v={odom['v']:.3f} w={odom['w']:.3f}, "
                                          f"{odom.get('deplacement', 0.0):.2f} m"))
    nav_cmd = brut if chaine.get('brut') else sortie
    if nav and immobile(nav_cmd) and not p['bt']['recovery']:
        verdicts.append(('nav2_muet', ''))
    locale, globale = p['costmaps'].get('locale'), p['costmaps'].get('globale')
    if locale and locale.get('letal_emprise'):
        verdicts.append(('costmap_locale_letal', f"{locale['letal_emprise']} cellules létales"))
    if globale and (globale.get('centre') or 0) >= 99:
        verdicts.append(('depart_occupe', f"coût {globale['centre']} au centre"))
    for nom in ('locale', 'globale'):
        cm = p['costmaps'].get(nom)
        if cm and cm.get('age', 0) > 5.0:
            verdicts.append(('costmap_figee', f"{nom} : {cm['age']:.0f} s"))
    loc = p['loc']
    if loc['sauts']:
        verdicts.append(('saut_localisation', f"{loc['sauts']} saut(s), max {loc['saut_max']}"))
    if loc['echantillons'] and loc['tf_map_manquante'] > 0.5 * loc['echantillons']:
        verdicts.append(('tf_manquante', ''))
    if nav and p['progres'] is not None and p['progres'] < p['progres_rayon']:
        verdicts.append(('pas_de_progres', f"{p['progres']:.2f} m en {p['progres_delai']:.0f} s"))
    for topic, sc in p['scans'].items():
        if topic in p['sources_garde_fou'] and sc.get('emprise_max', 0) >= 3:
            verdicts.append(('retours_robot', f"{topic} : {sc['emprise_max']} pts dans l'emprise"))
    for topic in p['manquants']:
        verdicts.append(('capteur_absent', topic))
    for topic in p['muets']:
        verdicts.append(('capteur_muet', topic))
    co = p.get('controleur_odom')
    if co and co['publieurs'] == 0:
        verdicts.append(('odom_controleur', co['topic']))
    for texte in p['conflits']:
        verdicts.append(('conflit_commande', texte))
    for texte in p['inactifs']:
        verdicts.append(('cycle_vie', texte))
    for texte in p['temps']:
        verdicts.append(('temps', texte))
    return verdicts


def fmt_cmd(st):
    if st is None:
        return 'rien'
    return f"v{st['v']:+.2f} w{st['w']:+.2f} ({st['part']:.0%})"


def fmt_rapport(p, verdicts):
    lignes = []
    nav = p['nav']
    entete = f"── +{p['t']:.0f} s "
    if nav['actif']:
        entete += f"─ NAV en cours (reste {nav['reste']}, recoveries {nav['recoveries']})"
    else:
        entete += '─ pas de navigation en cours'
    if p['loc']['pose']:
        x, y, c = p['loc']['pose']
        entete += f" ─ map ({x:.2f}, {y:.2f}, {c:.0f}°)"
    lignes.append(entete)
    ch, cmd = p['chaine'], p['cmd']
    etages = []
    if ch.get('brut'):
        etages.append(f"Nav2 {fmt_cmd(cmd.get(ch['brut']))}")
    if ch.get('sortie'):
        etages.append(f"{ch['sortie']} {fmt_cmd(cmd.get(ch['sortie']))}")
    if ch.get('base'):
        etages.append(f"base {fmt_cmd(cmd.get(ch['base']))}")
    od = p['odom']
    odom = (f"odom v{od['v']:.2f} w{od['w']:.2f} ({od.get('deplacement', 0):.2f} m, "
            f"{od.get('rotation', 0):.0f}°)" if od else 'odom : rien')
    lignes.append(f"  commandes : {' → '.join(etages) or 'aucun topic de commande'} │ {odom}")
    gf = p['garde_fou']
    total = sum(gf['durees'].values())
    if total:
        parts = ', '.join(f'{k} {d / total:.0%}' for k, d in
                          sorted(gf['durees'].items(), key=lambda kv: -kv[1]))
    else:
        parts = f"{gf['actuel']} (aucun changement)"
    lignes.append(f"  garde-fou : {parts}")
    for topic, sc in p['scans'].items():
        if not sc.get('analyses'):
            if sc.get('tf_ko'):
                lignes.append(f"    {topic} : TF vers {BASE} introuvable")
            continue
        zones = ' '.join(f'{k} {v}' for k, v in sc['zones'].items())
        devant = f", devant {sc['devant']:.2f} m" if sc.get('devant') is not None else ''
        lignes.append(f"    {topic} : emprise {sc['emprise_max']} pts, {zones}{devant}"
                      + ('  [garde-fou]' if topic in p['sources_garde_fou'] else ''))
        if sc.get('proches') and (sc['emprise_max'] or any(
                sc['zones'].get(n, 0) for n, (_, a) in p['zones'].items() if a == 'stop')):
            lignes.append('      gênants (x, y, angle lidar, distance) : ' + ' ; '.join(
                f'({x:+.2f}, {y:+.2f}) {a:+.0f}° {r:.2f} m' for x, y, a, r in sc['proches']))
    for nom in ('locale', 'globale'):
        cm = p['costmaps'].get(nom)
        if cm is None:
            continue
        if cm.get('erreur'):
            lignes.append(f"  costmap {nom} : {cm['erreur']}")
            continue
        proche = cm.get('letal_proche')
        proche = (f", létal le + proche {proche[0]:.2f} m ({proche[1]:+.2f}, {proche[2]:+.2f})"
                  if proche else ', aucun létal à 1 m')
        lignes.append(f"  costmap {nom} : centre {cm['centre']}, max emprise {cm['max_emprise']} "
                      f"(létal {cm['letal_emprise']}, inscrit {cm['inscrit_emprise']}, inconnu "
                      f"{cm['inconnu_emprise']}){proche}, âge {cm['age']:.1f} s")
    loc = p['loc']
    etat_loc = 'OK' if not loc['sauts'] and not loc['tf_map_manquante'] else (
        f"{loc['sauts']} saut(s), TF map manquante {loc['tf_map_manquante']}/{loc['echantillons']}")
    progres = (f"{p['progres']:.2f} m / {p['progres_delai']:.0f} s"
               if p['progres'] is not None else '-')
    lignes.append(f"  localisation : {etat_loc} │ progrès {progres} │ BT : recoveries "
                  f"{p['bt']['recoveries']}, échecs plan {p['bt']['echecs'].get('ComputePathToPose', 0)}"
                  f", suivi {p['bt']['echecs'].get('FollowPath', 0)}")
    debits = ', '.join(f'{t} {hz:.1f}' for t, hz in p['debits'].items())
    lignes.append(f"  débits (Hz) : {debits}")
    for cle, detail in verdicts:
        titre = EXPLICATIONS.get(cle, (cle, ''))[0]
        lignes.append(f"  ⇒ {titre}" + (f" — {detail}" if detail else ''))
    return lignes


def dossier_sortie(argument):
    if argument:
        return os.path.abspath(os.path.expanduser(argument))
    if os.environ.get('FOSA_ROOT'):
        return os.path.join(os.environ['FOSA_ROOT'], 'diag')
    equipe = os.path.expanduser('~/teams/Fosa')
    if os.path.isdir(equipe):
        return os.path.join(equipe, 'diag')
    # PC : hors du workspace (jamais dans src/, que l'on zippe pour le robot).
    return os.path.expanduser('~/fosa_diag')


class Journal:
    def __init__(self, dossier, horodatage):
        os.makedirs(dossier, exist_ok=True)
        self.chemin = os.path.join(dossier, f'diag_{horodatage}.log')
        self.chemin_mesures = os.path.join(dossier, f'diag_{horodatage}.jsonl')
        self._fichier = open(self.chemin, 'a', encoding='utf-8')
        self._mesures = open(self.chemin_mesures, 'a', encoding='utf-8')
        self.debut = time.monotonic()

    def __call__(self, texte, console=True):
        horo = f'{time.monotonic() - self.debut:7.1f}s '
        for ligne in str(texte).splitlines() or ['']:
            if console:
                print(ligne, flush=True)
            self._fichier.write(horo + ligne + '\n')
        self._fichier.flush()

    def mesure(self, donnees):
        self._mesures.write(json.dumps(donnees, ensure_ascii=False, default=str) + '\n')
        self._mesures.flush()

    def fermer(self):
        self._fichier.close()
        self._mesures.close()


# ------------------------------------------------------------------ nœud ROS
def creer_noeud(args, journal):
    """Construit le nœud (imports ROS ici : les calculs ci-dessus se testent sans ROS)."""
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError:              # très vieux ROS : pas d'abonnement générique
        get_message = None
    classes = {}
    for chemin, nom in (('nav2_msgs.action', 'NavigateToPose'), ('action_msgs.msg', 'GoalStatusArray'),
                        ('rcl_interfaces.msg', 'Log'), ('rcl_interfaces.srv', 'GetParameters'),
                        ('lifecycle_msgs.srv', 'GetState')):
        try:
            classes[nom] = getattr(__import__(chemin, fromlist=[nom]), nom)
        except Exception:            # noqa: BLE001 - paquet absent : fonction coupée
            classes[nom] = None

    class Voulu:
        def __init__(self, topic, rappel, types=None, classe=None, latched=False, profondeur=10,
                     capteur=False):
            self.topic, self.rappel, self.types, self.classe = topic, rappel, types, classe
            self.latched, self.profondeur, self.capteur = latched, profondeur, capteur
            self.abonnement, self.type, self.erreur = None, None, False

    class Diagnostic(Node):
        def __init__(self):
            super().__init__('fosa_diagnostic', enable_rosout=False, start_parameter_services=False,
                             parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL,
                                                            args.sim)])
            self.fini = False
            self._debut = time.monotonic()
            self._voulus = {}
            self._erreurs_rappel = set()
            self._tf = Buffer(cache_time=Duration(seconds=30.0))
            self._ecouteur_tf = TransformListener(self._tf, self, spin_thread=False)
            self._transfos = {}
            self._zones = dict(ZONES_DEFAUT)
            self._emprise = elargir(EMPRISE_DEFAUT, MARGE_EMPRISE_DEFAUT)
            self._progres = (PROGRES_RAYON, PROGRES_DELAI)
            self._chaine = {'brut': TOPIC_BRUT, 'sortie': TOPIC_SORTIE, 'base': TOPIC_BASE}
            self._sources_garde_fou = {'/scan_clean'}
            self._odom_controleur = None
            self._params = {}
            self._clients_services = {}
            self._cycle_vie = {}
            self._temps = []
            self._publieurs = {}
            self._conflits = []
            # cumuls sur toute la durée (bilan)
            self._cumul_verdicts = collections.Counter()
            self._cumul_indices = collections.Counter()
            self._cumul_messages = collections.Counter()
            self._exemples_messages = {}
            self._cumul_recoveries = collections.Counter()
            self._cumul_echecs = collections.Counter()
            self._buts = {}
            self._periodes = 0
            self._periodes_nav = 0
            self._sauts_total = 0
            self._dernier_imprime = {}
            self._indices_expliques = set()
            # état courant
            self._statuts = {}
            self._feedback = None
            self._feedback_t = -1e9
            self._recoveries_nav = 0
            self._bt_en_cours = set()
            self._gf_etat, self._gf_depuis = 'inconnu', time.monotonic()
            self._plan = None
            self._but_plan = None
            self._costmaps = {}
            self._dernier_mo = None
            self._historique = collections.deque(maxlen=3000)
            self._run_status = None
            self._reinitialiser_fenetre()

            for topic in [TOPIC_BRUT, TOPIC_SORTIE, TOPIC_BASE] + AUTRES_COMMANDES:
                self._vouloir(topic, self._commande, types=TYPES_TWIST, capteur=True)
            self._vouloir(args.odom, self._odometrie, types=('nav_msgs/msg/Odometry',), capteur=True)
            for topic in SCANS:
                self._vouloir(topic, self._scan, types=('sensor_msgs/msg/LaserScan',), capteur=True)
            for nom, topic in COSTMAPS.items():
                self._vouloir(topic, self._costmap, types=('nav_msgs/msg/OccupancyGrid',),
                              latched=True, profondeur=1)
            self._vouloir('/plan', self._chemin, types=('nav_msgs/msg/Path',))
            self._vouloir(args.topic_garde_fou, self._garde_fou,
                          types=('nav2_msgs/msg/CollisionMonitorState',))
            self._vouloir('/behavior_tree_log', self._arbre, types=('nav2_msgs/msg/BehaviorTreeLog',),
                          profondeur=50)
            if classes['NavigateToPose'] is not None:
                self._vouloir('/navigate_to_pose/_action/feedback', self._retour_nav,
                              classe=classes['NavigateToPose'].Impl.FeedbackMessage)
            if classes['GoalStatusArray'] is not None:
                self._vouloir('/navigate_to_pose/_action/status', self._statut_nav,
                              classe=classes['GoalStatusArray'], latched=True, profondeur=5)
            if classes['Log'] is not None:
                self._vouloir('/rosout', self._rosout, classe=classes['Log'], profondeur=1000)
            self._vouloir(args.topic_statut, self._statut_course, types=('std_msgs/msg/String',),
                          latched=True)
            self._vouloir('/parc/run_status', self._statut_arbitre, latched=True)
            for topic in DEBITS:
                if topic not in self._voulus:
                    self._vouloir(topic, None, capteur=True, latched=topic == '/map', profondeur=1)

            self._horloge = Clock(clock_type=ClockType.STEADY_TIME)
            self._minuteur(2.0, self._graphe)
            self._minuteur(0.2, self._echantillonner)
            self._minuteur(float(args.periode), self._rapport)
            self._minuteur(10.0, self._controles)
            self._minuteur(30.0, self._interroger_cycle_vie)
            self._graphe()

        # ------------------------------------------------------------ outils
        def _minuteur(self, periode, rappel):
            def protege():
                # Un contrôle qui échoue ne doit jamais arrêter le diagnostic.
                try:
                    rappel()
                except Exception as erreur:      # noqa: BLE001
                    nom = getattr(rappel, '__name__', str(rappel))
                    if nom not in self._erreurs_rappel:
                        self._erreurs_rappel.add(nom)
                        self._evenement(f'(diagnostic) erreur dans {nom} : {erreur!r}')
            try:
                return self.create_timer(periode, protege, clock=self._horloge)
            except TypeError:        # rclpy sans argument clock
                return self.create_timer(periode, protege)

        def _ros(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _t(self):
            return time.monotonic() - self._debut

        def _evenement(self, texte, console=True):
            journal(f'[+{self._t():6.1f} s] {texte}', console=console)

        def _reinitialiser_fenetre(self):
            self._f_debut_mur, self._f_debut_ros = time.monotonic(), self._ros()
            self._f_cmd = collections.defaultdict(list)
            self._f_odom = []
            self._f_scans = {}
            self._f_compte = collections.Counter()
            self._f_gf = collections.Counter()
            self._f_recoveries = 0
            self._f_echecs = collections.Counter()
            self._f_loc = {'sauts': 0, 'saut_max': '', 'tf_map_manquante': 0, 'echantillons': 0}
            self._f_nav = []

        def _vouloir(self, topic, rappel, types=None, classe=None, latched=False, profondeur=10,
                     capteur=False):
            self._voulus[topic] = Voulu(topic, rappel, types, classe, latched, profondeur, capteur)

        def _qos(self, voulu):
            infos = self.get_publishers_info_by_topic(voulu.topic)
            fiable = (not voulu.capteur and bool(infos) and all(
                i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos))
            durable = voulu.latched and bool(infos) and all(
                i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=voulu.profondeur,
                reliability=ReliabilityPolicy.RELIABLE if fiable else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)

        def _recu(self, voulu, message):
            self._f_compte[voulu.topic] += 1
            if voulu.rappel is None:
                return
            try:
                voulu.rappel(voulu.topic, message)
            except Exception as erreur:          # noqa: BLE001 - le diagnostic continue
                if voulu.topic not in self._erreurs_rappel:
                    self._erreurs_rappel.add(voulu.topic)
                    self._evenement(f'(diagnostic) erreur sur {voulu.topic} : {erreur!r}')

        def _graphe(self):
            """Abonnements dès que les topics existent, avec leur type et une QoS compatible."""
            types = dict(self.get_topic_names_and_types())
            for voulu in self._voulus.values():
                if voulu.abonnement is not None or voulu.topic not in types:
                    continue
                try:
                    if voulu.classe is not None:
                        classe, nom = voulu.classe, types[voulu.topic][0]
                    else:
                        nom = next((t for t in types[voulu.topic]
                                    if not voulu.types or t in voulu.types), None)
                        if nom is None or get_message is None:
                            if not voulu.erreur:
                                voulu.erreur = True
                                self._evenement(f'{voulu.topic} : type inattendu '
                                                f'{types[voulu.topic]}, ignoré')
                            continue
                        classe = get_message(nom)
                    voulu.abonnement = self.create_subscription(
                        classe, voulu.topic, lambda m, v=voulu: self._recu(v, m), self._qos(voulu))
                    voulu.type = nom
                    journal(f'  écoute {voulu.topic} ({nom})', console=False)
                except Exception as erreur:      # noqa: BLE001
                    if not voulu.erreur:
                        voulu.erreur = True
                        self._evenement(f'{voulu.topic} : abonnement impossible ({erreur})')

        def _transfo(self, repere):
            if repere in ('', BASE):
                return np.eye(3), np.zeros(3)
            maintenant = time.monotonic()
            cache = self._transfos.get(repere)
            if cache and maintenant - cache[0] < 2.0:
                return cache[1]
            try:
                t = self._tf.lookup_transform(BASE, repere, Time())
            except Exception:                    # noqa: BLE001
                return cache[1] if cache else None
            q, tr = t.transform.rotation, t.transform.translation
            valeur = (matrice_quaternion(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z]))
            self._transfos[repere] = (maintenant, valeur)
            return valeur

        def _pose(self, cible, source):
            try:
                t = self._tf.lookup_transform(cible, source, Time())
            except Exception:                    # noqa: BLE001
                return None
            q, tr = t.transform.rotation, t.transform.translation
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            return tr.x, tr.y, lacet(q.x, q.y, q.z, q.w), stamp

        def _nav_active(self):
            if any(s in (1, 2, 3) for s in self._statuts.values()):
                return True
            return time.monotonic() - self._feedback_t < 3.0

        # ------------------------------------------------------------ rappels
        def _commande(self, topic, message):
            tw = getattr(message, 'twist', message)
            self._f_cmd[topic].append((time.monotonic(), float(tw.linear.x), float(tw.angular.z)))

        def _odometrie(self, _topic, message):
            tw, po = message.twist.twist, message.pose.pose
            q = po.orientation
            self._f_odom.append((abs(float(tw.linear.x)), abs(float(tw.angular.z)),
                                 float(po.position.x), float(po.position.y),
                                 lacet(q.x, q.y, q.z, q.w)))

        def _scan(self, topic, message):
            maintenant = time.monotonic()
            f = self._f_scans.setdefault(topic, {'analyses': 0, 'emprise_max': 0, 'zones': {},
                                                 'proches': [], 'devant': None, 'tf_ko': 0,
                                                 '_pire': -1, '_dernier': 0.0})
            if maintenant - f['_dernier'] < 0.15:
                return
            f['_dernier'] = maintenant
            transfo = self._transfo(message.header.frame_id)
            if transfo is None:
                f['tf_ko'] += 1
                return
            xs, ys, r, a = points_scan(message.ranges, message.angle_min, message.angle_increment,
                                       message.range_min, message.range_max, *transfo)
            res = analyser_points(xs, ys, r, a, self._zones, self._emprise)
            f['analyses'] += 1
            f['emprise_max'] = max(f['emprise_max'], res['emprise'])
            for nom, n in res['zones'].items():
                f['zones'][nom] = max(f['zones'].get(nom, 0), n)
            gravite = res['emprise'] + sum(n for nom, n in res['zones'].items()
                                           if self._zones[nom][1] == 'stop')
            if gravite > f['_pire']:
                f['_pire'], f['proches'] = gravite, res['proches']
            if res['devant'] is not None:
                f['devant'] = res['devant'] if f['devant'] is None else min(f['devant'], res['devant'])

        def _costmap(self, topic, message):
            self._costmaps[topic] = (time.monotonic(), message)

        def _chemin(self, _topic, message):
            poses = message.poses
            if not poses:
                self._evenement('Nav2 : chemin VIDE publié sur /plan')
                return
            fin = poses[-1].pose.position
            but = (round(fin.x, 2), round(fin.y, 2))
            longueur = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                                      b.pose.position.y - a.pose.position.y)
                           for a, b in zip(poses[:-1], poses[1:]))
            self._plan = {'n': len(poses), 'longueur': round(longueur, 2), 'but': but,
                          'repere': message.header.frame_id, 't': time.monotonic()}
            if self._but_plan is None or math.hypot(but[0] - self._but_plan[0],
                                                    but[1] - self._but_plan[1]) > 0.3:
                self._but_plan = but
                self._evenement(f'Nav2 : nouveau chemin vers ({but[0]:.2f}, {but[1]:.2f}) '
                                f'[{message.header.frame_id}], {longueur:.1f} m')

        def _garde_fou(self, _topic, message):
            nom = ETATS_GARDE_FOU.get(int(message.action_type), str(message.action_type))
            etat = nom + (f' {message.polygon_name}' if message.polygon_name else '')
            if etat == self._gf_etat:
                return
            maintenant = time.monotonic()
            self._f_gf[self._gf_etat] += maintenant - self._gf_depuis
            self._evenement(f'Garde-fou : {self._gf_etat} -> {etat}')
            self._gf_etat, self._gf_depuis = etat, maintenant

        def _arbre(self, _topic, message):
            for ev in message.event_log:
                nom, avant, apres = ev.node_name, ev.previous_status, ev.current_status
                if nom in BT_RECOVERIES:
                    if avant == 'IDLE' and apres != 'IDLE':
                        self._f_recoveries += 1
                        self._cumul_recoveries[nom] += 1
                        self._evenement(f'BT : recovery {nom}' + (
                            f' -> {apres}' if apres != 'RUNNING' else ''))
                    elif apres == 'FAILURE':
                        self._evenement(f'BT : recovery {nom} ÉCHOUE')
                    if apres == 'RUNNING':
                        self._bt_en_cours.add(nom)
                    else:
                        self._bt_en_cours.discard(nom)
                elif nom in BT_ECHECS and apres == 'FAILURE':
                    self._f_echecs[nom] += 1
                    self._cumul_echecs[nom] += 1
                    self._evenement(f'BT : {nom} ÉCHOUE')
                elif nom == 'GlobalRecovery' and avant == 'IDLE' and apres == 'RUNNING':
                    self._evenement('BT : ==> RECOVERY GLOBAL (Clear / Spin / Wait / BackUp)')
                elif nom == 'NavigateRecovery' and apres in ('FAILURE', 'SUCCESS'):
                    self._evenement('BT : but ' + ('ATTEINT' if apres == 'SUCCESS'
                                                   else 'ABANDONNÉ par Nav2 (essais épuisés)'))
                else:
                    journal(f'  BT {nom} {avant}->{apres}', console=False)

        def _retour_nav(self, _topic, message):
            fb = message.feedback
            self._feedback_t = time.monotonic()
            n = int(fb.number_of_recoveries)
            if n > self._recoveries_nav:
                self._evenement(f'Nav2 : recovery n° {n} (reste {fb.distance_remaining:.2f} m)')
            self._recoveries_nav = n
            self._feedback = {'reste': round(float(fb.distance_remaining), 2), 'recoveries': n}

        def _statut_nav(self, _topic, message):
            vus = {}
            for st in message.status_list:
                ident = bytes(st.goal_info.goal_id.uuid).hex()[:6]
                vus[ident] = int(st.status)
                if self._buts.get(ident) != st.status:
                    self._buts[ident] = int(st.status)
                    self._evenement(f'But {ident} : {ETATS_BUT.get(int(st.status), st.status)}')
                    if st.status == 2:
                        self._recoveries_nav = 0
            self._statuts = vus

        def _rosout(self, _topic, message):
            if message.name.startswith('fosa_diagnostic'):
                return
            niveau = int(message.level)
            if niveau < 30 and not message.name.startswith(INFO_SUIVIS):
                return
            if niveau < 20:
                return
            texte = message.msg.strip()
            cle = (message.name, normaliser(texte))
            self._cumul_messages[cle] += 1
            indice = indice_message(texte)
            if indice and (niveau >= 30 or message.name.startswith('course_tags')):
                self._cumul_indices[indice] += 1
            if niveau >= 30:
                self._exemples_messages.setdefault(cle, (niveau, texte))
            maintenant = time.monotonic()
            dernier = self._dernier_imprime.get(cle)
            if dernier and maintenant - dernier[0] < 15.0:
                self._dernier_imprime[cle] = (dernier[0], dernier[1] + 1)
                return
            repetitions = dernier[1] if dernier else 0
            self._dernier_imprime[cle] = (maintenant, 0)
            etiquette = {20: 'info', 30: 'WARN', 40: 'ERREUR', 50: 'FATAL'}.get(niveau, niveau)
            self._evenement(f'[{etiquette}] {message.name} : {texte[:300]}'
                            + (f'  (+{repetitions} fois)' if repetitions else ''))
            if indice and indice not in self._indices_expliques and niveau >= 30:
                self._indices_expliques.add(indice)
                self._evenement(f'    ↳ {EXPLICATIONS[indice][0]} : {EXPLICATIONS[indice][1]}')

        def _statut_course(self, _topic, message):
            self._evenement(f'Course : {message.data[:300]}')

        def _statut_arbitre(self, _topic, message):
            etat = getattr(message, 'state', None)
            if etat is None:
                etat = getattr(message, 'status', None)
            texte = str(etat) if etat is not None else str(message)[:200]
            if texte != self._run_status:
                self._run_status = texte
                self._evenement(f'Arbitre /parc/run_status : {texte}')

        # ------------------------------------------------------------ échantillons
        def _echantillonner(self):
            mur, ros = time.monotonic(), self._ros()
            active = self._nav_active()
            self._f_nav.append(active)
            self._f_loc['echantillons'] += 1
            pose = self._pose(CARTE, BASE)
            if pose is None:
                self._f_loc['tf_map_manquante'] += 1
            else:
                self._historique.append((ros, mur, pose[0], pose[1], pose[2], active))
            mo = self._pose(CARTE, ODOM)
            if mo is not None:
                if self._dernier_mo is not None:
                    d = math.hypot(mo[0] - self._dernier_mo[0], mo[1] - self._dernier_mo[1])
                    da = abs(math.degrees(ecart_angle(mo[2], self._dernier_mo[2])))
                    if d > 0.15 or da > 8.0:
                        self._f_loc['sauts'] += 1
                        self._sauts_total += 1
                        self._f_loc['saut_max'] = f'{d:.2f} m / {da:.0f}°'
                        self._evenement(f'Localisation : map -> odom saute de {d:.2f} m et '
                                        f'{da:.0f}°')
                self._dernier_mo = mo

        def _progres_recent(self):
            """Plus grand écart à la pose d'il y a PROGRES_DELAI s, si la navigation dure."""
            rayon, delai = self._progres
            if not self._historique:
                return None
            fin = self._historique[-1][0]
            fenetre = [h for h in self._historique if h[0] >= fin - delai]
            if not fenetre or fin - fenetre[0][0] < 0.9 * delai or not all(h[5] for h in fenetre):
                return None
            x0, y0 = fenetre[0][2], fenetre[0][3]
            return round(max(math.hypot(h[2] - x0, h[3] - y0) for h in fenetre), 3)

        def _analyse_costmaps(self):
            resultats = {}
            for nom, topic in COSTMAPS.items():
                recu = self._costmaps.get(topic)
                if recu is None:
                    continue
                t, msg = recu
                info = msg.info
                if info.width == 0 or info.height == 0:
                    continue
                pose = self._pose(msg.header.frame_id, BASE)
                if pose is None:
                    resultats[nom] = {'erreur': f'TF {msg.header.frame_id} -> {BASE} introuvable'}
                    continue
                valeurs = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
                o = info.origin
                origine = (o.position.x, o.position.y,
                           lacet(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w))
                res = analyser_costmap(valeurs, info.resolution, origine, pose[:3], self._emprise)
                res['age'] = round(time.monotonic() - t, 1)
                resultats[nom] = res
            return resultats

        def _rapport(self):
            maintenant_mur, maintenant_ros = time.monotonic(), self._ros()
            self._f_gf[self._gf_etat] += maintenant_mur - self._gf_depuis
            self._gf_depuis = maintenant_mur
            duree_ros = maintenant_ros - self._f_debut_ros
            duree = duree_ros if args.sim and duree_ros > 0 else maintenant_mur - self._f_debut_mur
            duree = max(duree, 1e-3)
            types = dict(self.get_topic_names_and_types())
            chaine = {
                'brut': self._chaine['brut'] if self.count_publishers(self._chaine['brut']) else None,
                'sortie': self._chaine['sortie'] if self._chaine['sortie'] in types else None,
                'sortie_publiee': bool(self.count_publishers(self._chaine['sortie'])),
                'base': self._chaine['base'] if self.count_publishers(self._chaine['base']) else None,
            }
            cmd = {topic: stats_commandes(ech) for topic, ech in self._f_cmd.items()}
            for topic in AUTRES_COMMANDES:
                if topic in types and topic not in cmd:
                    cmd[topic] = None
            odom = None
            if self._f_odom:
                tab = np.array(self._f_odom)
                # Pose de l'odométrie en plus des vitesses : certaines bases laissent twist à 0.
                odom = {'n': len(self._f_odom), 'v': round(float(tab[:, 0].mean()), 3),
                        'w': round(float(tab[:, 1].mean()), 3),
                        'deplacement': round(float(np.hypot(tab[-1, 2] - tab[0, 2],
                                                            tab[-1, 3] - tab[0, 3])), 3),
                        'rotation': round(abs(math.degrees(ecart_angle(tab[-1, 4], tab[0, 4]))), 1)}
            nav_actif = bool(self._f_nav) and sum(self._f_nav) >= 0.5 * len(self._f_nav)
            fb = self._feedback or {}
            scans = {}
            for topic in SCANS:
                if topic in self._f_scans:
                    scans[topic] = {k: v for k, v in self._f_scans[topic].items()
                                    if not k.startswith('_')}
            debits = {}
            for topic in [self._chaine['brut'], self._chaine['sortie'], self._chaine['base'],
                          args.odom] + SCANS + list(COSTMAPS.values()) + DEBITS:
                if topic in types and topic not in debits:
                    debits[topic] = round(self._f_compte[topic] / duree, 1)
            manquants, muets = [], []
            if self._t() > 15.0:
                for topic in [args.odom, '/scan', '/scan_clean']:
                    if topic not in types or not self.count_publishers(topic):
                        manquants.append(topic)
                    elif self._f_compte[topic] == 0:
                        muets.append(topic)
            co = None
            if self._odom_controleur:
                co = {'topic': self._odom_controleur,
                      'publieurs': self.count_publishers(self._odom_controleur)}
            p = {
                't': round(self._t(), 1), 'ros': round(maintenant_ros, 2),
                'nav': {'actif': nav_actif, 'reste': f"{fb['reste']:.2f} m" if 'reste' in fb else '?',
                        'recoveries': fb.get('recoveries', 0)},
                'chaine': chaine, 'cmd': cmd, 'odom': odom,
                'garde_fou': {'durees': {k: round(v, 2) for k, v in self._f_gf.items() if v > 0},
                              'actuel': self._gf_etat},
                'sources_garde_fou': sorted(self._sources_garde_fou),
                'zones': {k: (v[0], v[1]) for k, v in self._zones.items()},
                'scans': scans, 'costmaps': self._analyse_costmaps(),
                'loc': dict(self._f_loc, pose=None), 'progres': self._progres_recent(),
                'progres_rayon': self._progres[0], 'progres_delai': self._progres[1],
                'bt': {'recoveries': self._f_recoveries, 'echecs': dict(self._f_echecs),
                       'recovery': bool(self._f_recoveries or self._bt_en_cours)},
                'plan': self._plan, 'debits': debits, 'manquants': manquants, 'muets': muets,
                'controleur_odom': co, 'conflits': list(self._conflits),
                'inactifs': [f'{n} : {e}' for n, e in self._cycle_vie.items() if e != 'active'],
                'temps': list(self._temps),
            }
            if self._historique:
                h = self._historique[-1]
                p['loc']['pose'] = (round(h[2], 2), round(h[3], 2), round(math.degrees(h[4])))
            verdicts = diagnostiquer(p)
            for ligne in fmt_rapport(p, verdicts):
                journal(ligne)
            p['verdicts'] = verdicts
            journal.mesure(p)
            self._periodes += 1
            self._periodes_nav += int(nav_actif)
            for cle, _ in verdicts:
                self._cumul_verdicts[cle] += 1
            self._reinitialiser_fenetre()
            if args.duree and self._t() >= args.duree:
                self.fini = True

        # ------------------------------------------------------------ graphe, nœuds
        def _controles(self):
            """Publieurs des commandes (conflits), odométrie du contrôleur, paramètres."""
            conflits = []
            for topic in (self._chaine['brut'], self._chaine['sortie']):
                noms = sorted({i.node_name for i in self.get_publishers_info_by_topic(topic)})
                if noms != self._publieurs.get(topic):
                    self._publieurs[topic] = noms
                    abonnes = sorted({i.node_name for i in self.get_subscriptions_info_by_topic(topic)})
                    self._evenement(f'Graphe : {topic} publié par {noms or "personne"}, '
                                    f'écouté par {abonnes or "personne"}')
                attendus = PUBLIEURS_ATTENDUS.get(topic, set())
                if topic == self._chaine['sortie'] and not self.count_publishers(self._chaine['brut']):
                    attendus = PUBLIEURS_ATTENDUS[TOPIC_BRUT]      # garde-fou désactivé
                autres = [n for n in noms if n not in attendus and not n.startswith('_')]
                if autres and attendus:
                    conflits.append(f"{topic} publié aussi par {', '.join(autres)}")
            self._conflits = conflits
            for noeud, noms in (
                    ('/controller_server', ['odom_topic', 'use_sim_time', 'controller_frequency',
                                            'FollowPath.plugin', 'progress_checker.required_movement_radius',
                                            'progress_checker.movement_time_allowance']),
                    ('/collision_monitor', ['use_sim_time', 'cmd_vel_in_topic', 'cmd_vel_out_topic',
                                            'polygons', 'observation_sources', 'source_timeout']),
                    ('/local_costmap/local_costmap', ['use_sim_time', 'footprint',
                                                      'footprint_padding', 'robot_radius']),
                    ('/bt_navigator', ['use_sim_time', 'default_nav_to_pose_bt_xml']),
                    ('/planner_server', ['use_sim_time']),
                    ('/slam_toolbox', ['use_sim_time', 'mode', 'map_file_name'])):
                if noeud not in self._params:
                    self._demander_params(noeud, noms)

        def _client(self, classe, service):
            if classe is None:
                return None
            if service not in self._clients_services:
                self._clients_services[service] = self.create_client(classe, service)
            return self._clients_services[service]

        def _demander_params(self, noeud, noms, suite=None):
            client = self._client(classes['GetParameters'], f'{noeud}/get_parameters')
            if client is None or not client.service_is_ready():
                return
            requete = classes['GetParameters'].Request()
            requete.names = list(noms)
            futur = client.call_async(requete)
            futur.add_done_callback(lambda f: self._params_recus(noeud, noms, f, suite))

        def _params_recus(self, noeud, noms, futur, suite):
            try:
                valeurs = futur.result().values
            except Exception:                    # noqa: BLE001
                return
            lus = {}
            for nom, v in zip(noms, valeurs):
                valeur = {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value,
                          6: list(v.bool_array_value), 7: list(v.integer_array_value),
                          8: list(v.double_array_value), 9: list(v.string_array_value)}.get(v.type)
                if valeur is not None:
                    lus[nom] = valeur
            self._params.setdefault(noeud, {}).update(lus)
            self._evenement(f'Paramètres {noeud} : ' + ', '.join(f'{k}={v}' for k, v in lus.items()),
                            console=suite is None)
            try:
                self._appliquer_params(noeud, lus, suite)
            except Exception as erreur:          # noqa: BLE001
                self._evenement(f'(diagnostic) paramètres {noeud} illisibles : {erreur!r}')

        def _appliquer_params(self, noeud, lus, suite):
            if 'use_sim_time' in lus and bool(lus['use_sim_time']) != args.sim:
                texte = (f"{noeud} : use_sim_time={lus['use_sim_time']} alors que /clock "
                         f"{'existe' if args.sim else 'est absent'}")
                if texte not in self._temps:
                    self._temps.append(texte)
            if noeud == '/controller_server':
                if lus.get('odom_topic'):
                    self._odom_controleur = lus['odom_topic']
                rayon = lus.get('progress_checker.required_movement_radius', self._progres[0])
                delai = lus.get('progress_checker.movement_time_allowance', self._progres[1])
                self._progres = (float(rayon), float(delai))
            elif noeud == '/local_costmap/local_costmap':
                poly = parse_polygone(lus['footprint']) if lus.get('footprint') else None
                if poly is None and lus.get('robot_radius'):
                    poly = cercle(float(lus['robot_radius']))
                if poly:
                    self._emprise = elargir(poly, float(lus.get('footprint_padding', 0.0)))
            elif noeud == '/collision_monitor' and suite is None:
                if lus.get('cmd_vel_in_topic'):
                    self._chaine['brut'] = lus['cmd_vel_in_topic']
                if lus.get('cmd_vel_out_topic'):
                    self._chaine['sortie'] = lus['cmd_vel_out_topic']
                polygones = list(lus.get('polygons') or [])
                sources = list(lus.get('observation_sources') or [])
                details = [f'{p}.{c}' for p in polygones for c in ('points', 'action_type', 'type',
                                                                    'radius', 'enabled')]
                details += [f'{s}.{c}' for s in sources for c in ('topic', 'type', 'enabled')]
                if details:
                    self._demander_params(noeud, details, suite=(polygones, sources))
            elif noeud == '/collision_monitor':
                polygones, sources = suite
                zones = {}
                for p in polygones:
                    if lus.get(f'{p}.enabled') is False:
                        continue
                    action = str(lus.get(f'{p}.action_type', 'stop'))
                    if lus.get(f'{p}.type') == 'circle' and lus.get(f'{p}.radius'):
                        poly = cercle(float(lus[f'{p}.radius']))
                    else:
                        poly = parse_polygone(lus.get(f'{p}.points', ''))
                    if poly:
                        zones[p] = (poly, action)
                if zones:
                    self._zones = zones
                topics = {lus[f'{s}.topic'] for s in sources
                          if lus.get(f'{s}.topic') and lus.get(f'{s}.enabled') is not False}
                if topics:
                    self._sources_garde_fou = {t if t.startswith('/') else '/' + t for t in topics}
                self._evenement(f"Garde-fou : sources {sorted(self._sources_garde_fou)}, zones "
                                + ', '.join(f'{k} ({v[1]})' for k, v in self._zones.items()))

        def _interroger_cycle_vie(self):
            for noeud in NOEUDS_CYCLE_VIE + ['/slam_toolbox']:
                client = self._client(classes['GetState'], f'{noeud}/get_state')
                if client is None or not client.service_is_ready():
                    continue
                futur = client.call_async(classes['GetState'].Request())
                futur.add_done_callback(lambda f, n=noeud: self._etat_recu(n, f))

        def _etat_recu(self, noeud, futur):
            try:
                etat = futur.result().current_state.label
            except Exception:                    # noqa: BLE001
                return
            if self._cycle_vie.get(noeud) != etat:
                self._cycle_vie[noeud] = etat
                self._evenement(f'Nœud {noeud} : {etat}', console=etat != 'active')

        # ------------------------------------------------------------ bilan
        def bilan(self):
            lignes = ['', '=' * 78,
                      f'BILAN : {self._t():.0f} s d\'écoute, {self._periodes} périodes de '
                      f'{args.periode:.0f} s dont {self._periodes_nav} en navigation']
            if self._cumul_recoveries:
                lignes.append('Recoveries de Nav2 : ' + ', '.join(
                    f'{k} ×{v}' for k, v in self._cumul_recoveries.most_common()))
            if self._cumul_echecs:
                lignes.append('Échecs : ' + ', '.join(
                    f'{k} ×{v}' for k, v in self._cumul_echecs.most_common()))
            etats = collections.Counter(ETATS_BUT.get(s, s) for s in self._buts.values())
            if etats:
                lignes.append('Buts Nav2 : ' + ', '.join(f'{k} ×{v}' for k, v in etats.items()))
            if self._sauts_total:
                lignes.append(f'Sauts de localisation : {self._sauts_total}')
            if self._cycle_vie:
                lignes.append('Nœuds : ' + ', '.join(f'{k} {v}' for k, v in self._cycle_vie.items()))
            frequents = [(cle, n) for cle, n in self._cumul_messages.most_common()
                         if cle in self._exemples_messages][:10]
            if frequents:
                lignes.append('Messages WARN/ERREUR les plus fréquents :')
                for cle, n in frequents:
                    niveau, texte = self._exemples_messages[cle]
                    lignes.append(f'  {n:5d} × {cle[0]} : {texte[:150]}')
            lignes.append('')
            lignes.append('PISTES (de la plus fréquente à la moins fréquente) :')
            # D'abord ce qui a été mesuré (durée), puis ce que disent les messages (nombre).
            pistes = sorted(((cle, n * args.periode, 'période')
                             for cle, n in self._cumul_verdicts.items()), key=lambda p: -p[1])
            pistes += sorted(((cle, n, 'message') for cle, n in self._cumul_indices.items()
                              if cle not in self._cumul_verdicts), key=lambda p: -p[1])
            if not pistes:
                lignes.append('  rien d\'anormal vu. Le robot a-t-il reçu un but pendant l\'écoute ?')
            for rang, (cle, quantite, sorte) in enumerate(pistes[:6], 1):
                titre, texte = EXPLICATIONS.get(cle, (cle, ''))
                mesure = f'{quantite:.0f} s' if sorte == 'période' else f'{quantite:.0f} messages'
                lignes.append(f'  {rang}. {titre} ({mesure})')
                if texte:
                    lignes.append(f'     {texte}')
            lignes.append('')
            lignes.append(f'Journal : {journal.chemin}')
            lignes.append(f'Mesures : {journal.chemin_mesures}')
            for ligne in lignes:
                journal(ligne)

    return Diagnostic()


# ------------------------------------------------------------------ principal
TOPICS_BAG = ['/scan', '/fosa/scan_filtered', '/scan_clean', '/scan_clear', '/top_camera_scan',
              '/bottom_camera_scan', '/tf', '/tf_static', TOPIC_ODOM, TOPIC_BRUT, TOPIC_SORTIE,
              TOPIC_BASE, '/plan', '/local_plan', '/local_costmap/costmap', '/map',
              '/behavior_tree_log', '/rosout',
              '/navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/status',
              '/fosa/tags_vus', '/parc/run_status', '/clock']


def topics_config():
    """Topics de la course lus dans caytu_config quand il est installé (fichier seul : défauts)."""
    try:
        from caytu_config import load_config
        course = load_config().course
        return {'garde_fou': course.safety_state_topic, 'statut': course.status_topic}
    except Exception:                        # noqa: BLE001 - copié seul sur le robot
        return {}


def detecter_simulation():
    import rclpy
    sonde = rclpy.create_node('fosa_diagnostic_sonde', enable_rosout=False,
                              start_parameter_services=False)
    fin = time.monotonic() + 3.0
    trouve = False
    while time.monotonic() < fin and not trouve:
        trouve = sonde.count_publishers('/clock') > 0
        rclpy.spin_once(sonde, timeout_sec=0.2)
    sonde.destroy_node()
    return trouve


def main(argv=None):
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('--duree', type=float, default=0.0, help='s, 0 = jusqu\'à Ctrl-C')
    lecteur.add_argument('--periode', type=float, default=5.0, help='s entre deux rapports')
    lecteur.add_argument('--sortie', default='', help='dossier des fichiers produits')
    lecteur.add_argument('--temps', choices=('auto', 'sim', 'reel'), default='auto')
    lecteur.add_argument('--odom', default=TOPIC_ODOM, help='odométrie du robot')
    config = topics_config()
    lecteur.add_argument('--topic-garde-fou', default=config.get('garde_fou', '/fosa/garde_fou_etat'),
                         help='état de nav2_collision_monitor (course.safety_state_topic)')
    lecteur.add_argument('--topic-statut', default=config.get('statut', '/fosa/course/statut'),
                         help='statut de la course (course.status_topic)')
    lecteur.add_argument('--bag', action='store_true',
                         help='enregistre aussi un bag léger (sans images) pour analyse')
    args, _ = lecteur.parse_known_args(argv)

    horodatage = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    dossier = dossier_sortie(args.sortie)
    journal = Journal(dossier, horodatage)
    journal(f'Diagnostic Fosa {horodatage} — écoute seule, Ctrl-C pour le bilan')
    for cle in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'FOSA_CONFIG_OVERRIDE',
                'FOSA_MAPS_DIR', 'FOSA_ROOT'):
        journal(f'  {cle}={os.environ.get(cle, "")}')

    import rclpy
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.init(args=None)
    except Exception as erreur:              # noqa: BLE001
        journal(f'ROS ne démarre pas : {erreur}')
        journal('Participant DDS refusé ? Sourcer env.sh (MaxAutoParticipantIndex) avant.')
        return 2
    args.sim = detecter_simulation() if args.temps == 'auto' else args.temps == 'sim'
    journal(f"  horloge : {'simulée (/clock)' if args.sim else 'réelle'} ; fichiers dans {dossier}")

    bag = None
    if args.bag:
        chemin_bag = os.path.join(dossier, f'bag_{horodatage}')
        try:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '--include-hidden-topics',
                                    '-o', chemin_bag] + TOPICS_BAG
                                   + [args.topic_garde_fou, args.topic_statut],
                                   stdout=open(os.path.join(dossier, f'bag_{horodatage}.txt'), 'w'),
                                   stderr=subprocess.STDOUT, start_new_session=True)
            journal(f'  bag : {chemin_bag}')
        except Exception as erreur:          # noqa: BLE001
            journal(f'  bag impossible : {erreur}')

    noeud = creer_noeud(args, journal)
    erreurs = set()
    try:
        while rclpy.ok() and not noeud.fini:
            try:
                rclpy.spin_once(noeud, timeout_sec=0.1)
            except (KeyboardInterrupt, ExternalShutdownException):
                raise
            except Exception as erreur:      # noqa: BLE001 - le diagnostic continue
                if repr(erreur) not in erreurs:
                    erreurs.add(repr(erreur))
                    journal(f'(diagnostic) erreur ignorée : {erreur!r}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            noeud.bilan()
        except Exception as erreur:          # noqa: BLE001
            journal(f'(bilan incomplet : {erreur!r})')
        if bag is not None:
            try:
                os.killpg(bag.pid, signal.SIGINT)
                bag.wait(timeout=15)
            except Exception:                # noqa: BLE001
                bag.kill()
        journal.fermer()
        try:
            noeud.destroy_node()
        except Exception:                    # noqa: BLE001
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
