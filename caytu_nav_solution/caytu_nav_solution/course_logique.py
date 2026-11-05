"""Course de la phase finale : calculs purs, sans ROS (testés par pytest).

- lecture de l'ordre des tags publié par les organisateurs, quel que soit
  le type du message ;
- géométrie de la visite : pose à 0,5 m face au tag, écarts mesurés ;
- loi de commande de l'approche finale ;
- choix des points d'exploration quand un tag n'a pas encore été vu ;
- garde-fou lidar devant (ou derrière) le robot.
"""

import math
import re

from caytu_nav_solution.nav_math import Pose2D, normaliser_angle


# --------------------------------------------------------------- ordre des tags
def extraire_ordre(message, identifiants_attendus):
    """Liste ordonnée des tags à visiter, tirée d'un message quelconque.

    Accepte std_msgs/String (« 7,1,9,4 », « [7, 1, 9, 4] », « tags: 7 1 9 4 »),
    les tableaux d'entiers (Int32MultiArray, UInt8MultiArray...), un entier
    seul, ou un message dont un champ contient ces valeurs. Les identifiants
    inconnus et les doublons sont ignorés.
    """
    valeurs = _valeurs_entieres(message)
    attendus = set(identifiants_attendus)
    ordre = []
    for valeur in valeurs:
        if valeur in attendus and valeur not in ordre:
            ordre.append(valeur)
    return ordre


def _valeurs_entieres(objet, profondeur=0):
    if profondeur > 4 or objet is None:
        return []
    if isinstance(objet, bool):
        return []
    if isinstance(objet, int):
        return [objet]
    if isinstance(objet, float):
        return [int(objet)] if float(objet).is_integer() else []
    if isinstance(objet, str):
        return [int(texte) for texte in re.findall(r'-?\d+', objet)]
    if isinstance(objet, (bytes, bytearray)):
        return list(objet)
    if isinstance(objet, (list, tuple)) or hasattr(objet, 'tolist'):
        resultat = []
        for element in (objet.tolist() if hasattr(objet, 'tolist') else objet):
            resultat.extend(_valeurs_entieres(element, profondeur + 1))
        return resultat
    # Message ROS : on parcourt ses champs (data, ids, tags...) sauf l'en-tête.
    champs = getattr(objet, 'get_fields_and_field_types', None)
    if champs is not None:
        resultat = []
        for nom in champs():
            if nom in ('header', 'layout', 'stamp'):
                continue
            resultat.extend(_valeurs_entieres(getattr(objet, nom), profondeur + 1))
        return resultat
    return []


# ------------------------------------------------------------- géométrie
def pose_devant_tag(tag_x, tag_y, lacet_normale, distance):
    """Pose du robot à `distance` du tag, du côté libre, face au tag."""
    return Pose2D(tag_x + distance * math.cos(lacet_normale),
                  tag_y + distance * math.sin(lacet_normale),
                  normaliser_angle(lacet_normale + math.pi))


def ecarts_visite(robot, tag_x, tag_y, lacet_normale, distance_voulue):
    """(écart de distance au tag, écart d'alignement en degrés, écart latéral).

    L'alignement compare le cap du robot à la direction opposée à la normale :
    c'est « face au tag, aligné sur sa surface ».
    """
    distance = math.hypot(robot.x - tag_x, robot.y - tag_y)
    alignement = normaliser_angle(robot.yaw - (lacet_normale + math.pi))
    # Position du robot dans le repère du tag (x : le long de la normale).
    dx, dy = robot.x - tag_x, robot.y - tag_y
    lateral = -dx * math.sin(lacet_normale) + dy * math.cos(lacet_normale)
    return distance - distance_voulue, math.degrees(alignement), lateral


def commande_approche(robot, cible, vitesse_max, rotation_max,
                      tolerance_position=0.02, tolerance_cap=math.radians(2.0)):
    """Commande (v, w, terminé) pour rejoindre `cible` (Pose2D) lentement.

    Trois temps : se tourner vers le point, y aller en corrigeant le cap,
    puis tourner sur place jusqu'au cap final. La marche arrière est permise
    pour les petites corrections (moins de 0,25 m) : le robot ne fait pas
    demi-tour devant le tag.
    """
    dx, dy = cible.x - robot.x, cible.y - robot.y
    distance = math.hypot(dx, dy)
    if distance > tolerance_position:
        direction = math.atan2(dy, dx)
        erreur = normaliser_angle(direction - robot.yaw)
        sens = 1.0
        if abs(erreur) > math.pi / 2.0 and distance < 0.25:
            sens = -1.0
            erreur = normaliser_angle(erreur + math.pi)
        if abs(erreur) > math.radians(20.0):
            return 0.0, _borne(1.5 * erreur, rotation_max), False
        vitesse = sens * min(vitesse_max, max(0.03, 0.8 * distance))
        return vitesse, _borne(1.5 * erreur, rotation_max), False
    erreur_cap = normaliser_angle(cible.yaw - robot.yaw)
    if abs(erreur_cap) > tolerance_cap:
        rotation = _borne(1.2 * erreur_cap, rotation_max)
        if abs(rotation) < 0.08:
            rotation = math.copysign(0.08, rotation)
        return 0.0, rotation, False
    return 0.0, 0.0, True


def commande_alignement(robot, tag_x, tag_y, lacet_normale, distance_voulue, vitesse_max,
                        rotation_max, tolerance_distance, tolerance_cap):
    """Dernier réglage devant le tag, une fois à quelques centimètres de la pose visée.

    commande_approche vise un point : tout près de lui, sa direction change à
    chaque vue du tag (1-2 cm de bruit) et le robot hésite sans fin entre
    tourner vers le point et tourner vers le tag. Ici, plus de point : le
    robot tourne sur place pour faire face au tag, puis avance ou recule en
    ligne droite pour régler la distance. Seuls comptent la distance au tag et
    l'alignement, comme pour la visite (ecarts_visite).
    """
    ecart_distance, ecart_cap, _ = ecarts_visite(robot, tag_x, tag_y, lacet_normale,
                                                 distance_voulue)
    ecart_cap = math.radians(ecart_cap)
    if abs(ecart_cap) > tolerance_cap:
        rotation = _borne(-1.2 * ecart_cap, rotation_max)
        if abs(rotation) < 0.08:
            rotation = math.copysign(0.08, rotation)
        return 0.0, rotation, False
    if abs(ecart_distance) > tolerance_distance:
        vitesse = math.copysign(min(vitesse_max, max(0.02, 0.8 * abs(ecart_distance))),
                                ecart_distance)
        return vitesse, _borne(-1.2 * ecart_cap, rotation_max), False
    return 0.0, 0.0, True


def _borne(valeur, limite):
    return max(-limite, min(limite, valeur))


# ------------------------------------------------------------ garde-fou lidar
def distance_libre(points_robot, demi_largeur, avant=True,
                   bord_avant=0.22, bord_arriere=0.31):
    """Distance libre devant (ou derrière) le pare-chocs, en mètres.

    points_robot : points (x, y) du lidar dans base_footprint. Seuls les points
    dans le couloir de la largeur du robot comptent.
    """
    libre = float('inf')
    for x, y in points_robot:
        if abs(y) > demi_largeur:
            continue
        if avant and x > bord_avant:
            libre = min(libre, x - bord_avant)
        elif not avant and x < -bord_arriere:
            libre = min(libre, -bord_arriere - x)
    return libre


# --------------------------------------------------------------- exploration
def points_exploration(grille, largeur, hauteur, resolution, origine_x, origine_y,
                       pas=0.75, degagement=0.45):
    """Points libres de la carte, espacés de `pas`, à `degagement` des obstacles.

    grille : valeurs OccupancyGrid (-1 inconnu, 0 libre, 100 occupé), ligne 0
    en bas. Sert à chercher un tag pas encore vu.
    """
    occupees = [(i % largeur, i // largeur) for i, valeur in enumerate(grille)
                if valeur >= 50]
    rayon = int(math.ceil(degagement / resolution))
    bloquees = set()
    for colonne, ligne in occupees:
        for dc in range(-rayon, rayon + 1):
            for dl in range(-rayon, rayon + 1):
                if dc * dc + dl * dl <= rayon * rayon:
                    bloquees.add((colonne + dc, ligne + dl))
    saut = max(1, int(round(pas / resolution)))
    points = []
    for ligne in range(saut // 2, hauteur, saut):
        for colonne in range(saut // 2, largeur, saut):
            if grille[ligne * largeur + colonne] != 0 or (colonne, ligne) in bloquees:
                continue
            points.append((origine_x + (colonne + 0.5) * resolution,
                           origine_y + (ligne + 0.5) * resolution))
    return points


def choisir_point_exploration(points, robot, deja_vus, rayon_vu=1.2):
    """Le point le plus loin des endroits déjà balayés, puis le plus proche."""
    candidats = [p for p in points
                 if all(math.hypot(p[0] - v[0], p[1] - v[1]) > rayon_vu for v in deja_vus)]
    if not candidats:
        return None
    return min(candidats, key=lambda p: math.hypot(p[0] - robot.x, p[1] - robot.y))
