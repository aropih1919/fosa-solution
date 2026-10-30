"""Calculs purs de la solution Fosa (aucune dépendance ROS).

Tout ce qui est ici se teste avec `pytest`, sans simulateur :
  - poses 2D et changements de repère ;
  - fusion « distance des roues + cap de l'IMU » (localisation) ;
  - inclinaison du robot et distance à laquelle le plan du lidar coupe le sol ;
  - lecture d'une carte PGM et comparaison d'un scan avec cette carte.

Règle de nommage : les mots du métier sont en français, les termes techniques
ROS/géométrie restent en anglais (Pose2D, yaw, scan, PGM...).
"""

import math

# Accélération de la pesanteur utilisée par l'accéléromètre (m/s²).
GRAVITE = 9.80665


# --------------------------------------------------------------------------- #
# Angles et poses 2D
# --------------------------------------------------------------------------- #
def normaliser_angle(angle):
    """Ramène un angle dans ]-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def lacet_depuis_quaternion(x, y, z, w):
    """Lacet (rotation autour de z) d'un quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_depuis_lacet(lacet):
    """Quaternion (x, y, z, w) d'une rotation pure autour de z."""
    return 0.0, 0.0, math.sin(lacet / 2.0), math.cos(lacet / 2.0)


def quaternion_depuis_roulis_tangage_lacet(roulis, tangage, lacet):
    """Quaternion (x, y, z, w) pour des angles roulis/tangage/lacet (ROS)."""
    cos_roulis = math.cos(roulis / 2.0)
    sin_roulis = math.sin(roulis / 2.0)
    cos_tangage = math.cos(tangage / 2.0)
    sin_tangage = math.sin(tangage / 2.0)
    cos_lacet = math.cos(lacet / 2.0)
    sin_lacet = math.sin(lacet / 2.0)
    return (
        sin_roulis * cos_tangage * cos_lacet - cos_roulis * sin_tangage * sin_lacet,
        cos_roulis * sin_tangage * cos_lacet + sin_roulis * cos_tangage * sin_lacet,
        cos_roulis * cos_tangage * sin_lacet - sin_roulis * sin_tangage * cos_lacet,
        cos_roulis * cos_tangage * cos_lacet + sin_roulis * sin_tangage * sin_lacet,
    )


class Pose2D:
    """Pose plane : position (m) et cap (rad).

    On ne modifie jamais une pose existante : chaque calcul crée une nouvelle
    Pose2D. C'est plus simple à relire et cela évite les effets de bord.
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def composer(self, autre):
        """Retourne self ∘ autre (autre exprimé dans le repère de self)."""
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Pose2D(
            self.x + cos_yaw * autre.x - sin_yaw * autre.y,
            self.y + sin_yaw * autre.x + cos_yaw * autre.y,
            normaliser_angle(self.yaw + autre.yaw),
        )

    def inverse(self):
        """Retourne la pose inverse : le changement de repère opposé."""
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Pose2D(
            -(cos_yaw * self.x + sin_yaw * self.y),
            -(-sin_yaw * self.x + cos_yaw * self.y),
            normaliser_angle(-self.yaw),
        )

    def distance_vers(self, x, y):
        """Distance en ligne droite entre cette pose et le point (x, y)."""
        return math.hypot(x - self.x, y - self.y)


# --------------------------------------------------------------------------- #
# Localisation : distance des roues + cap de l'IMU
# --------------------------------------------------------------------------- #
class FusionOdomImu:
    """Estime la pose du robot dans `map` (= repère du monde Gazebo).

    Hypothèses, toutes vérifiées dans le dépôt officiel PARC :
      - l'odométrie du plugin DiffDrive démarre à (0, 0, 0) au point de départ ;
      - le départ (x, y, yaw) est donné dans task_params.yaml, en repère Gazebo ;
      - la distance parcourue par les roues est fiable, mais le CAP de cette
        odométrie ne l'est pas forcément (entraxe déclaré 0.3409 m pour des
        roues espacées de 0.393 m). Le cap est donc pris sur l'IMU.

    `mettre_a_jour()` renvoie la transformation map -> odom à publier sur /tf :
    c'est exactement le rôle qu'avait AMCL, sans carte ni filtre à particules,
    donc sans « localisation perdue ».
    """

    def __init__(self, depart, pas_max=1.0):
        self._depart = depart
        self._pas_max = pas_max
        self._pose = None
        self._derniere_odom = None
        self._decalage_imu = None

    def get_pose(self):
        """Dernière pose estimée dans `map` (None avant la première odométrie)."""
        return self._pose

    def utilise_imu(self):
        """Vrai si le cap courant vient de l'IMU."""
        return self._decalage_imu is not None

    def transformation_initiale(self):
        """map -> odom avant toute mesure : l'origine d'odom est le départ."""
        return self._depart

    def mettre_a_jour(self, odom, lacet_imu):
        """Intègre une mesure d'odométrie (et le cap IMU s'il est disponible).

        odom     : pose base_footprint dans `odom`, telle que publiée par Gazebo.
        lacet_imu: lacet absolu de l'IMU (rad), ou None si l'IMU est muette.
        Retourne map -> odom.
        """
        # Premier appel : on place l'origine d'odom sur le point de départ.
        if self._pose is None or self._derniere_odom is None:
            self._pose = self._depart.composer(odom)
            self._derniere_odom = odom
            self._decalage_imu = None
            if lacet_imu is not None:
                self._decalage_imu = normaliser_angle(self._pose.yaw - lacet_imu)
            return self._pose.composer(odom.inverse())

        dx = odom.x - self._derniere_odom.x
        dy = odom.y - self._derniere_odom.y
        dyaw_odom = normaliser_angle(odom.yaw - self._derniere_odom.yaw)

        # Un saut d'odométrie (réinitialisation du simulateur) n'est pas un
        # déplacement : on le saute pour ne pas fausser la pose.
        if math.hypot(dx, dy) > self._pas_max:
            self._derniere_odom = odom
            return self._pose.composer(odom.inverse())

        # Distance signée le long de l'axe du robot (base différentielle).
        cap_milieu_odom = self._derniere_odom.yaw + 0.5 * dyaw_odom
        distance_parcourue = (dx * math.cos(cap_milieu_odom)
                              + dy * math.sin(cap_milieu_odom))

        if lacet_imu is not None:
            if self._decalage_imu is None:
                # L'IMU (ré)apparaît : on la recale sur le cap courant, sans saut.
                self._decalage_imu = normaliser_angle(self._pose.yaw - lacet_imu)
            nouveau_cap = normaliser_angle(lacet_imu + self._decalage_imu)
        else:
            self._decalage_imu = None
            nouveau_cap = normaliser_angle(self._pose.yaw + dyaw_odom)

        cap_milieu = self._pose.yaw + 0.5 * normaliser_angle(nouveau_cap - self._pose.yaw)
        self._pose = Pose2D(
            self._pose.x + distance_parcourue * math.cos(cap_milieu),
            self._pose.y + distance_parcourue * math.sin(cap_milieu),
            nouveau_cap,
        )
        self._derniere_odom = odom
        return self._pose.composer(odom.inverse())


# --------------------------------------------------------------------------- #
# Inclinaison du robot et intersection du plan lidar avec le sol
# --------------------------------------------------------------------------- #
def inclinaison_depuis_acceleration(ax, ay, az):
    """Roulis et tangage (rad) d'un capteur immobile, lus sur l'accéléromètre.

    Convention ROS : tangage positif = nez vers le bas. Au repos, le capteur
    mesure la réaction à la gravité, soit g * (-sin p, cos p sin r, cos p cos r).
    """
    roulis = math.atan2(ay, az)
    tangage = math.atan2(-ax, math.hypot(ay, az))
    return roulis, tangage


def hauteur_lidar(tangage_robot, hauteur_fixation, bras_x):
    """Hauteur du lidar au-dessus du sol quand le robot pivote sur ses roues.

    hauteur_fixation : hauteur du lidar dans base_footprint (robot à plat).
    bras_x           : distance horizontale entre l'axe des roues et le lidar
                       (positive si le lidar est derrière l'axe : il monte quand
                       le nez descend).
    """
    return (hauteur_fixation * math.cos(tangage_robot)
            + bras_x * math.sin(tangage_robot))


def distances_sol(angles, tangage_plan, roulis_plan, hauteur):
    """Distance à laquelle chaque rayon du lidar touche le sol (inf sinon).

    angles       : angle de chaque rayon dans le repère du lidar (0 = avant).
    tangage_plan : tangage du plan de balayage dans le monde (positif = l'avant
                   du plan plonge vers le sol).
    roulis_plan  : roulis du plan de balayage.
    hauteur      : hauteur du lidar au-dessus du sol.
    """
    sin_tangage = math.sin(tangage_plan)
    cos_tangage = math.cos(tangage_plan)
    sin_roulis = math.sin(roulis_plan)

    distances = []
    for angle in angles:
        # Composante verticale du rayon : négative = le rayon descend.
        composante_verticale = (-sin_tangage * math.cos(angle)
                                + cos_tangage * sin_roulis * math.sin(angle))
        if composante_verticale < -1e-6:
            distances.append(hauteur / -composante_verticale)
        else:
            distances.append(math.inf)
    return distances


def separer_retours_sol(distances, distances_sol_attendues, portee_max,
                        ratio_sol=0.8, ratio_effacement=0.7):
    """Sépare obstacles réels et retours du sol.

    Retourne (marquage, effacement, nombre de retours de sol) :
      - marquage   : le scan sans les retours du sol (remplacés par inf) ;
      - effacement : le scan où un retour de sol devient une distance « libre
        jusque-là » (ratio_effacement * distance du sol), pour que la costmap
        puisse encore nettoyer la zone devant le robot.

    Un retour est attribué au sol s'il est au moins à ratio_sol fois la
    distance théorique du sol dans cette direction : au-delà, le rayon serait
    sous le plancher, il ne peut donc s'agir d'un obstacle.
    """
    marquage = []
    effacement = []
    nombre_sol = 0

    for distance, distance_sol_attendue in zip(distances, distances_sol_attendues):
        valide = math.isfinite(distance) and distance <= portee_max
        if (valide and math.isfinite(distance_sol_attendue)
                and distance >= ratio_sol * distance_sol_attendue):
            nombre_sol += 1
            marquage.append(math.inf)
            effacement.append(min(ratio_effacement * distance_sol_attendue, portee_max))
        else:
            marquage.append(distance)
            effacement.append(distance)

    return marquage, effacement, nombre_sol


# --------------------------------------------------------------------------- #
# Carte statique : lecture PGM et contrôle de cohérence avec un scan
# --------------------------------------------------------------------------- #
class CarteGrille:
    """Carte d'occupation minimale (ligne 0 = y minimal)."""

    def __init__(self, largeur, hauteur, resolution, origine_x, origine_y, occupee):
        self.largeur = largeur
        self.hauteur = hauteur
        self.resolution = resolution
        self.origine_x = origine_x
        self.origine_y = origine_y
        # occupee[ligne][colonne] = 1 si la cellule est occupée.
        self.occupee = occupee

    def est_occupee_pres(self, x, y, rayon_cases):
        """Vrai si une cellule occupée est à moins de rayon_cases de (x, y)."""
        colonne = int(math.floor((x - self.origine_x) / self.resolution))
        ligne = int(math.floor((y - self.origine_y) / self.resolution))

        for ligne_test in range(ligne - rayon_cases, ligne + rayon_cases + 1):
            if not 0 <= ligne_test < self.hauteur:
                continue
            ligne_occupee = self.occupee[ligne_test]
            debut = max(0, colonne - rayon_cases)
            fin = min(self.largeur, colonne + rayon_cases + 1)
            for colonne_test in range(debut, fin):
                if ligne_occupee[colonne_test]:
                    return True
        return False


def lire_jetons_pgm(donnees, nombre):
    """Lit `nombre` jetons d'en-tête PGM en sautant blancs et commentaires.

    Un fichier PGM binaire commence par un en-tête en texte :
        P5
        largeur hauteur
        valeur_maximale
    puis vient un octet par pixel. Cette fonction lit les jetons de l'en-tête
    et renvoie aussi la position du premier octet de l'image.
    """
    jetons = []
    position = 0

    while len(jetons) < nombre:
        # Sauter les espaces et les retours à la ligne.
        while position < len(donnees) and donnees[position:position + 1].isspace():
            position += 1

        # Sauter un commentaire qui commence par # jusqu'à la fin de la ligne.
        if donnees[position:position + 1] == b'#':
            while position < len(donnees) and donnees[position:position + 1] != b'\n':
                position += 1
            continue

        debut = position
        while position < len(donnees) and not donnees[position:position + 1].isspace():
            position += 1
        jetons.append(donnees[debut:position])

    # Un seul blanc sépare l'en-tête des données binaires.
    return jetons, position + 1


def charger_carte_pgm(donnees_pgm, resolution, origine_x, origine_y, seuil_occupe=90):
    """Charge un PGM binaire (P5, 8 bits) : pixel sombre = occupé (mode trinary)."""
    jetons, position_image = lire_jetons_pgm(donnees_pgm, 4)

    if jetons[0] != b'P5':
        raise ValueError('format PGM non supporté (P5 attendu)')

    largeur = int(jetons[1])
    hauteur = int(jetons[2])
    valeur_max = int(jetons[3])

    if valeur_max > 255:
        raise ValueError('PGM 16 bits non supporté')

    pixels = donnees_pgm[position_image:position_image + largeur * hauteur]
    if len(pixels) != largeur * hauteur:
        raise ValueError('PGM tronqué')

    lignes = []
    # Dans un PGM, la première ligne est le y maximal : on inverse l'ordre pour
    # avoir ligne 0 = y minimal, comme le veut la carte d'occupation ROS.
    for indice_ligne in range(hauteur - 1, -1, -1):
        debut = indice_ligne * largeur
        ligne_pixels = pixels[debut:debut + largeur]
        lignes.append(bytearray(1 if pixel < seuil_occupe else 0
                                for pixel in ligne_pixels))

    return CarteGrille(largeur, hauteur, resolution, origine_x, origine_y, lignes)


def accord_scan_carte(carte, pose_lidar, angle_min, increment_angle, distances,
                      portee_min=0.45, portee_max=11.5, tolerance_cases=4):
    """Part des retours du scan qui tombent sur un mur de la carte.

    carte      : CarteGrille chargée depuis le PGM.
    pose_lidar : pose du lidar dans le repère de la carte.
    Retourne (fraction en accord, nombre de retours utilisés). Les retours trop
    proches (le robot lui-même) ou hors portée sont ignorés.
    """
    utilises = 0
    touches = 0

    for indice, distance in enumerate(distances):
        if not math.isfinite(distance) or distance < portee_min or distance > portee_max:
            continue

        angle = pose_lidar.yaw + angle_min + indice * increment_angle
        x = pose_lidar.x + distance * math.cos(angle)
        y = pose_lidar.y + distance * math.sin(angle)
        utilises += 1

        if carte.est_occupee_pres(x, y, tolerance_cases):
            touches += 1

    if utilises == 0:
        return 0.0, 0
    return touches / utilises, utilises
