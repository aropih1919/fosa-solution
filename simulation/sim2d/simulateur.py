#!/usr/bin/env python3
"""Simulateur 2D léger du café PARC, avec 4 AprilTags, pour essayer toute la
solution sans Gazebo ni robot.

    python3 simulation/sim2d/simulateur.py                  # interfaces du Gazebo PARC (robot:=sim)
    python3 simulation/sim2d/simulateur.py --reel           # interfaces du vrai robot (robot:=real)
    python3 simulation/sim2d/simulateur.py --personne       # une personne traverse devant le départ
    python3 simulation/sim2d/simulateur.py --sans-tag 9     # le tag 9 n'est pas dans l'arène

Monde : murs de caytu_nav_bringup/maps/cafe_map.pgm, pieds de tables absents de
cette carte, 4 panneaux portant les tags 1, 4, 7, 9 à 1 m du sol.

Publie, comme le Gazebo PARC (défaut) : /clock, /scan (lidar_link), /odom et TF
odom -> base_footprint (cap faussé de 15 % comme le DiffDrive de Gazebo), /imu,
/top_camera_color/image_raw + camera_info (tags dessinés en perspective,
masqués par les murs), /top_camera_depth/points, /bottom_camera_depth/points.
Écoute /robot_base_controller/cmd_vel_unstamped.

Avec --reel, comme le vrai robot (relevé officiel du 2 novembre) : horloge
réelle, /botwheel_explorer/odom et TF odom -> base_link, /scan (laser), pas
d'IMU, /camera1/camera1/color/image_raw + camera_info (camera1 = caméra du
haut, comme le simulateur officiel du 5 novembre), images de profondeur
/camera1 et /camera2 (camera2 = caméra du bas). Écoute /cmd_vel.

Vérité terrain, pour simulation/outils/verifier_course.py :
/sitoe_robot/pose (comme le Gazebo PARC), /sim/true_pose, /sim/tags (JSON
{id: [x, y, cap]}), /sim/obstacles (JSON {nom: [x, y, rayon]}). Chaque contact
du robot avec un mur, un panneau ou la personne est écrit « CONTACT ».
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

DEPOT = Path(__file__).resolve().parents[2]
CARTE_MONDE = DEPOT / 'caytu_nav_bringup' / 'maps' / 'cafe_map.pgm'
RES_CARTE, OX, OY = 0.05, -5.5, -11.8           # cafe_map.yaml
SPAWN = (-0.200546, -7.485170, 1.571)            # simulation/sim2d/task_params.yaml
TABLES = [(1.8, -8.0), (1.6, 0.5), (1.5, 4.0), (-1.8, -3.0), (3.0, -8.8)]
LIDAR_X, LIDAR_Z = -0.021, 0.0481
ECHELLE_CAP_ODOM = 1.153                         # odométrie Gazebo trop « tournante »
EMPREINTE = ((-0.288, 0.198), (-0.222, 0.222))   # x min/max, y min/max (m)
# id : (x, y, cap de la face) — panneaux autour de la zone de départ.
TAGS = {
    1: (2.6, -6.0, math.pi),
    4: (-2.8, -5.0, 0.0),
    7: (0.5, -3.2, -math.pi / 2.0),
    9: (-1.5, -9.8, math.pi / 2.0),
}
TAILLE_TAG_DEFAUT = 0.18    # perception.apriltag.size_m de fosa.yaml (simulation)
HAUTEUR_TAG = 1.0
W, H, F = 640, 480, 460.0    # caméra couleur ~69°
CAM_X, CAM_Z = 0.20, 1.01
# Personne : va et vient devant le départ, sur le chemin des tags du nord, à
# plus d'un mètre des poses d'attente et de visite des tags.
PERSONNE_DE, PERSONNE_A, PERSONNE_RAYON = (-1.2, -6.6), (0.8, -6.6), 0.25
PERSONNE_PATIENCE = 3.0      # s : bloquée plus longtemps, elle fait demi-tour


def taille_tag():
    """Côté du carré noir lu par le détecteur : celui de caytu_config (0,18 m en
    simulation, 0,165 m avec robot_reel.yaml), pour que les distances soient justes."""
    try:
        from caytu_config import load_config
        return load_config().perception.apriltag.size_m
    except Exception:
        return TAILLE_TAG_DEFAUT


def stamp(t):
    message = TimeMsg()
    message.sec = int(t)
    message.nanosec = int((t - int(t)) * 1e9)
    return message


def quat(lacet):
    return 0.0, 0.0, math.sin(lacet / 2), math.cos(lacet / 2)


class Simulateur(Node):
    def __init__(self, args):
        super().__init__('simulateur_2d')
        self.reel = args.reel
        self.base = 'base_link' if self.reel else 'base_footprint'
        self.laser = 'laser' if self.reel else 'lidar_link'
        self.optique = ('camera1_color_optical_frame' if self.reel
                        else 'top_camera_color_optical_frame')
        self.tags = {i: v for i, v in TAGS.items() if i not in args.sans_tag}
        self.taille_tag = taille_tag()
        self.personne = None
        if args.personne:
            self.personne = {'x': PERSONNE_DE[0], 'y': PERSONNE_DE[1], 'sens': 1.0,
                             'vitesse': args.vitesse_personne, 'pause': 0.0, 'bloquee': 0.0}

        self._construire_monde()
        dictionnaire = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.motifs = {}
        for identifiant in self.tags:
            if hasattr(cv2.aruco, 'generateImageMarker'):           # OpenCV >= 4.7
                noir = cv2.aruco.generateImageMarker(dictionnaire, identifiant, 200, borderBits=1)
            else:                                                   # OpenCV 4.6 d'Ubuntu
                noir = cv2.aruco.drawMarker(dictionnaire, identifiant, 200, borderBits=1)
            papier = np.full((300, 300), 255, np.uint8)
            papier[50:250, 50:250] = noir
            self.motifs[identifiant] = papier

        self.angles_lidar = -math.pi + np.arange(540) * (2 * math.pi / 540)
        self.pas = np.arange(0.05, 12.0, 0.01)
        self.rng = np.random.default_rng(1)
        self.t0 = time.monotonic()
        self.x, self.y, self.lacet = SPAWN
        self.v = self.w = self.w_reel = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.ox = self.oy = self.olacet = 0.0
        self.contacts = 0
        self.en_contact = False
        self.dernier = time.monotonic()

        odom = '/botwheel_explorer/odom' if self.reel else '/odom'
        commande = '/cmd_vel' if self.reel else '/robot_base_controller/cmd_vel_unstamped'
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, odom, 10)
        self.pub_vraie = self.create_publisher(PoseStamped, '/sim/true_pose', 10)
        self.pub_gz = self.create_publisher(Pose, '/sitoe_robot/pose', 10)
        self.pub_tags = self.create_publisher(String, '/sim/tags', 10)
        self.pub_obstacles = self.create_publisher(String, '/sim/obstacles', 10)
        prefixe = '/camera1/camera1/color' if self.reel else '/top_camera_color'
        self.pub_image = self.create_publisher(Image, f'{prefixe}/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, f'{prefixe}/camera_info', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_statique = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, commande, self.recevoir_commande, 10)
        if self.reel:
            self.profondeurs = [
                (self.create_publisher(Image, f'/{c}/{c}/depth/image_rect_raw', 10),
                 self.create_publisher(CameraInfo, f'/{c}/{c}/depth/camera_info', 10),
                 f'{c}_depth_optical_frame')
                for c in ('camera1', 'camera2')]
            self.create_timer(0.1, self.publier_profondeurs)
        else:
            self.pub_horloge = self.create_publisher(Clock, '/clock', 10)
            self.pub_imu = self.create_publisher(Imu, '/imu', 10)
            self.pub_nuage_haut = self.create_publisher(PointCloud2, '/top_camera_depth/points', 10)
            self.pub_nuage_bas = self.create_publisher(PointCloud2, '/bottom_camera_depth/points', 10)
            self.create_timer(0.01, self.publier_horloge)
            self.create_timer(0.01, self.publier_imu)
            self.create_timer(0.2, self.publier_nuages)
        self.create_timer(0.02, self.avancer)
        self.create_timer(0.1, self.publier_scan)
        self.create_timer(0.2, self.publier_image)
        self.create_timer(0.2, self.publier_obstacles)
        self.create_timer(1.0, lambda: self.pub_tags.publish(String(data=json.dumps(self.tags))))
        self.publier_statiques()
        self.get_logger().info(
            f'Simulateur 2D prêt : interfaces {"du vrai robot" if self.reel else "du Gazebo PARC"}, '
            f'tags {sorted(self.tags)} de {self.taille_tag:.3f} m, personne {"oui" if self.personne else "non"}.')

    # ------------------------------------------------------------- monde
    def _construire_monde(self):
        donnees = CARTE_MONDE.read_bytes()
        morceaux = donnees.split(b'\n', 3)
        largeur, hauteur = map(int, morceaux[1].split())
        pixels = np.frombuffer(morceaux[3][:largeur * hauteur], dtype=np.uint8)
        occupe = pixels.reshape(hauteur, largeur)[::-1] < 90        # ligne 0 = y min
        self.res = 0.01
        grille = np.kron(occupe, np.ones((5, 5), dtype=bool))
        for tx, ty in TABLES:                       # 4 pieds de 5 cm par table
            for dx in (-0.35, 0.35):
                for dy in (-0.35, 0.35):
                    c0, r0 = int((tx + dx - OX) / self.res), int((ty + dy - OY) / self.res)
                    rr, cc = np.ogrid[-4:5, -4:5]
                    grille[r0 - 4:r0 + 5, c0 - 4:c0 + 5] |= rr * rr + cc * cc <= 16
        for x, y, cap in self.tags.values():        # panneaux 40 × 4 cm derrière le tag
            nx, ny = math.cos(cap), math.sin(cap)
            for s in np.arange(-0.20, 0.2001, 0.005):
                for e in np.arange(0.0, 0.0401, 0.005):
                    px, py = x - ny * s - nx * e, y + nx * s - ny * e
                    grille[int((py - OY) / self.res), int((px - OX) / self.res)] = True
        self.grille = grille
        self.h, self.l = grille.shape

    def maintenant(self):
        if self.reel:
            return time.time()
        return 100.0 + (time.monotonic() - self.t0)

    def lancer_rayons(self, x, y, angles, portee_max):
        """Distance au premier obstacle (murs, panneaux, personne) le long de chaque angle."""
        xs = x + np.outer(np.cos(angles), self.pas)
        ys = y + np.outer(np.sin(angles), self.pas)
        colonnes = ((xs - OX) / self.res).astype(int)
        lignes = ((ys - OY) / self.res).astype(int)
        dedans = (lignes >= 0) & (lignes < self.h) & (colonnes >= 0) & (colonnes < self.l)
        touche = np.zeros_like(dedans)
        touche[dedans] = self.grille[lignes[dedans], colonnes[dedans]]
        premier = np.where(touche.any(axis=1), touche.argmax(axis=1), -1)
        distances = np.where(premier >= 0, self.pas[np.maximum(premier, 0)], np.inf)
        if self.personne:
            dx, dy = x - self.personne['x'], y - self.personne['y']
            b = np.cos(angles) * dx + np.sin(angles) * dy
            c = dx * dx + dy * dy - PERSONNE_RAYON ** 2
            discriminant = b * b - c
            t = -b - np.sqrt(np.maximum(discriminant, 0.0))
            distances = np.where((discriminant >= 0) & (t > 0), np.minimum(distances, t), distances)
        distances[distances > portee_max] = np.inf
        return distances

    # ---------------------------------------------------------- mouvement
    def recevoir_commande(self, message):
        self.cmd_v, self.cmd_w = message.linear.x, message.angular.z

    def points_empreinte(self, x, y, lacet):
        c, s = math.cos(lacet), math.sin(lacet)
        for fx in np.linspace(EMPREINTE[0][0], EMPREINTE[0][1], 8):
            for fy in np.linspace(EMPREINTE[1][0], EMPREINTE[1][1], 6):
                yield x + c * fx - s * fy, y + s * fx + c * fy

    def bloque(self, x, y, lacet):
        """Nom de ce que l'empreinte toucherait en (x, y, lacet), sinon None."""
        for px, py in self.points_empreinte(x, y, lacet):
            ligne, colonne = int((py - OY) / self.res), int((px - OX) / self.res)
            if 0 <= ligne < self.h and 0 <= colonne < self.l and self.grille[ligne, colonne]:
                return 'mur ou panneau'
            if self.personne and math.hypot(px - self.personne['x'],
                                            py - self.personne['y']) < PERSONNE_RAYON:
                return 'PERSONNE'
        return None

    def deplacer_personne(self, dt):
        p = self.personne
        if p['pause'] > 0.0:
            p['pause'] -= dt
            return
        ax, ay = PERSONNE_A if p['sens'] > 0 else PERSONNE_DE
        dx, dy = ax - p['x'], ay - p['y']
        distance = math.hypot(dx, dy)
        pas = p['vitesse'] * dt
        if distance <= pas:
            p['x'], p['y'], p['sens'], p['pause'] = ax, ay, -p['sens'], 1.0
            return
        nx, ny = p['x'] + dx / distance * pas, p['y'] + dy / distance * pas
        # Le robot est sur son chemin : elle attend (elle ne le bouscule jamais),
        # puis fait demi-tour, comme un piéton qui cède le passage.
        if any(math.hypot(px - nx, py - ny) < PERSONNE_RAYON + 0.05
               for px, py in self.points_empreinte(self.x, self.y, self.lacet)):
            p['bloquee'] += dt
            if p['bloquee'] > PERSONNE_PATIENCE:
                p['sens'], p['bloquee'] = -p['sens'], 0.0
            return
        p['bloquee'] = 0.0
        p['x'], p['y'] = nx, ny

    def avancer(self):
        instant = time.monotonic()
        dt = min(0.1, instant - self.dernier)
        self.dernier = instant
        if self.personne:
            self.deplacer_personne(dt)
        self.v += max(-1.0 * dt, min(1.0 * dt, self.cmd_v - self.v))
        self.w += max(-2.5 * dt, min(2.5 * dt, self.cmd_w - self.w))
        self.w_reel = self.w
        nouveau_lacet = self.lacet + self.w_reel * dt
        nx = self.x + self.v * dt * math.cos(self.lacet + 0.5 * self.w_reel * dt)
        ny = self.y + self.v * dt * math.sin(self.lacet + 0.5 * self.w_reel * dt)
        obstacle = self.bloque(nx, ny, nouveau_lacet)
        if obstacle:
            if not self.en_contact:
                self.contacts += 1
                self.get_logger().warn(
                    f'CONTACT n°{self.contacts} ({obstacle}) en ({self.x:.2f}, {self.y:.2f})')
            self.en_contact = True
            self.v = 0.0
        else:
            self.en_contact = False
            self.x, self.y, self.lacet = nx, ny, nouveau_lacet
        # Odométrie : distance juste (+ bruit), rotation exagérée de 15 %.
        ds = self.v * dt * (1.0 + self.rng.normal(0, 0.01))
        dlacet = self.w * dt * ECHELLE_CAP_ODOM
        self.ox += ds * math.cos(self.olacet + 0.5 * dlacet)
        self.oy += ds * math.sin(self.olacet + 0.5 * dlacet)
        self.olacet += dlacet
        t = self.maintenant()
        self.publier_tf_odom(t)
        odometrie = Odometry()
        odometrie.header.stamp = stamp(t)
        odometrie.header.frame_id = 'odom'
        odometrie.child_frame_id = self.base
        odometrie.pose.pose.position.x, odometrie.pose.pose.position.y = self.ox, self.oy
        q = quat(self.olacet)
        odometrie.pose.pose.orientation.z, odometrie.pose.pose.orientation.w = q[2], q[3]
        odometrie.twist.twist.linear.x = self.v
        odometrie.twist.twist.angular.z = self.w * ECHELLE_CAP_ODOM
        self.pub_odom.publish(odometrie)
        vraie = PoseStamped()
        vraie.header.stamp = stamp(t)
        vraie.header.frame_id = 'world'
        vraie.pose.position.x, vraie.pose.position.y = self.x, self.y
        q = quat(self.lacet)
        vraie.pose.orientation.z, vraie.pose.orientation.w = q[2], q[3]
        self.pub_vraie.publish(vraie)
        self.pub_gz.publish(vraie.pose)

    def publier_tf_odom(self, t):
        tf = TransformStamped()
        tf.header.stamp = stamp(t)
        tf.header.frame_id, tf.child_frame_id = 'odom', self.base
        tf.transform.translation.x, tf.transform.translation.y = self.ox, self.oy
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quat(self.olacet)
        self.tf.sendTransform(tf)

    # ------------------------------------------------------------ capteurs
    def publier_statiques(self):
        """Tous les repères statiques en un seul message."""
        optique = (-0.5, 0.5, -0.5, 0.5)      # z devant, x à droite, y vers le bas
        if self.reel:
            reperes = (('base_link', 'laser', LIDAR_X, LIDAR_Z, (0, 0, 0, 1)),
                       ('base_link', 'camera1_link', CAM_X, CAM_Z, (0, 0, 0, 1)),
                       ('camera1_link', 'camera1_color_optical_frame', 0.0, 0.0, optique),
                       ('camera1_link', 'camera1_depth_optical_frame', 0.0, 0.0, optique),
                       ('base_link', 'camera2_link', CAM_X, 0.40, (0, 0, 0, 1)),
                       ('camera2_link', 'camera2_depth_optical_frame', 0.0, 0.0, optique))
        else:
            reperes = (('base_footprint', 'base_link', 0.0, 0.0, (0, 0, 0, 1)),
                       ('base_link', 'lidar_link', LIDAR_X, LIDAR_Z, (0, 0, 0, 1)),
                       ('base_link', 'imu_link', 0.0, 0.1, (0, 0, 0, 1)),
                       ('base_link', 'top_camera_link', CAM_X, CAM_Z, (0, 0, 0, 1)),
                       ('top_camera_link', self.optique, 0.0, 0.0, optique))
        messages = []
        for parent, enfant, x, z, q in reperes:
            tf = TransformStamped()
            tf.header.stamp = stamp(self.maintenant())
            tf.header.frame_id, tf.child_frame_id = parent, enfant
            tf.transform.translation.x, tf.transform.translation.z = x, z
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = (float(v) for v in q)
            messages.append(tf)
        self.tf_statique.sendTransform(messages)

    def publier_horloge(self):
        horloge = Clock()
        horloge.clock = stamp(self.maintenant())
        self.pub_horloge.publish(horloge)

    def publier_imu(self):
        imu = Imu()
        imu.header.stamp = stamp(self.maintenant())
        imu.header.frame_id = 'imu_link'
        q = quat(self.lacet + 0.4)
        imu.orientation.z, imu.orientation.w = q[2], q[3]
        imu.angular_velocity.z = self.w_reel + self.rng.normal(0, 0.002)
        # Robot penché de 2,46° vers l'avant, comme dans Gazebo.
        imu.linear_acceleration.x = -9.80665 * math.sin(0.0429) + self.rng.normal(0, 0.01)
        imu.linear_acceleration.z = 9.80665 * math.cos(0.0429) + self.rng.normal(0, 0.01)
        self.pub_imu.publish(imu)

    def publier_scan(self):
        lx = self.x + LIDAR_X * math.cos(self.lacet)
        ly = self.y + LIDAR_X * math.sin(self.lacet)
        distances = self.lancer_rayons(lx, ly, self.lacet + self.angles_lidar, 12.0)
        distances = distances + self.rng.normal(0, 0.01, distances.shape)
        distances[distances < 0.05] = np.inf
        scan = LaserScan()
        scan.header.stamp = stamp(self.maintenant())
        scan.header.frame_id = self.laser
        scan.angle_min = float(self.angles_lidar[0])
        scan.angle_increment = float(2 * math.pi / len(self.angles_lidar))
        scan.angle_max = float(self.angles_lidar[-1])
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = 0.05, 12.0
        scan.ranges = [float(r) for r in distances]
        self.pub_scan.publish(scan)

    def camera(self):
        return (self.x + CAM_X * math.cos(self.lacet), self.y + CAM_X * math.sin(self.lacet))

    def visible(self, cx, cy, x, y):
        distance = math.hypot(x - cx, y - cy)
        angle = math.atan2(y - cy, x - cx)
        return self.lancer_rayons(cx, cy, np.array([angle]), 12.0)[0] >= distance - 0.08

    def publier_image(self):
        # Odométrie republiée à l'heure exacte de l'image, comme un vrai robot (TF à 50-100 Hz).
        t_image = self.maintenant()
        self.publier_tf_odom(t_image)
        image = np.full((H, W), 140, np.uint8)
        image[:H // 2] = 170
        cx, cy = self.camera()
        avant = np.array([math.cos(self.lacet), math.sin(self.lacet), 0.0])
        droite = np.array([math.sin(self.lacet), -math.cos(self.lacet), 0.0])
        bas = np.array([0.0, 0.0, -1.0])
        for identifiant, (x, y, cap) in self.tags.items():
            n = np.array([math.cos(cap), math.sin(cap), 0.0])
            if np.dot(n, np.array([cx - x, cy - y, 0.0])) <= 0.05 or not self.visible(cx, cy, x, y):
                continue
            r = np.array([-n[1], n[0], 0.0])            # droite vue de face
            haut = np.array([0.0, 0.0, 1.0])
            centre = np.array([x, y, HAUTEUR_TAG])
            m = self.taille_tag * 1.5 / 2.0              # papier blanc autour
            coins = [centre - r * m + haut * m, centre + r * m + haut * m,
                     centre + r * m - haut * m, centre - r * m - haut * m]
            projetes = []
            for coin in coins:
                relatif = coin - np.array([cx, cy, CAM_Z])
                profondeur = float(np.dot(relatif, avant))
                if profondeur < 0.15:
                    break
                projetes.append((W / 2.0 + F * float(np.dot(relatif, droite)) / profondeur,
                                 H / 2.0 + F * float(np.dot(relatif, bas)) / profondeur))
            if len(projetes) < 4:
                continue
            source = np.float32([[0, 0], [300, 0], [300, 300], [0, 300]])
            homographie = cv2.getPerspectiveTransform(source, np.float32(projetes))
            motif = cv2.warpPerspective(self.motifs[identifiant], homographie, (W, H),
                                        flags=cv2.INTER_LINEAR, borderValue=0)
            masque = cv2.warpPerspective(np.full((300, 300), 255, np.uint8), homographie,
                                         (W, H), flags=cv2.INTER_NEAREST, borderValue=0)
            image[masque > 0] = motif[masque > 0]
        image = np.clip(image.astype(np.int16)
                        + self.rng.integers(-4, 5, image.shape, dtype=np.int16), 0, 255).astype(np.uint8)
        message = Image()
        message.header.stamp = stamp(t_image)
        message.header.frame_id = self.optique
        message.height, message.width, message.encoding = H, W, 'rgb8'
        message.step = W * 3
        message.data = np.repeat(image[:, :, None], 3, axis=2).tobytes()
        info = CameraInfo()
        info.header = message.header
        info.width, info.height = W, H
        info.k = [F, 0.0, W / 2.0, 0.0, F, H / 2.0, 0.0, 0.0, 1.0]
        info.p = [F, 0.0, W / 2.0, 0.0, 0.0, F, H / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        self.pub_image.publish(message)
        self.pub_info.publish(info)

    def publier_nuages(self):
        """Points des obstacles devant le robot, dans base_link (hauteurs fixes)."""
        cx, cy = self.camera()
        angles = self.lacet + np.linspace(-0.7, 0.7, 60)
        portees = self.lancer_rayons(cx, cy, angles, 3.0)
        for publicateur, hauteur, portee_max in ((self.pub_nuage_haut, 0.9, 3.0),
                                                 (self.pub_nuage_bas, 0.4, 1.2)):
            points = []
            for angle, r in zip(angles, portees):
                if np.isfinite(r) and r <= portee_max:
                    dx = cx + r * math.cos(angle) - self.x
                    dy = cy + r * math.sin(angle) - self.y
                    points.append((dx * math.cos(self.lacet) + dy * math.sin(self.lacet),
                                   -dx * math.sin(self.lacet) + dy * math.cos(self.lacet), hauteur))
            entete = Image().header
            entete.stamp = stamp(self.maintenant())
            entete.frame_id = 'base_link'
            publicateur.publish(point_cloud2.create_cloud_xyz32(entete, points))

    def publier_profondeurs(self):
        """Robot réel : images de profondeur 16 bits des deux caméras (murs infiniment hauts)."""
        largeur, hauteur, focale = 160, 120, 80.0
        cx, cy = self.camera()
        u = np.arange(largeur)
        angles = -np.arctan((u - (largeur - 1) / 2.0) / focale)       # x optique à droite
        z = self.lancer_rayons(cx, cy, self.lacet + angles, 3.0) * np.cos(angles)
        millimetres = np.where(np.isfinite(z), z * 1000.0, 0).astype(np.uint16)
        donnees = np.tile(millimetres, (hauteur, 1)).tobytes()
        t = stamp(self.maintenant())
        for pub_image, pub_info, repere in self.profondeurs:
            image = Image()
            image.header.stamp, image.header.frame_id = t, repere
            image.height, image.width, image.encoding = hauteur, largeur, '16UC1'
            image.step = largeur * 2
            image.data = donnees
            info = CameraInfo()
            info.header = image.header
            info.width, info.height = largeur, hauteur
            info.k = [focale, 0.0, (largeur - 1) / 2.0, 0.0, focale, (hauteur - 1) / 2.0, 0.0, 0.0, 1.0]
            info.p = [focale, 0.0, (largeur - 1) / 2.0, 0.0, 0.0, focale, (hauteur - 1) / 2.0,
                      0.0, 0.0, 0.0, 1.0, 0.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            pub_image.publish(image)
            pub_info.publish(info)

    def publier_obstacles(self):
        obstacles = {}
        if self.personne:
            obstacles['personne'] = [self.personne['x'], self.personne['y'], PERSONNE_RAYON]
        self.pub_obstacles.publish(String(data=json.dumps(obstacles)))


def main():
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('--reel', action='store_true', help='interfaces du vrai robot (robot:=real)')
    lecteur.add_argument('--personne', action='store_true', help='une personne traverse devant le départ')
    lecteur.add_argument('--vitesse-personne', type=float, default=0.4, help='m/s')
    lecteur.add_argument('--sans-tag', type=int, action='append', default=[],
                         help='identifiant absent de l\'arène (répétable)')
    args, reste = lecteur.parse_known_args()
    rclpy.init(args=reste)
    noeud = Simulateur(args)
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():            # erreur réelle ; sinon : arrêt pendant une publication
            raise
    finally:
        print(f'Contacts : {noeud.contacts}', flush=True)
        noeud.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
