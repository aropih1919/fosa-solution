"""Classification géométrique de nuages de profondeur, complétée par YOLO."""

import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from caytu_config import load_config
from caytu_config.loader import config_directory
from caytu_interfaces.msg import DetectedEntity, DetectedEntityArray


def _tourner_quaternion(quaternion, vecteur):
    x, y, z, w = quaternion
    vx, vy, vz = vecteur
    uv = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2],
           x * uv[1] - y * uv[0])
    return tuple(vecteur[i] + 2.0 * (w * uv[i] + uuv[i]) for i in range(3))


def _matrice_quaternion(quaternion):
    """Construit la matrice de rotation d'un quaternion ROS (x, y, z, w)."""
    x, y, z, w = quaternion
    return np.asarray((
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)),
    ), dtype=np.float64)


def _timestamp_ns(en_tete):
    return en_tete.stamp.sec * 1_000_000_000 + en_tete.stamp.nanosec


def _parametres_letterbox(largeur_image, hauteur_image, taille):
    facteur = min(taille / largeur_image, taille / hauteur_image)
    largeur_redimensionnee = round(largeur_image * facteur)
    hauteur_redimensionnee = round(hauteur_image * facteur)
    gauche = int(round((taille - largeur_redimensionnee) / 2.0 - 0.1))
    haut = int(round((taille - hauteur_redimensionnee) / 2.0 - 0.1))
    return facteur, gauche, haut


def _preparer_image_yolo(image, taille):
    """Préserve le ratio de l'image avec le padding attendu par YOLO."""
    hauteur, largeur = image.shape[:2]
    facteur, gauche, haut = _parametres_letterbox(largeur, hauteur, taille)
    largeur_redimensionnee = round(largeur * facteur)
    hauteur_redimensionnee = round(hauteur * facteur)
    droite = taille - largeur_redimensionnee - gauche
    bas = taille - hauteur_redimensionnee - haut
    image_redimensionnee = cv2.resize(
        image, (largeur_redimensionnee, hauteur_redimensionnee),
        interpolation=cv2.INTER_LINEAR)
    return cv2.copyMakeBorder(
        image_redimensionnee, haut, bas, gauche, droite,
        cv2.BORDER_CONSTANT, value=(114, 114, 114)), facteur, gauche, haut


def _supprimer_boites_redondantes(boites, seuil_iou, maximum):
    """Applique une NMS indépendante par catégorie et limite les résultats."""
    retenues = []
    categories = sorted({boite[0] for boite in boites})
    for categorie in categories:
        detections = sorted(
            (boite for boite in boites if boite[0] == categorie),
            key=lambda boite: boite[1], reverse=True)
        while detections:
            meilleure = detections.pop(0)
            retenues.append(meilleure)
            x1, y1, x2, y2 = meilleure[2:]
            aire_meilleure = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            conserves = []
            for detection in detections:
                ax1, ay1, ax2, ay2 = detection[2:]
                intersection = max(0.0, min(x2, ax2) - max(x1, ax1)) * max(
                    0.0, min(y2, ay2) - max(y1, ay1))
                aire_candidate = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
                union = aire_meilleure + aire_candidate - intersection
                iou = intersection / union if union > 0.0 else 0.0
                if iou <= seuil_iou:
                    conserves.append(detection)
            detections = conserves
    return sorted(retenues, key=lambda boite: boite[1], reverse=True)[:maximum]


def _extraire_boites_yolo(sortie, largeur_image, hauteur_image, configuration):
    """Interprète les sorties YOLOv8/11 brutes et les sorties NMS à six colonnes."""
    sortie = np.squeeze(np.asarray(sortie))
    if sortie.ndim == 1 and sortie.size in (6, 84, 85):
        sortie = sortie.reshape((1, -1))
    if sortie.ndim != 2:
        return []
    axes_caracteristiques = (6, 84, 85)
    if (sortie.shape[0] in axes_caracteristiques
            and sortie.shape[1] > sortie.shape[0]):
        sortie = sortie.T
    elif sortie.shape[1] not in axes_caracteristiques and sortie.shape[0] < sortie.shape[1]:
        sortie = sortie.T

    boites = []
    taille = float(configuration.input_size)
    facteur, pad_x, pad_y = _parametres_letterbox(
        largeur_image, hauteur_image, taille)
    for ligne in sortie:
        if len(ligne) == 6:
            x1, y1, x2, y2, confiance, classe = map(float, ligne)
            indice_classe = int(classe)
        else:
            # YOLOv5 ajoute objectness ; YOLOv8/11 ne l'inclut pas.
            decalage = 5 if len(ligne) == 85 else 4
            if len(ligne) <= decalage:
                continue
            scores = ligne[decalage:]
            indice_classe = int(np.argmax(scores))
            confiance = float(scores[indice_classe])
            if decalage == 5:
                confiance *= float(ligne[4])
            centre_x, centre_y, largeur, hauteur = map(float, ligne[:4])
            x1, y1 = centre_x - largeur / 2.0, centre_y - hauteur / 2.0
            x2, y2 = centre_x + largeur / 2.0, centre_y + hauteur / 2.0

        if (not all(math.isfinite(valeur)
                    for valeur in (x1, y1, x2, y2, confiance))
                or confiance < configuration.confidence):
            continue
        x1 = min(max((x1 - pad_x) / facteur, 0.0), float(largeur_image - 1))
        x2 = min(max((x2 - pad_x) / facteur, 0.0), float(largeur_image - 1))
        y1 = min(max((y1 - pad_y) / facteur, 0.0), float(hauteur_image - 1))
        y2 = min(max((y2 - pad_y) / facteur, 0.0), float(hauteur_image - 1))
        if x2 <= x1 or y2 <= y1:
            continue
        boites.append((
            'personne' if indice_classe == configuration.person_class_index
            else 'obstacle', confiance, x1, y1, x2, y2))
    return _supprimer_boites_redondantes(
        boites, configuration.nms_iou_threshold,
        configuration.max_detections)


def _associer_suivis_odom(candidats, suivis_precedents, identifiant_suivant,
                          horodatage_ns, distance_max, age_max, vitesse_min,
                          quaternion_base_depuis_odom):
    """Associe les entités dans odom et calcule une vitesse indépendante du robot."""
    disponibles = set(range(len(suivis_precedents)))
    suivis_suivants = []
    entites = []
    for entite, odom_x, odom_y in candidats:
        meilleur = None
        meilleure_distance = distance_max
        temps_ecoule = None
        for index in disponibles:
            precedent = suivis_precedents[index]
            if precedent[1] != entite.class_name:
                continue
            delta_t = (horodatage_ns - precedent[4]) * 1e-9
            if not 0.0 < delta_t <= age_max:
                continue
            distance = math.hypot(odom_x - precedent[2], odom_y - precedent[3])
            if distance < meilleure_distance:
                meilleur, meilleure_distance, temps_ecoule = index, distance, delta_t

        if meilleur is None:
            identifiant = identifiant_suivant
            identifiant_suivant += 1
            vitesse = (0.0, 0.0)
            norme_vitesse = 0.0
        else:
            precedent = suivis_precedents[meilleur]
            disponibles.remove(meilleur)
            identifiant = precedent[0]
            vitesse = (
                (odom_x - precedent[2]) / temps_ecoule,
                (odom_y - precedent[3]) / temps_ecoule)
            norme_vitesse = meilleure_distance / temps_ecoule

        vitesse_base = _tourner_quaternion(
            quaternion_base_depuis_odom,
            (vitesse[0], vitesse[1], 0.0))
        entite.track_id = identifiant
        entite.velocity.x = vitesse_base[0]
        entite.velocity.y = vitesse_base[1]
        entite.is_dynamic = norme_vitesse >= vitesse_min
        suivis_suivants.append((
            identifiant, entite.class_name, odom_x, odom_y, horodatage_ns))
        entites.append(entite)

    for index in disponibles:
        precedent = suivis_precedents[index]
        age = (horodatage_ns - precedent[4]) * 1e-9
        if 0.0 <= age <= age_max:
            suivis_suivants.append(precedent)
    return entites, suivis_suivants, identifiant_suivant


def _associer_boite_yolo(cluster, boites, nombre_points_min):
    """Associe une boîte à un cluster uniquement par recouvrement de pixels 3D."""
    meilleure = None
    meilleur_score = 0
    for boite in boites:
        _, _, x1, y1, x2, y2 = boite
        points_dans_boite = sum(
            x1 <= point[5] <= x2 and y1 <= point[6] <= y2
            for point in cluster)
        if (points_dans_boite >= nombre_points_min
                and points_dans_boite > meilleur_score):
            meilleure = boite
            meilleur_score = points_dans_boite
    return meilleure


def _composantes(points, tolerance):
    """Regroupe les points sur une grille XY sans dépendance à scikit-learn."""
    cellules = {}
    for point in points:
        key = (math.floor(point[0] / tolerance),
               math.floor(point[1] / tolerance))
        cellules.setdefault(key, []).append(point)
    restantes = set(cellules)
    while restantes:
        en_attente = [restantes.pop()]
        composante = []
        while en_attente:
            cellule = en_attente.pop()
            composante.extend(cellules[cellule])
            cx, cy = cellule
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    voisine = (cx + dx, cy + dy)
                    if voisine in restantes:
                        restantes.remove(voisine)
                        en_attente.append(voisine)
        yield composante


class ClassificateurObstacles(Node):
    def __init__(self, configuration=None):
        super().__init__('obstacle_classifier')
        self._configuration = configuration or load_config()
        self._bridge = CvBridge()
        self._camera_info = None
        self._yolo = None
        self._dernier_traitement_yolo = 0.0
        self._suivis_precedents = []
        self._prochain_identifiant_suivi = 1
        self._images = deque(maxlen=10)
        self._nuages = deque(maxlen=10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        perception = self._configuration.perception
        self.declare_parameter('cloud_topic', perception.top_depth_topic)
        self.declare_parameter('image_topic', perception.top_color_topic)
        self.declare_parameter('camera_info_topic', perception.top_info_topic)
        self.declare_parameter('output_topic', perception.entities_topic)
        self._publisher = self.create_publisher(
            DetectedEntityArray, self.get_parameter('output_topic').value, 10)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._recevoir_infos_camera, 10)
        self._cloud_sub = self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self._recevoir_nuage, 5)
        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._recevoir_image, 5)
        self._charger_yolo()

    def _charger_yolo(self):
        yolo = self._configuration.perception.yolo
        if not yolo.enabled:
            return
        path = os.path.expanduser(yolo.model_path)
        if not os.path.isabs(path):
            path = str(config_directory() / path)
        try:
            self._yolo = cv2.dnn.readNetFromONNX(path)
            if yolo.device.lower() == 'cuda':
                self._yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self._yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            self.get_logger().info(f'Modèle YOLO chargé : {path}')
        except (cv2.error, OSError) as error:
            self.get_logger().warning(
                f'YOLO indisponible ({error}); la géométrie reste active.')

    def _recevoir_infos_camera(self, message):
        self._camera_info = message

    def _recevoir_image(self, message):
        self._images.append((message, time.monotonic()))
        self._traiter_paires_image_nuage()

    def _recevoir_nuage(self, message):
        self._nuages.append((message, time.monotonic()))
        self._traiter_paires_image_nuage()

    def _traiter_paires_image_nuage(self):
        delai_max = self._configuration.perception.tracking.image_cloud_sync_slop_sec
        while self._images and self._nuages:
            image_candidate = self._images[0][0]
            nuage_candidat = self._nuages[0][0]
            stamp_image = _timestamp_ns(image_candidate.header)
            stamp_nuage = _timestamp_ns(nuage_candidat.header)
            if abs(stamp_image - stamp_nuage) > delai_max * 1e9:
                # Les flux sont ordonnés par stamp : le message le plus ancien
                # ne pourra plus trouver un partenaire dans la fenêtre.
                if stamp_image < stamp_nuage:
                    self._images.popleft()
                else:
                    self._nuages.popleft()
                continue
            image, reception_image = self._images.popleft()
            nuage, reception_nuage = self._nuages.popleft()
            age_max = self._configuration.perception.tracking.max_observation_age_sec
            if max(time.monotonic() - reception_image,
                   time.monotonic() - reception_nuage) > age_max:
                self.get_logger().warning(
                    'Paire image/profondeur trop ancienne, observation ignorée.')
                continue
            maintenant_ros = self.get_clock().now().nanoseconds
            if maintenant_ros > 0:
                age_image = (maintenant_ros - _timestamp_ns(image.header)) * 1e-9
                age_nuage = (maintenant_ros - _timestamp_ns(nuage.header)) * 1e-9
                if (min(age_image, age_nuage) < -delai_max
                        or max(age_image, age_nuage) > age_max):
                    self.get_logger().warning(
                        'Paire image/profondeur périmée selon l’horloge ROS, '
                        'observation ignorée.')
                    continue
            self._traiter_observation(image, nuage)

    def _boites_yolo(self, image_message):
        if self._yolo is None:
            return []
        info = self._camera_info
        if info is None:
            return []
        image_frame = image_message.header.frame_id or info.header.frame_id
        if (info.width != image_message.width or info.height != image_message.height
                or (info.header.frame_id and image_frame
                    and info.header.frame_id != image_frame)
                or info.k[0] <= 0.0 or info.k[4] <= 0.0
                or info.distortion_model not in ('plumb_bob', 'rational_polynomial')):
            self.get_logger().warning(
                'CameraInfo incompatible avec l’image (calibration, résolution '
                'ou frame) : '
                'YOLO désactivé pour cette observation.')
            return []
        config = self._configuration.perception.yolo
        maintenant = time.monotonic()
        if maintenant - self._dernier_traitement_yolo < 1.0 / config.max_fps:
            return []
        self._dernier_traitement_yolo = maintenant
        image = self._bridge.imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')
        image_yolo, _, _, _ = _preparer_image_yolo(image, config.input_size)
        blob = cv2.dnn.blobFromImage(
            image_yolo, scalefactor=1.0 / 255.0, swapRB=True)
        self._yolo.setInput(blob)
        return _extraire_boites_yolo(
            self._yolo.forward(), image.shape[1], image.shape[0], config)

    def _traiter_observation(self, image_message, nuage_message):
        horodatage_ns = _timestamp_ns(nuage_message.header)
        if horodatage_ns <= 0:
            self.get_logger().warning(
                'Nuage sans horodatage valide : observation ignorée.')
            return
        try:
            stamp = Time.from_msg(nuage_message.header.stamp)
            base_depuis_nuage = self._tf_buffer.lookup_transform(
                self._configuration.system.base_frame,
                nuage_message.header.frame_id, stamp)
            odom_depuis_nuage = self._tf_buffer.lookup_transform(
                self._configuration.system.odom_frame,
                nuage_message.header.frame_id, stamp)
            base_depuis_odom = self._tf_buffer.lookup_transform(
                self._configuration.system.base_frame,
                self._configuration.system.odom_frame, stamp)
        except TransformException:
            self.get_logger().warning(
                'TF image/profondeur vers base/odom indisponible à '
                'l’horodatage : observation ignorée.')
            return

        image_frame = image_message.header.frame_id
        if not image_frame and self._camera_info is not None:
            image_frame = self._camera_info.header.frame_id
        nuage_vers_camera = None
        if self._yolo is not None and self._camera_info is not None and image_frame:
            try:
                nuage_vers_camera = self._tf_buffer.lookup_transform(
                    image_frame, nuage_message.header.frame_id, stamp)
            except TransformException:
                self.get_logger().warning(
                    'TF profondeur vers caméra couleur absent : YOLO ignoré '
                    'pour cette paire.')
        try:
            points_lus = point_cloud2.read_points(
                nuage_message, field_names=('x', 'y', 'z'), skip_nans=True)
            if isinstance(points_lus, np.ndarray) and points_lus.dtype.names:
                points_bruts = np.column_stack(
                    tuple(points_lus[nom] for nom in ('x', 'y', 'z')))
            else:
                points_bruts = np.asarray(
                    [tuple(point)[:3] for point in points_lus], dtype=np.float64)
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'Nuage de points ignoré : {error}')
            return
        if points_bruts.size == 0:
            self._publier([], nuage_message.header)
            return
        points_bruts = points_bruts.reshape((-1, 3))
        points_bruts = points_bruts[np.all(np.isfinite(points_bruts), axis=1)]
        if points_bruts.size == 0:
            self._publier([], nuage_message.header)
            return
        points_base = self._transformer_points(points_bruts, base_depuis_nuage)
        points_odom = self._transformer_points(points_bruts, odom_depuis_nuage)
        pixels = np.full((len(points_bruts), 2), -1.0, dtype=np.float64)
        try:
            boites = self._boites_yolo(image_message)
        except (cv2.error, CvBridgeError, ValueError, TypeError) as error:
            boites = []
            self.get_logger().warning(f'Image YOLO ignorée : {error}')
        if nuage_vers_camera is not None and boites:
            try:
                camera_points = self._transformer_points(
                    points_bruts, nuage_vers_camera)
                camera_info = self._camera_info
                matrice = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
                distorsion = np.asarray(camera_info.d, dtype=np.float64)
                indices = np.flatnonzero(
                    (camera_points[:, 2] > 1e-5)
                    & np.all(np.isfinite(camera_points), axis=1))
                if indices.size:
                    projetees, _ = cv2.projectPoints(
                        camera_points[indices], np.zeros(3), np.zeros(3),
                        matrice, distorsion)
                    pixels[indices] = projetees.reshape((-1, 2))
            except (cv2.error, CvBridgeError, ValueError, TypeError) as error:
                self.get_logger().warning(
                    f'Projection YOLO/profondeur ignorée : {error}')

        points = [(
            float(points_base[index, 0]), float(points_base[index, 1]),
            float(points_base[index, 2]), float(points_odom[index, 0]),
            float(points_odom[index, 1]), float(pixels[index, 0]),
            float(pixels[index, 1]))
            for index in range(len(points_base))
            if np.all(np.isfinite(points_base[index]))
            and np.all(np.isfinite(points_odom[index]))]

        config = self._configuration.perception.person
        candidates = []
        for cluster in _composantes(points, config.cluster_tolerance_m):
            if len(cluster) < config.min_cluster_points:
                continue
            xs, ys, zs = zip(*((point[0], point[1], point[2])
                               for point in cluster))
            width = max(max(xs) - min(xs), max(ys) - min(ys))
            height = max(zs) - min(zs)
            x = sum(xs) / len(xs)
            y = sum(ys) / len(ys)
            odom_x = sum(point[3] for point in cluster) / len(cluster)
            odom_y = sum(point[4] for point in cluster) / len(cluster)
            distance = math.hypot(x, y)
            if distance == 0.0:
                continue
            detection = self._associer_detection(cluster, boites)
            is_person = (
                detection[0] == 'personne' if detection is not None
                else height >= config.min_height_m and width <= config.max_width_m)
            entity = DetectedEntity()
            entity.class_name = 'personne' if is_person else 'obstacle'
            entity.confidence = min(1.0, len(cluster) / config.min_cluster_points)
            if detection is not None:
                entity.confidence = detection[1]
            entity.position = Point(x=x, y=y, z=sum(zs) / len(zs))
            entity.distance_m = distance
            entity.bearing_rad = math.atan2(y, x)
            entity.is_person = is_person
            entity.source = 'geometry+yolo' if detection is not None else 'geometry'
            candidates.append((entity, odom_x, odom_y))

        entities = self._attribuer_suivis(
            candidates, horodatage_ns, base_depuis_odom)
        self._publier(entities, nuage_message.header)

    @staticmethod
    def _transformer_points(points, transformation):
        translation = transformation.transform.translation
        rotation = transformation.transform.rotation
        matrice = _matrice_quaternion(
            (rotation.x, rotation.y, rotation.z, rotation.w))
        return points @ matrice.T + np.asarray(
            (translation.x, translation.y, translation.z), dtype=np.float64)

    def _associer_detection(self, cluster, boites):
        minimum = self._configuration.perception.tracking.min_projected_points
        return _associer_boite_yolo(cluster, boites, minimum)

    def _attribuer_suivis(self, candidats, horodatage_ns, base_depuis_odom):
        config_poursuite = self._configuration.perception.person
        config_age = self._configuration.perception.tracking
        rotation = base_depuis_odom.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        entites, self._suivis_precedents, self._prochain_identifiant_suivi = (
            _associer_suivis_odom(
                candidats, self._suivis_precedents,
                self._prochain_identifiant_suivi,
                horodatage_ns, config_poursuite.tracker_match_m,
                config_age.track_max_gap_sec,
                config_poursuite.dynamic_speed_mps, quaternion))
        return entites

    def _publier(self, entities, header):
        message = DetectedEntityArray()
        message.header = header
        message.header.frame_id = self._configuration.system.base_frame
        message.entities = entities
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ClassificateurObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
