"""Surveillance technique de la navigation, sans ROS.

Complète le récit de la course par des événements bas niveau, dans l'esprit
du moniteur de la branche benchmark : trajets Nav2, chemins, rotations,
arrêts prolongés, et surtout le robot BLOQUÉ (commandes envoyées mais aucun
mouvement mesuré par l'odométrie) ou qui OSCILLE (tourne à gauche puis à
droite sans progresser). Chaque méthode reçoit le temps `t`
(horloge ROS) et renvoie une liste de lignes `[ÉTIQUETTE] texte`.
"""

import math

STATUTS_NAV2 = {
    1: '[NAV2] trajet en cours',
    4: '[NAV2] arrivé au but',
    5: '[NAV2] trajet annulé',
    6: "[NAV2] ÉCHEC : Nav2 a abandonné le trajet",
}
COMMANDE_LINEAIRE_MIN = 0.02     # m/s : en dessous, pas d'avance demandée
COMMANDE_ANGULAIRE_MIN = 0.10    # rad/s
ROTATION_MIN_SECONDES = 1.0      # rotations plus courtes non signalées
ARRET_SIGNALE_APRES = 6.0        # s à l'arrêt pendant un trajet Nav2
ARRET_RAPPEL = 6.0
BLOCAGE_FENETRE = 5.0            # s de commandes sans mouvement
BLOCAGE_DEPLACEMENT_MIN = 0.03   # m
BLOCAGE_ROTATION_MIN = math.radians(4.0)
BLOCAGE_RAPPEL = 5.0
DETOUR_MIN = 0.5                 # m : un chemin qui s'allonge d'autant = détour


def _m(valeur):
    return f'{valeur:.2f}'.replace('.', ',')


class Surveillance:

    def __init__(self):
        self.nav2_actif = False
        self.replanifications = 0
        self.contacts = 0
        self._statut = None
        self._longueur_chemin = None    # longueur du dernier chemin reçu
        self._chemin_annonce = False    # premier chemin du trajet déjà affiché
        self._rotation_debut = None
        self._arret_debut = None
        self._arret_rappel = -math.inf
        self._commande = (0.0, 0.0)
        self._commande_depuis = None    # instant où une commande non nulle a commencé
        self._reference = None          # (t, x, y, cap) au début de la fenêtre
        self._blocage_rappel = -math.inf
        self._sens_rotation = 0         # signe de la dernière rotation commandée
        self._inversions = 0            # changements de sens dans la fenêtre

    # ---------------------------------------------------------------- Nav2
    def statut_nav2(self, code, t):
        """Statut action_msgs : 1 accepté et 2 en exécution = trajet en cours."""
        if code == 2:
            code = 1
        if code == self._statut:
            return []
        self._statut = code
        self.nav2_actif = code == 1
        if self.nav2_actif:
            self._chemin_annonce = False
        texte = STATUTS_NAV2.get(code)
        return [texte] if texte else []

    def chemin(self, points, longueur, t):
        """Nav2 replanifie deux fois par seconde : on n'affiche que le premier
        chemin d'un trajet et les vrais détours (chemin nettement rallongé)."""
        if points == 0:
            return ['[CHEMIN] aucun chemin trouvé vers le but']
        self.replanifications += 1
        precedente, self._longueur_chemin = self._longueur_chemin, longueur
        if not self._chemin_annonce:
            self._chemin_annonce = True
            return [f'[CHEMIN] trajet planifié : {_m(longueur)} m']
        if precedente is not None and longueur - precedente >= DETOUR_MIN:
            return [f'[CHEMIN] détour : chemin rallongé de {_m(longueur - precedente)} m '
                    f'({_m(longueur)} m), obstacle sur l\'ancien chemin ?']
        return []

    # ----------------------------------------------------------- commandes
    def commande(self, lineaire, angulaire, t):
        lignes = []
        avance = abs(lineaire) >= COMMANDE_LINEAIRE_MIN
        tourne = abs(angulaire) >= COMMANDE_ANGULAIRE_MIN
        if tourne and not avance:
            if self._rotation_debut is None:
                self._rotation_debut = t
        elif self._rotation_debut is not None:
            duree = t - self._rotation_debut
            self._rotation_debut = None
            if duree >= ROTATION_MIN_SECONDES:
                lignes.append(f'[ROTATION] rotation sur place de {duree:.0f} s')
        arret = not avance and not tourne
        if arret and self.nav2_actif:
            if self._arret_debut is None:
                self._arret_debut = t
            duree = t - self._arret_debut
            if duree >= ARRET_SIGNALE_APRES and t - self._arret_rappel >= ARRET_RAPPEL:
                self._arret_rappel = t
                lignes.append(f"[ARRÊT] immobile depuis {duree:.0f} s pendant un trajet Nav2")
        else:
            self._arret_debut = None
        if tourne:
            sens = 1 if angulaire > 0.0 else -1
            if self._sens_rotation and sens != self._sens_rotation:
                self._inversions += 1
            self._sens_rotation = sens
        if arret:
            self._commande_depuis = None
        elif self._commande_depuis is None:
            self._commande_depuis = t
            self._reference = None
        self._commande = (lineaire, angulaire)
        return lignes

    # ---------------------------------------------------------- mouvement
    def mouvement(self, x, y, cap, t):
        """Position odométrique : détecte des commandes sans effet (blocage)."""
        if self._commande_depuis is None:
            self._reference = None
            return []
        if self._reference is None:
            self._reference = (t, x, y, cap)
            self._inversions = 0
            return []
        debut, x0, y0, cap0 = self._reference
        if t - debut < BLOCAGE_FENETRE:
            return []
        deplacement = math.hypot(x - x0, y - y0)
        rotation = abs(math.atan2(math.sin(cap - cap0), math.cos(cap - cap0)))
        inversions, self._inversions = self._inversions, 0
        self._reference = (t, x, y, cap)
        if deplacement >= BLOCAGE_DEPLACEMENT_MIN or rotation >= BLOCAGE_ROTATION_MIN:
            return []
        if t - self._blocage_rappel < BLOCAGE_RAPPEL:
            return []
        self._blocage_rappel = t
        if inversions >= 2:
            return [f'[OSCILLATION] le robot tourne à gauche puis à droite sans progresser '
                    f'({inversions} changements de sens en {BLOCAGE_FENETRE:.0f} s)']
        lineaire, angulaire = self._commande
        return [f'[BLOQUÉ] commandes envoyées (v={lineaire:+.2f}, w={angulaire:+.2f}) '
                f'mais le robot ne bouge pas depuis {BLOCAGE_FENETRE:.0f} s']

    # ------------------------------------------------------------ contacts
    def contact(self, objets):
        self.contacts += 1
        return [f'[CONTACT] le robot touche : {", ".join(sorted(objets))}']

    # ---------------------------------------------------------------- état
    def ligne_etat(self, pose):
        position = ('position inconnue' if pose is None else
                    f'position ({_m(pose[0])} ; {_m(pose[1])}), cap {math.degrees(pose[2]):+.0f}°')
        lineaire, angulaire = self._commande
        return (f'[ÉTAT] {position} | v={lineaire:+.2f} w={angulaire:+.2f} | '
                f'Nav2 {"actif" if self.nav2_actif else "inactif"} | '
                f'chemins {self.replanifications} | contacts {self.contacts}')
