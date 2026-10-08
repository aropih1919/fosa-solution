"""Que faire quand Nav2 interrompt la navigation (aucune dépendance ROS).

Deux rôles :
  1. traduire le code d'erreur de Nav2 en une cause lisible, pour que le
     journal dise POURQUOI la navigation s'est arrêtée ;
  2. choisir la suite (quoi effacer, quand viser un point de repli) à partir
     de cette cause et du progrès réel du robot vers le but.

Tout est en O(1) par échec et se teste avec `pytest`.
"""

from typing import Dict, Iterable, Optional, Tuple

# Actions rendues par RetryPolicy.on_failure().
CLEAR_FAR = 'clear_far'      # costmap globale effacée loin du robot seulement
CLEAR_NEAR = 'clear_near'    # costmap globale effacée autour du robot
CLEAR_ALL = 'clear_all'      # dernier recours : tout effacer
RESEND = 'resend'            # rien à nettoyer : renvoyer le but après une courte attente
FALLBACK = 'fallback'        # viser le point libre le plus proche du but

# Causes internes à task_solution.py (elles ne viennent pas de Nav2).
CAUSE_NONE = 'NONE'
CAUSE_SILENT = 'NAV2_SILENT'
CAUSE_TOO_FAR = 'ARRIVAL_TOO_FAR'
CAUSE_CANCELED = 'CANCELED'
# Cause donnée par Nav2 (contrôleur ou planificateur) : position du robot indisponible.
CAUSE_TF = 'TF_ERROR'

# Constantes de nav2_msgs/action/*.action (Nav2 Jazzy), par famille. Les
# numéros sont relus dans le paquet installé au démarrage (resolve_error_names) ;
# ceux-ci ne servent que si cette lecture échoue.
ERROR_FAMILIES: Dict[str, Tuple[Tuple[str, int], ...]] = {
    'contrôleur': (
        ('UNKNOWN', 100), ('INVALID_CONTROLLER', 101), ('TF_ERROR', 102),
        ('INVALID_PATH', 103), ('PATIENCE_EXCEEDED', 104),
        ('FAILED_TO_MAKE_PROGRESS', 105), ('NO_VALID_CONTROL', 106),
        ('CONTROLLER_TIMED_OUT', 107)),
    'planificateur': (
        ('UNKNOWN', 200), ('INVALID_PLANNER', 201), ('TF_ERROR', 202),
        ('START_OUTSIDE_MAP', 203), ('GOAL_OUTSIDE_MAP', 204), ('START_OCCUPIED', 205),
        ('GOAL_OCCUPIED', 206), ('TIMEOUT', 207), ('NO_VALID_PATH', 208)),
    'pivot': (
        ('UNKNOWN', 700), ('TIMEOUT', 701), ('TF_ERROR', 702), ('COLLISION_AHEAD', 703)),
    'recul': (
        ('UNKNOWN', 710), ('TIMEOUT', 711), ('TF_ERROR', 712), ('INVALID_INPUT', 713),
        ('COLLISION_AHEAD', 714)),
}

EXPLANATIONS = {
    CAUSE_NONE: 'Nav2 ne précise pas la cause',
    CAUSE_SILENT: 'Nav2 ne répondait plus',
    CAUSE_TOO_FAR: 'Nav2 annonce l\'arrivée trop loin de la cible',
    CAUSE_CANCELED: 'navigation annulée',
    'INVALID_PATH': 'chemin vide ou invalide',
    'PATIENCE_EXCEEDED': 'un obstacle est resté devant le robot',
    'FAILED_TO_MAKE_PROGRESS': 'le robot n\'avançait plus',
    'NO_VALID_CONTROL': 'aucune commande sûre (obstacle juste devant le robot)',
    'CONTROLLER_TIMED_OUT': 'le contrôleur a mis trop de temps',
    'TF_ERROR': 'position du robot indisponible',
    'START_OUTSIDE_MAP': 'le robot est hors de la carte',
    'GOAL_OUTSIDE_MAP': 'le but est hors de la carte',
    'START_OCCUPIED': 'le robot est dans un obstacle de la costmap',
    'GOAL_OCCUPIED': 'le but est dans un obstacle',
    'TIMEOUT': 'calcul trop long',
    'NO_VALID_PATH': 'aucun chemin vers le but',
    'COLLISION_AHEAD': 'obstacle sur la trajectoire de la manœuvre',
    'UNKNOWN': 'erreur non précisée',
}


def resolve_error_names(classes: Optional[Dict[str, object]] = None) -> Dict[int, Tuple[str, str]]:
    """Table code -> (famille, nom).

    `classes` associe une famille à la classe Result du paquet nav2_msgs
    installé : ses constantes remplacent les numéros par défaut, ce qui garde
    la table juste si une autre version de Nav2 numérote autrement.
    """
    table: Dict[int, Tuple[str, str]] = {}
    for family, constants in ERROR_FAMILIES.items():
        cls = (classes or {}).get(family)
        for name, default in constants:
            code = getattr(cls, name, None) if cls is not None else None
            if not isinstance(code, int) or isinstance(code, bool) or code <= 0:
                code = default
            table.setdefault(code, (family, name))
    return table


def describe_error(code: Optional[int], table: Dict[int, Tuple[str, str]]) -> Tuple[str, str]:
    """(cause, phrase pour le journal) à partir du code d'erreur de Nav2."""
    if not code:
        return CAUSE_NONE, EXPLANATIONS[CAUSE_NONE]
    if code not in table:
        return f'CODE_{code}', f'code d\'erreur Nav2 {code}'
    family, name = table[code]
    return name, f'{family} : {EXPLANATIONS.get(name, name)} ({name}, code {code})'


def count_causes(causes: Iterable[str]) -> str:
    """« NO_VALID_PATH x2; NONE x1 » : résumé stable pour le rapport."""
    counts: Dict[str, int] = {}
    for cause in causes:
        counts[cause] = counts.get(cause, 0) + 1
    return '; '.join(f'{name} x{counts[name]}' for name in sorted(counts))


class RetryPolicy:
    """Suite à donner à un échec, selon sa cause et le progrès du robot.

    Un échec « sans progrès » est un échec survenu alors que le robot ne s'est
    pas rapproché du but d'au moins `progress_min` depuis le meilleur point
    atteint. Échelle appliquée aux échecs sans progrès consécutifs :

        1er, 3e, 5e...  -> CLEAR_FAR : on efface la costmap globale loin du
                           robot et on garde ce qui l'entoure (les plateaux de
                           table ne sont vus que vers l'avant) ;
        2e, 4e, 6e...   -> FALLBACK si le repli est permis (l'appelant passe à
                           CLEAR_ALL si aucun point de repli n'existe),
                           sinon CLEAR_ALL.

    Deux cas court-circuitent l'échelle :
        but occupé      -> FALLBACK tout de suite : réessayer ne sert à rien.
                           Nav2 ne le dit presque jamais lui-même : avec une
                           tolérance de planification non nulle, Smac répond
                           NO_VALID_PATH. C'est donc l'appelant qui lit la
                           costmap et passe `goal_blocked=True`. Le code
                           GOAL_OCCUPIED est traité de la même façon ;
        START_OCCUPIED  -> CLEAR_NEAR au premier échec sans progrès : un
                           obstacle fantôme sous le robot l'explique souvent.
                           S'il se répète, l'échelle normale reprend.
        TF_ERROR        -> RESEND : Nav2 n'a pas encore reçu la position du
                           robot (cas typique : tout premier but, juste après
                           son démarrage). Effacer les costmaps n'y changerait
                           rien et ferait perdre deux secondes.
        échec immédiat  -> RESEND aussi : sans code d'erreur, moins d'une
        (immediate)        seconde après l'envoi, robot immobile. C'est la
                           signature d'un serveur que l'arbre de Nav2 vient
                           de créer et qui n'a pas encore « vu » son
                           interlocuteur (observé sur notre banc d'essai au
                           premier but). Au-delà de `resend_limit` renvois de
                           suite, l'échelle normale reprend.
    """

    def __init__(self, progress_min: float = 0.25, fallback_after: int = 2,
                 fallback_enabled: bool = True, resend_limit: int = 5):
        self.progress_min = float(progress_min)
        self.fallback_after = max(1, int(fallback_after))
        self.fallback_enabled = bool(fallback_enabled)
        self.resend_limit = max(0, int(resend_limit))
        self.stalls = 0
        self._resends = 0
        self._best: Optional[float] = None

    def reset(self, distance: Optional[float]) -> None:
        """Nouvelle cible (ou départ) : on repart de la distance courante."""
        self._best = distance
        self.stalls = 0
        self._resends = 0

    def on_failure(self, distance: Optional[float], cause: str,
                   goal_blocked: bool = False, immediate: bool = False) -> str:
        glitch = cause == CAUSE_TF or (immediate and cause == CAUSE_NONE)
        if glitch and self._resends < self.resend_limit:
            # Ni un blocage ni un obstacle : l'échelle des nettoyages n'avance pas.
            self._resends += 1
            return RESEND
        self._resends = 0
        progressed = (distance is not None and self._best is not None
                      and self._best - distance >= self.progress_min)
        if self._best is None and distance is not None:
            self._best = distance
        if progressed:
            self._best = distance
            self.stalls = 0
        else:
            self.stalls += 1

        if (goal_blocked or cause == 'GOAL_OCCUPIED') and self.fallback_enabled:
            return FALLBACK
        if cause == 'START_OCCUPIED' and self.stalls <= 1:
            return CLEAR_NEAR
        if self.stalls == 0 or self.stalls % self.fallback_after != 0:
            return CLEAR_FAR
        return FALLBACK if self.fallback_enabled else CLEAR_ALL
