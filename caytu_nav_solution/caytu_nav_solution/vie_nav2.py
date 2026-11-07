"""Nœuds Nav2 dont le lifecycle doit être actif avant le premier but.

Ce module reste sans dépendance ROS : il est testable sans rclpy et partagé
par le lancement et le client Nav2.
"""

NOEUDS_GERES = (
    'slam_toolbox',
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
)

# lifecycle_msgs/State : PRIMARY_STATE_ACTIVE = 3 (constante du protocole).
ETAT_ACTIF = 3


def noeuds_attendus(mode):
    """Nœuds dont l'état ACTIVE est exigé selon le mode de navigation."""
    if mode == 'sans_carte':
        return tuple(nom for nom in NOEUDS_GERES if nom != 'slam_toolbox')
    return NOEUDS_GERES


def etats_inactifs(etats, mode='carte'):
    """Noms attendus qui ne sont pas ACTIVE (absents compris)."""
    return tuple(nom for nom in noeuds_attendus(mode)
                 if etats.get(nom) != ETAT_ACTIF)
