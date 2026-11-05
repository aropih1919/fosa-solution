"""Politique déterministe de reprise après un échec de navigation."""


def doit_conserver_cible(decision_strategie, echecs, reprises_max):
    """Protège la cible après un échec jusqu'aux reprises configurées."""
    return bool(decision_strategie) or (0 < echecs <= reprises_max)
