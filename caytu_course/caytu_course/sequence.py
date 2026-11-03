"""Séquence de visite des tags et partage du temps de course.

Règlement : les tags se visitent dans l'ordre publié ; un tag introuvable peut
être sauté mais ne peut plus être visité ensuite.
"""


class SequenceTags:

    def __init__(self, ordre):
        self.ordre = tuple(ordre)
        self.visites = []
        self.sautes = []
        self._index = 0

    def cible(self):
        """Tag à visiter maintenant, ou None si la séquence est terminée."""
        return self.ordre[self._index] if not self.terminee() else None

    def restants(self):
        """Nombre de tags encore à traiter, cible comprise."""
        return len(self.ordre) - self._index

    def terminee(self):
        return self._index >= len(self.ordre)

    def est_attendu(self, tag_id):
        return tag_id == self.cible()

    def marquer_visite(self, tag_id):
        if not self.est_attendu(tag_id):
            raise ValueError(
                f'visite du tag {tag_id} refusée : tag attendu {self.cible()}')
        self.visites.append(tag_id)
        self._index += 1

    def sauter(self):
        if self.terminee():
            raise ValueError('aucun tag à sauter : séquence terminée')
        self.sautes.append(self.cible())
        self._index += 1

    def resume(self):
        return {
            'ordre': list(self.ordre),
            'visites': list(self.visites),
            'sautes': list(self.sautes),
        }


def budget_pour_tag(temps_restant, tags_restants, minimum):
    """Temps accordé à la cible : part égale, au moins `minimum`, jamais plus
    que le temps restant."""
    temps_restant = max(0.0, float(temps_restant))
    if tags_restants <= 0:
        return 0.0
    part = temps_restant / tags_restants
    return min(temps_restant, max(part, float(minimum)))
