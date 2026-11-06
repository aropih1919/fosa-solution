"""Séquence de visite des tags.

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
        """Tags encore à traiter, dans l'ordre, cible comprise."""
        return self.ordre[self._index:]

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

