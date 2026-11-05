"""Validation stricte des intentions produites par le modèle de langage."""

from dataclasses import dataclass
import json


INTENTIONS_AUTORISEES = {
    'navigation', 'localiser', 'information', 'confirmation',
    'salutation', 'fin', 'autre',
}
CONFIRMATIONS_AUTORISEES = {'oui', 'non', ''}
LANGUES_AUTORISEES = {'fr', 'en', ''}


class ErreurIntention(ValueError):
    """Le modèle n’a pas retourné une intention conforme au contrat."""


@dataclass(frozen=True)
class Intention:
    intentions: tuple
    lieux: tuple
    confirmation: str
    reponse: str
    langue: str

    @classmethod
    def depuis_json(cls, payload):
        try:
            data = json.loads(payload)
        except (TypeError, json.JSONDecodeError) as error:
            raise ErreurIntention('Le modèle n’a pas retourné un JSON valide.') from error
        if not isinstance(data, dict):
            raise ErreurIntention('L’intention doit être un objet JSON.')
        expected = {'intentions', 'lieux', 'confirmation', 'reponse', 'langue'}
        if set(data) != expected:
            raise ErreurIntention('Les champs JSON de l’intention sont invalides.')
        intentions = data['intentions']
        lieux = data['lieux']
        confirmation = data['confirmation']
        reponse = data['reponse']
        langue = data['langue']
        if (not isinstance(intentions, list) or not intentions
                or any(item not in INTENTIONS_AUTORISEES for item in intentions)):
            raise ErreurIntention('La liste des intentions est invalide.')
        if not isinstance(lieux, list) or any(not isinstance(item, str) for item in lieux):
            raise ErreurIntention('La liste des lieux est invalide.')
        if confirmation not in CONFIRMATIONS_AUTORISEES:
            raise ErreurIntention('La confirmation doit valoir oui, non ou vide.')
        if not isinstance(reponse, str) or not isinstance(langue, str):
            raise ErreurIntention('La réponse et la langue doivent être des chaînes.')
        if langue not in LANGUES_AUTORISEES:
            raise ErreurIntention('La langue doit valoir fr, en ou vide.')
        return cls(tuple(intentions), tuple(lieux), confirmation, reponse, langue)
