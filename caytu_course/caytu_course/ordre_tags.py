"""Lecture de l'ordre des tags publié par les organisateurs.

Le format exact n'est pas encore publié : on accepte une liste d'entiers, du
JSON (liste ou objet avec une clé `order`, `ordre`, `tags` ou `tag_order`) ou
un texte libre contenant les identifiants (« 7,1,9,4 », « 7 1 9 4 »...).
L'ordre n'est accepté que s'il contient exactement chaque tag attendu une fois.
"""

import json
import re

CLES_JSON = ('order', 'ordre', 'tags', 'tag_order')


class OrdreInvalide(ValueError):
    """L'ordre reçu n'est pas une permutation des tags attendus."""


def _extraire_entiers(valeur):
    if isinstance(valeur, (list, tuple)):
        return [int(element) for element in valeur]
    texte = str(valeur).strip()
    try:
        contenu = json.loads(texte)
    except (json.JSONDecodeError, ValueError):
        contenu = None
    if isinstance(contenu, dict):
        for cle in CLES_JSON:
            if isinstance(contenu.get(cle), list):
                return [int(element) for element in contenu[cle]]
        return []
    if isinstance(contenu, list):
        return [int(element) for element in contenu]
    return [int(nombre) for nombre in re.findall(r'\d+', texte)]


def lire_ordre(valeur, tags_attendus):
    """Retourne l'ordre (tuple d'entiers) ou lève `OrdreInvalide`."""
    try:
        ordre = tuple(_extraire_entiers(valeur))
    except (TypeError, ValueError) as erreur:
        raise OrdreInvalide(f'ordre illisible : {valeur!r}') from erreur
    if sorted(ordre) != sorted(tags_attendus):
        raise OrdreInvalide(
            f'ordre {list(ordre)} : il faut chaque tag de '
            f'{sorted(tags_attendus)} exactement une fois')
    return ordre
