"""Briques de validation partagées par les schémas de configuration.

Chaque fonction lève `ConfigurationError` avec le chemin exact de la clé
fautive. Aucune valeur par défaut métier n'est définie ici.
"""

import unicodedata

from caytu_config.errors import ConfigurationError

_MISSING = object()


# --------------------------------------------------------------------- outils
def _section(value, path):
    if not isinstance(value, dict):
        raise ConfigurationError(f"{path} doit être un mapping YAML.")
    return value


def _check_keys(data, allowed, path):
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ConfigurationError(
            f"{path}: clé(s) inconnue(s) : {', '.join(unknown)}")


def _get(data, key, path, default=_MISSING):
    if key in data:
        return data[key]
    if default is _MISSING:
        raise ConfigurationError(f"{path}.{key} est obligatoire.")
    return default


def _as_str(value, path, allow_empty=False):
    if not isinstance(value, str):
        raise ConfigurationError(f"{path} doit être une chaîne.")
    if not allow_empty and not value.strip():
        raise ConfigurationError(f"{path} ne doit pas être vide.")
    return value


def _as_bool(value, path):
    if not isinstance(value, bool):
        raise ConfigurationError(f"{path} doit être un booléen.")
    return value


def _as_int(value, path, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{path} doit être un entier.")
    if minimum is not None and value < minimum:
        raise ConfigurationError(f"{path} doit être >= {minimum}.")
    if maximum is not None and value > maximum:
        raise ConfigurationError(f"{path} doit être <= {maximum}.")
    return value


def _as_number(value, path, minimum=None, maximum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{path} doit être un nombre.")
    value = float(value)
    if positive and value <= 0.0:
        raise ConfigurationError(f"{path} doit être strictement positif.")
    if minimum is not None and value < minimum:
        raise ConfigurationError(f"{path} doit être >= {minimum}.")
    if maximum is not None and value > maximum:
        raise ConfigurationError(f"{path} doit être <= {maximum}.")
    return value


def _as_list(value, path):
    if not isinstance(value, list):
        raise ConfigurationError(f"{path} doit être une liste.")
    return value


def _as_numbers(value, path, positive=False):
    elements = _as_list(value, path)
    if not elements:
        raise ConfigurationError(f"{path} ne doit pas être vide.")
    return tuple(
        _as_number(element, f"{path}[{index}]", positive=positive)
        for index, element in enumerate(elements))


def _choice(value, choices, path):
    value = _as_str(value, path)
    if value not in choices:
        raise ConfigurationError(
            f"{path} doit valoir l'un de : {', '.join(choices)}.")
    return value


def _normaliser(texte):
    """Minuscules, sans accents, espaces uniques : comparaison de lieux."""
    decompose = unicodedata.normalize("NFKD", texte)
    sans_accents = "".join(c for c in decompose if not unicodedata.combining(c))
    return " ".join(
        sans_accents.casefold().replace("_", " ").replace("-", " ").split())
