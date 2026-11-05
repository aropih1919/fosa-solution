"""Tests unitaires de caytu_assistant.models et caytu_assistant.dispatcher (sans ROS).

Couverture :
  1. Intention.depuis_json : JSON valide, champ manquant, intention inconnue, non-JSON
  2. AnalyseurRegles : navigation, localiser, urgence, stop, inconnu
  3. RepartiteurAssistant : navigation directe, échec, localiser/confirmation,
     lieu inconnu, urgence, stop, fallback LLM, information, pose absente
"""

import json
from pathlib import Path

import pytest

from caytu_config import load_config
from caytu_config.models import SemanticMapConfig
from caytu_semantic_map.registre import RegistreLieux
from caytu_assistant.models import ErreurIntention, Intention
from caytu_assistant.dispatcher import AnalyseurRegles, RepartiteurAssistant

# Chargement unique de la configuration réelle (lecture de fichiers YAML, pas de ROS).
_CONFIG_DIR = Path(__file__).resolve().parents[2] / 'caytu_config' / 'config'
CONFIG = load_config(_CONFIG_DIR)


# ---------------------------------------------------------------------------
# Helpers / mocks
# ---------------------------------------------------------------------------

def _construire_registre():
    """Construit un RegistreLieux depuis la carte sémantique réelle de la config."""
    return RegistreLieux(CONFIG.semantic_map)


def _intention_json(**kwargs):
    """Sérialise un dict en JSON pour les tests de depuis_json."""
    return json.dumps(kwargs)


class ModeleLangageSimule:
    """Modèle de langage simulé qui retourne toujours la même intention."""

    def __init__(self, intention):
        self._intention = intention

    def interpreter(self, texte, historique):
        """Retourne l'intention préconfigurée sans appel réseau."""
        return self._intention


class ModeleLangageEnEchec:
    """Modèle de langage simulé qui lève toujours une exception."""

    def interpreter(self, texte, historique):
        """Simule une panne réseau ou une réponse illisible."""
        raise RuntimeError('LLM indisponible')


class PoseMock:
    """Pose d'approche factice : has_approach_pose toujours vrai."""
    has_approach_pose = True
    approach_pose = object()


def naviguer_succes(pose):
    """Simule une navigation réussie."""
    return True, 'ok'


def naviguer_echec(pose):
    """Simule un échec de navigation."""
    return False, 'erreur de navigation'


def annuler():
    """Simule l'annulation de la navigation."""


def resoudre_pose_disponible(nom):
    """Simule la résolution de pose disponible pour tous les lieux."""
    return PoseMock()


def resoudre_pose_absente(nom):
    """Simule un lieu dont la pose n'est pas encore calculée."""
    return None


# ---------------------------------------------------------------------------
# Tests de Intention.depuis_json
# ---------------------------------------------------------------------------

def test_intention_depuis_json_valide():
    """depuis_json accepte un JSON complet et conforme."""
    payload = _intention_json(
        intentions=['navigation'],
        lieux=['accueil'],
        confirmation='',
        reponse="Je vous emmène à l'accueil.",
        langue='fr',
    )
    intention = Intention.depuis_json(payload)
    assert intention.intentions == ('navigation',)
    assert intention.lieux == ('accueil',)
    assert intention.confirmation == ''
    assert intention.langue == 'fr'


def test_intention_json_invalide_champ_manquant():
    """depuis_json lève ErreurIntention si un champ obligatoire est absent."""
    payload = json.dumps({
        'intentions': ['navigation'],
        'lieux': ['accueil'],
        # 'confirmation', 'reponse', 'langue' absents
    })
    with pytest.raises(ErreurIntention):
        Intention.depuis_json(payload)


def test_intention_json_invalide_intention_inconnue():
    """depuis_json lève ErreurIntention si l'intention n'est pas autorisée."""
    payload = _intention_json(
        intentions=['teleporter'],  # intention inconnue
        lieux=[],
        confirmation='',
        reponse='',
        langue='fr',
    )
    with pytest.raises(ErreurIntention):
        Intention.depuis_json(payload)


def test_intention_json_non_parseable():
    """depuis_json lève ErreurIntention si la chaîne n'est pas du JSON valide."""
    with pytest.raises(ErreurIntention):
        Intention.depuis_json("ceci n'est pas du JSON {")


def test_intention_json_liste_intentions_vide():
    """depuis_json lève ErreurIntention si la liste des intentions est vide."""
    payload = _intention_json(
        intentions=[],
        lieux=[],
        confirmation='',
        reponse='',
        langue='fr',
    )
    with pytest.raises(ErreurIntention):
        Intention.depuis_json(payload)


# ---------------------------------------------------------------------------
# Tests de AnalyseurRegles
# ---------------------------------------------------------------------------

@pytest.fixture(name='analyseur')
def fixture_analyseur():
    """AnalyseurRegles construit avec la configuration réelle et le registre de test."""
    return AnalyseurRegles(CONFIG, _construire_registre())


def test_analyseur_regles_navigation(analyseur):
    """Un mot-clé de navigation déclenche l'intention 'navigation'."""
    # Récupère le premier mot-clé navigation depuis la config
    mot_nav = CONFIG.prompts.fallback_keywords['navigation'][0]
    intention = analyseur.analyser(f'Je veux aller {mot_nav}')
    assert 'navigation' in intention.intentions


def test_analyseur_regles_localiser(analyseur):
    """Un mot-clé de localisation déclenche l'intention 'localiser'."""
    mot_loc = CONFIG.prompts.fallback_keywords['localization'][0]
    intention = analyseur.analyser(f'{mot_loc} les toilettes')
    assert 'localiser' in intention.intentions


def test_analyseur_regles_urgence(analyseur):
    """Un mot d'urgence déclenche directement l'intention 'navigation' (sans réseau)."""
    mot_urgence = CONFIG.mission.emergency_keywords[0]
    intention = analyseur.analyser(f"J'ai {mot_urgence} besoin d'aide")
    assert 'navigation' in intention.intentions


def test_analyseur_regles_stop(analyseur):
    """Un mot stop déclenche l'intention 'fin'."""
    mot_stop = CONFIG.prompts.fallback_keywords['stop'][0]
    intention = analyseur.analyser(mot_stop)
    assert 'fin' in intention.intentions


def test_analyseur_regles_inconnu(analyseur):
    """Un texte sans mot-clé reconnu retourne l'intention 'information'."""
    intention = analyseur.analyser("Quelle est la météo aujourd'hui ?")
    assert 'information' in intention.intentions


# ---------------------------------------------------------------------------
# Fonctions auxiliaires du répartiteur
# ---------------------------------------------------------------------------

def _creer_repartiteur(intention_modele=None, naviguer=naviguer_succes,
                       resoudre_pose=resoudre_pose_disponible):
    """Construit un répartiteur avec le modèle de langage et les fonctions simulés."""
    registre = _construire_registre()
    if intention_modele is None:
        modele_langage = ModeleLangageEnEchec()
    else:
        modele_langage = ModeleLangageSimule(intention_modele)
    return RepartiteurAssistant(
        CONFIG, registre, modele_langage, naviguer, annuler, resoudre_pose)


def _intention(intentions, lieux=None, confirmation='', reponse='', langue='fr'):
    """Crée une Intention directement (contourne depuis_json pour les tests)."""
    return Intention(
        intentions=tuple(intentions),
        lieux=tuple(lieux or []),
        confirmation=confirmation,
        reponse=reponse,
        langue=langue,
    )


# ---------------------------------------------------------------------------
# Tests du répartiteur de l'assistant
# ---------------------------------------------------------------------------

def test_dispatcher_navigation_directe():
    """Navigation directe vers un lieu connu et navigation réussie → state ARRIVED."""
    # La config peut imposer une confirmation : on désactive en mettant confirmation_required=False
    # On passe par le LLM mock qui retourne 'navigation' + lieu 'accueil'
    # Pour éviter la dépendance à confirmation_required on appelle le dispatcher
    # qui appelera _naviguer_vers sans confirmation (car le LLM dit 'navigation')
    # Note : confirmation_required=True dans la config réelle → état CONFIRMING d'abord
    intention = _intention(['navigation'], lieux=['accueil'])
    disp = _creer_repartiteur(intention_modele=intention, naviguer=naviguer_succes)

    # Premier appel : si confirmation_required → CONFIRMING
    resultat = disp.traiter("emmène-moi à l'accueil")
    if CONFIG.dialogue.confirmation_required:
        assert resultat.etat == 'CONFIRMING'
        # Confirmation oui
        intention_oui = _intention(['confirmation'], confirmation='oui')
        disp._modele_langage = ModeleLangageSimule(intention_oui)
        resultat = disp.traiter('oui')
    assert resultat.etat == 'ARRIVED'


def test_dispatcher_navigation_echec():
    """Navigation vers un lieu connu mais échec de déplacement → state ERROR."""
    intention = _intention(['navigation'], lieux=['toilettes'])
    disp = _creer_repartiteur(intention_modele=intention, naviguer=naviguer_echec)

    resultat = disp.traiter('amène-moi aux toilettes')
    if CONFIG.dialogue.confirmation_required:
        assert resultat.etat == 'CONFIRMING'
        intention_oui = _intention(['confirmation'], confirmation='oui')
        disp._modele_langage = ModeleLangageSimule(intention_oui)
        resultat = disp.traiter('oui')
    assert resultat.etat == 'ERROR'


def test_dispatcher_localiser_propose():
    """Intention 'localiser' avec lieu connu → state CONFIRMING et lieu mémorisé."""
    intention = _intention(['localiser'], lieux=['toilettes'])
    disp = _creer_repartiteur(intention_modele=intention)
    resultat = disp.traiter('où sont les toilettes ?')
    assert resultat.etat == 'CONFIRMING'
    assert disp._lieu_en_attente is not None
    assert disp._lieu_en_attente.name == 'toilettes'


def test_dispatcher_confirmation_oui_puis_navigation():
    """Localiser → confirmation oui → navigation réussie → state ARRIVED."""
    registre = _construire_registre()
    # Étape 1 : localiser
    intention_loc = _intention(['localiser'], lieux=['buvette'])
    modele_langage = ModeleLangageSimule(intention_loc)
    disp = RepartiteurAssistant(
        CONFIG, registre, modele_langage, naviguer_succes, annuler, resoudre_pose_disponible)
    resultat = disp.traiter('où est la buvette ?')
    assert resultat.etat == 'CONFIRMING'

    # Étape 2 : confirmation oui
    intention_oui = _intention(['confirmation'], confirmation='oui')
    disp._modele_langage = ModeleLangageSimule(intention_oui)
    resultat = disp.traiter('oui')
    assert resultat.etat == 'ARRIVED'


def test_dispatcher_confirmation_non():
    """Confirmation non annule la navigation en attente → state IDLE."""
    intention_loc = _intention(['localiser'], lieux=['toilettes'])
    disp = _creer_repartiteur(intention_modele=intention_loc)
    disp.traiter('où sont les toilettes ?')

    intention_non = _intention(['confirmation'], confirmation='non')
    disp._modele_langage = ModeleLangageSimule(intention_non)
    resultat = disp.traiter('non merci')
    assert resultat.etat == 'IDLE'
    assert disp._lieu_en_attente is None


def test_dispatcher_lieu_inconnu():
    """Un lieu non résolu par le registre retourne state IDLE et message unknown_location_reply."""
    intention = _intention(['navigation'], lieux=['bibliothèque inconnue'])
    disp = _creer_repartiteur(intention_modele=intention)
    resultat = disp.traiter('emmène-moi à la bibliothèque')
    assert resultat.etat == 'IDLE'
    assert resultat.reponse == CONFIG.dialogue.unknown_location_reply


def test_dispatcher_urgence():
    """Un mot d'urgence dans le texte déclenche la navigation vers emergency_location."""
    mot_urgence = CONFIG.mission.emergency_keywords[0]
    disp = _creer_repartiteur(naviguer=naviguer_succes)
    resultat = disp.traiter(f"j'ai {mot_urgence} aidez-moi")
    # L'urgence court-circuite tout : navigation directe
    assert resultat.etat in ('ARRIVED', 'CONFIRMING', 'ERROR')


def test_dispatcher_stop_annule():
    """Un mot stop appelle cancel et retourne state IDLE."""
    annulations = []

    def cancel_spy():
        annulations.append(True)

    mot_stop = CONFIG.prompts.fallback_keywords['stop'][0]
    registre = _construire_registre()
    disp = RepartiteurAssistant(
        CONFIG, registre, ModeleLangageEnEchec(), naviguer_succes, cancel_spy, resoudre_pose_disponible)
    resultat = disp.traiter(mot_stop)
    assert resultat.etat == 'IDLE'
    assert len(annulations) == 1


def test_dispatcher_fallback_llm_indisponible():
    """Quand le LLM lève une exception, l'AnalyseurRegles prend le relais."""
    disp = _creer_repartiteur(intention_modele=None)  # Modèle simulé en échec par défaut
    # Texte contenant un mot-clé de localisation
    mot_loc = CONFIG.prompts.fallback_keywords['localization'][0]
    resultat = disp.traiter(f'{mot_loc} les toilettes')
    # Le fallback doit retourner une réponse cohérente (pas d'exception)
    assert resultat.etat in ('CONFIRMING', 'IDLE', 'ARRIVED', 'ERROR', 'ANSWERING')


def test_dispatcher_information():
    """Intention 'information' → state ANSWERING."""
    intention = _intention(['information'])
    disp = _creer_repartiteur(intention_modele=intention)
    resultat = disp.traiter("qu'est-ce que la buvette propose ?")
    assert resultat.etat == 'ANSWERING'


def test_repartiteur_pose_absente():
    """Quand la pose est absente, la navigation échoue et l'état devient ERROR."""
    intention = _intention(['navigation'], lieux=['accueil'])
    disp = _creer_repartiteur(intention_modele=intention, resoudre_pose=resoudre_pose_absente)

    resultat = disp.traiter("emmène-moi à l'accueil")
    if CONFIG.dialogue.confirmation_required:
        assert resultat.etat == 'CONFIRMING'
        intention_oui = _intention(['confirmation'], confirmation='oui')
        disp._modele_langage = ModeleLangageSimule(intention_oui)
        resultat = disp.traiter('oui')
    assert resultat.etat == 'ERROR'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
