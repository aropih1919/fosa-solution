"""Décisions métier sûres : lieux validés, confirmation et secours déterministe."""

from dataclasses import dataclass

from caytu_assistant.models import Intention
from caytu_semantic_map.registre import normaliser


@dataclass(frozen=True)
class ResultatDispatch:
    reponse: str
    etat: str
    detail: str = ''


def _contient(texte, expression):
    normalise = f' {normaliser(texte)} '
    return f' {normaliser(expression)} ' in normalise


class AnalyseurRegles:
    """Solution de secours sans réseau pour les commandes sûres et les lieux connus."""

    def __init__(self, configuration, registre):
        self._configuration = configuration
        self._registre = registre

    def analyser(self, texte):
        mots = self._configuration.prompts.fallback_keywords
        normalise = normaliser(texte)
        confirmation = ''
        if normalise in {normaliser(mot) for mot in mots['yes']}:
            confirmation = 'oui'
            intention = 'confirmation'
        elif normalise in {normaliser(mot) for mot in mots['no']}:
            confirmation = 'non'
            intention = 'confirmation'
        elif any(_contient(texte, mot)
                 for mot in self._configuration.mission.emergency_keywords):
            intention = 'navigation'
        elif any(_contient(texte, mot) for mot in mots['stop']):
            intention = 'fin'
        elif any(_contient(texte, mot) for mot in mots['navigation']):
            intention = 'navigation'
        elif any(_contient(texte, mot) for mot in mots['localization']):
            intention = 'localiser'
        elif any(_contient(texte, mot) for mot in mots['greeting']):
            intention = 'salutation'
        elif any(_contient(texte, mot) for mot in mots['farewell']):
            intention = 'fin'
        else:
            intention = 'information'

        lieux = []
        for lieu in self._registre.tous():
            if (_contient(texte, lieu.name)
                    or any(_contient(texte, alias) for alias in lieu.aliases)):
                lieux.append(lieu.name)
        return Intention((intention,), tuple(lieux), confirmation, '', '')


class RepartiteurAssistant:
    def __init__(self, configuration, registre, modele_langage, naviguer, annuler,
                 resoudre_pose=None):
        self._configuration = configuration
        self._registre = registre
        self._modele_langage = modele_langage
        self._naviguer = naviguer
        self._annuler = annuler
        self._resoudre_pose = resoudre_pose or (lambda nom: None)
        self._secours = AnalyseurRegles(configuration, registre)
        self._lieu_en_attente = None
        self._historique = []

    def traiter(self, texte):
        reponses = self._configuration.prompts.responses
        if self._est_urgence(texte):
            urgence = self._registre.resoudre(
                self._configuration.mission.emergency_location)
            if urgence is None:
                return ResultatDispatch(
                    reponses['emergency_unconfigured'],
                    'ERROR', 'urgence sans lieu')
            return self._naviguer_vers(urgence, confirmation=False)

        if any(_contient(texte, mot)
               for mot in self._configuration.prompts.fallback_keywords['stop']):
            self._annuler()
            self._lieu_en_attente = None
            return ResultatDispatch(reponses['navigation_cancelled'], 'IDLE', 'annulation')

        try:
            intention_structuree = self._modele_langage.interpreter(texte, self._historique)
        except Exception:
            intention_structuree = self._secours.analyser(texte)
        self._memoriser(texte, intention_structuree.reponse)
        intention = intention_structuree.intentions[0]

        if intention == 'fin':
            self._lieu_en_attente = None
            return ResultatDispatch(reponses['farewell'], 'IDLE')
        if intention == 'salutation':
            reponse = (self._configuration.dialogue.greeting_en_reply
                     if intention_structuree.langue == 'en'
                     else self._configuration.dialogue.greeting_reply)
            return ResultatDispatch(reponse, 'IDLE')
        if intention == 'confirmation':
            if intention_structuree.confirmation == 'non':
                self._lieu_en_attente = None
                return ResultatDispatch(reponses['declined'], 'IDLE')
            if intention_structuree.confirmation == 'oui' and self._lieu_en_attente:
                return self._naviguer_vers(
                    self._lieu_en_attente, confirmation=False)
            return ResultatDispatch(
                reponses['no_pending_destination'], 'IDLE')
        if intention == 'information':
            reponse = self._configuration.knowledge.find_answer(texte)
            if reponse is None:
                reponse = self._configuration.dialogue.unknown_information_reply
            return ResultatDispatch(reponse, 'ANSWERING')

        lieu = self._resoudre_lieu_intention(intention_structuree)
        if lieu is None:
            return ResultatDispatch(
                self._configuration.dialogue.unknown_location_reply,
                'IDLE', 'lieu absent ou inconnu')
        if intention == 'localiser':
            self._lieu_en_attente = lieu
            return ResultatDispatch(
                reponses['location_question'].format(description=lieu.description),
                'CONFIRMING', lieu.name)
        if intention == 'navigation':
            return self._naviguer_vers(
                lieu,
                confirmation=self._configuration.dialogue.confirmation_required)
        return ResultatDispatch(reponses['fallback'], 'IDLE')

    def _est_urgence(self, texte):
        return any(_contient(texte, keyword)
                   for keyword in self._configuration.mission.emergency_keywords)

    def _resoudre_lieu_intention(self, intention):
        for valeur in intention.lieux:
            lieu = self._registre.resoudre(valeur)
            if lieu is not None:
                return lieu
        # Une intention LLM ne peut nommer qu’un lieu présent dans le registre.
        return None

    def _naviguer_vers(self, lieu, confirmation):
        reponses = self._configuration.prompts.responses
        if confirmation:
            self._lieu_en_attente = lieu
            return ResultatDispatch(
                reponses['navigation_confirmation'].format(name=lieu.name),
                'CONFIRMING', lieu.name)
        self._lieu_en_attente = None
        pose = self._resoudre_pose(lieu.name)
        if pose is None or not pose.has_approach_pose:
            return ResultatDispatch(
                reponses['navigation_pose_missing'].format(name=lieu.name),
                'ERROR', f'pose absente: {lieu.name}')
        succes, detail = self._naviguer(pose.approach_pose)
        if succes:
            return ResultatDispatch(
                reponses['navigation_success'].format(name=lieu.name), 'ARRIVED')
        return ResultatDispatch(
            reponses['navigation_failure'].format(name=lieu.name), 'ERROR', detail)

    def _memoriser(self, texte_visiteur, reponse):
        self._historique.extend([
            {'role': 'user', 'content': texte_visiteur},
            {'role': 'assistant', 'content': reponse or ''},
        ])
        maximum = self._configuration.dialogue.max_turns_memory * 2
        self._historique = self._historique[-maximum:] if maximum else []
