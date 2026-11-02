"""Décisions métier sûres : lieux validés, confirmation et secours déterministe."""

from dataclasses import dataclass

from caytu_assistant.models import Intention
from caytu_semantic_map.registry import normaliser


@dataclass(frozen=True)
class ResultatDispatch:
    response: str
    state: str
    detail: str = ''


def _contient(texte, expression):
    normalise = f' {normaliser(texte)} '
    return f' {normaliser(expression)} ' in normalise


class AnalyseurRegles:
    """Fallback sans réseau : reconnaît les commandes sûres et les lieux connus."""

    def __init__(self, configuration, registry):
        self._configuration = configuration
        self._registry = registry

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
        for lieu in self._registry.tous():
            if (_contient(texte, lieu.name)
                    or any(_contient(texte, alias) for alias in lieu.aliases)):
                lieux.append(lieu.name)
        return Intention((intention,), tuple(lieux), confirmation, '', '')


class DispatcherAssistant:
    def __init__(self, configuration, registry, llm, navigate, cancel,
                 resolve_pose=None):
        self._configuration = configuration
        self._registry = registry
        self._llm = llm
        self._navigate = navigate
        self._cancel = cancel
        self._resolve_pose = resolve_pose or (lambda name: None)
        self._fallback = AnalyseurRegles(configuration, registry)
        self._pending_location = None
        self._history = []

    def traiter(self, texte):
        reponses = self._configuration.prompts.responses
        if self._est_urgence(texte):
            emergency = self._registry.resoudre(
                self._configuration.mission.emergency_location)
            if emergency is None:
                return ResultatDispatch(
                    reponses['emergency_unconfigured'],
                    'ERROR', 'urgence sans lieu')
            return self._naviguer_vers(emergency, confirmation=False)

        if any(_contient(texte, mot)
               for mot in self._configuration.prompts.fallback_keywords['stop']):
            self._cancel()
            self._pending_location = None
            return ResultatDispatch(reponses['navigation_cancelled'], 'IDLE', 'annulation')

        try:
            intent = self._llm.interpret(texte, self._history)
        except Exception:
            intent = self._fallback.analyser(texte)
        self._memoriser(texte, intent.reponse)
        intention = intent.intentions[0]

        if intention == 'fin':
            self._pending_location = None
            return ResultatDispatch(reponses['farewell'], 'IDLE')
        if intention == 'salutation':
            reply = (self._configuration.dialogue.greeting_en_reply
                     if intent.langue == 'en'
                     else self._configuration.dialogue.greeting_reply)
            return ResultatDispatch(reply, 'IDLE')
        if intention == 'confirmation':
            if intent.confirmation == 'non':
                self._pending_location = None
                return ResultatDispatch(reponses['declined'], 'IDLE')
            if intent.confirmation == 'oui' and self._pending_location:
                return self._naviguer_vers(
                    self._pending_location, confirmation=False)
            return ResultatDispatch(
                reponses['no_pending_destination'], 'IDLE')
        if intention == 'information':
            answer = self._configuration.knowledge.find_answer(texte)
            if answer is None:
                answer = self._configuration.dialogue.unknown_information_reply
            return ResultatDispatch(answer, 'ANSWERING')

        location = self._resoudre_lieu_intention(intent)
        if location is None:
            return ResultatDispatch(
                self._configuration.dialogue.unknown_location_reply,
                'IDLE', 'lieu absent ou inconnu')
        if intention == 'localiser':
            self._pending_location = location
            return ResultatDispatch(
                reponses['location_question'].format(description=location.description),
                'CONFIRMING', location.name)
        if intention == 'navigation':
            return self._naviguer_vers(
                location,
                confirmation=self._configuration.dialogue.confirmation_required)
        return ResultatDispatch(reponses['fallback'], 'IDLE')

    def _est_urgence(self, texte):
        return any(_contient(texte, keyword)
                   for keyword in self._configuration.mission.emergency_keywords)

    def _resoudre_lieu_intention(self, intention):
        for valeur in intention.lieux:
            lieu = self._registry.resoudre(valeur)
            if lieu is not None:
                return lieu
        # Une intention LLM ne peut nommer qu’un lieu présent dans le registre.
        return None

    def _naviguer_vers(self, lieu, confirmation):
        reponses = self._configuration.prompts.responses
        if confirmation:
            self._pending_location = lieu
            return ResultatDispatch(
                reponses['navigation_confirmation'].format(name=lieu.name),
                'CONFIRMING', lieu.name)
        self._pending_location = None
        pose = self._resolve_pose(lieu.name)
        if pose is None or not pose.has_approach_pose:
            return ResultatDispatch(
                reponses['navigation_pose_missing'].format(name=lieu.name),
                'ERROR', f'pose absente: {lieu.name}')
        success, detail = self._navigate(pose.approach_pose)
        if success:
            return ResultatDispatch(
                reponses['navigation_success'].format(name=lieu.name), 'ARRIVED')
        return ResultatDispatch(
            reponses['navigation_failure'].format(name=lieu.name), 'ERROR', detail)

    def _memoriser(self, texte_visiteur, reponse):
        self._history.extend([
            {'role': 'user', 'content': texte_visiteur},
            {'role': 'assistant', 'content': reponse or ''},
        ])
        maximum = self._configuration.dialogue.max_turns_memory * 2
        self._history = self._history[-maximum:] if maximum else []
