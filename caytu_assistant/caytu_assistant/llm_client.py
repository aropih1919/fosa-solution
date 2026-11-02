"""Client de chat structuré pour Ollama et les API compatibles OpenAI, sans commande moteur."""

import json
import os

from caytu_assistant.models import ErreurIntention, Intention


SCHEMA_INTENT = {
    'type': 'object',
    'properties': {
        'intentions': {
            'type': 'array', 'items': {'type': 'string'},
        },
        'lieux': {'type': 'array', 'items': {'type': 'string'}},
        'confirmation': {'type': 'string', 'enum': ['oui', 'non', '']},
        'reponse': {'type': 'string'},
        'langue': {'type': 'string', 'enum': ['fr', 'en', '']},
    },
    'required': ['intentions', 'lieux', 'confirmation', 'reponse', 'langue'],
    'additionalProperties': False,
}


class LLMIndisponible(RuntimeError):
    """Aucun fournisseur LLM n’a répondu dans les délais configurés."""


class ClientLLM:
    """Client de communication avec le modèle de langage (Ollama local ou API).

    Accessible pour un développeur débutant-intermédiaire :
      - `interpreter()` prend la phrase du visiteur et l'historique de conversation ;
      - contacte le serveur configuré (local ou API distante) ;
      - renvoie un objet `Intention` validé et typé.
    """

    def __init__(self, configuration):
        self._configuration_llm = configuration.dialogue.llm
        self._secrets = configuration.secrets
        self._prompts = configuration.prompts
        self._lieux = configuration.semantic_map.locations
        self._chaines = {}

    def _construire_prompt_systeme(self):
        """Assemble les instructions, règles de sécurité et liste des lieux autorisés."""
        lieux = [
            {'name': item.name, 'aliases': list(item.aliases)}
            for item in self._lieux
        ]
        return (self._prompts.system + '\n\n'
                + '\n'.join(self._prompts.rules)
                + '\nLieux autorisés (noms et alias seulement): '
                + json.dumps(lieux, ensure_ascii=False))

    def _creer_chaine(self, fournisseur):
        """Construit une chaîne LangChain avec une sortie d'intention structurée."""
        if fournisseur in self._chaines:
            return self._chaines[fournisseur]

        from langchain_core.messages import SystemMessage
        from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

        if fournisseur == 'local':
            from langchain_ollama import ChatOllama

            modele = ChatOllama(
                model=self._configuration_llm.local_model,
                base_url=os.environ.get(
                    'FOSA_OLLAMA_URL', self._configuration_llm.local_base),
                temperature=self._configuration_llm.temperature,
                client_kwargs={'timeout': self._configuration_llm.timeout_sec},
            )
        else:
            from langchain_openai import ChatOpenAI

            token = (self._secrets.fosa_llm_api_key
                     or self._secrets.openai_api_key)
            if not token:
                raise LLMIndisponible(
                    'FOSA_LLM_API_KEY ou OPENAI_API_KEY absent.')
            modele = ChatOpenAI(
                model=self._configuration_llm.api_model,
                base_url=self._configuration_llm.api_base,
                api_key=token,
                temperature=self._configuration_llm.temperature,
                timeout=self._configuration_llm.timeout_sec,
                max_retries=0,
                use_responses_api=False,
            )

        invite = ChatPromptTemplate.from_messages([
            SystemMessage(content=self._construire_prompt_systeme()),
            MessagesPlaceholder(variable_name='historique'),
            ('human', '{texte}'),
        ])
        chaine = invite | modele.with_structured_output(
            SCHEMA_INTENT, method='json_schema')
        self._chaines[fournisseur] = chaine
        return chaine

    def _interpreter_avec(self, fournisseur, texte, historique):
        """Exécute la chaîne et valide à nouveau les champs reçus."""
        from langchain_core.messages import AIMessage, HumanMessage

        messages = []
        for tour in historique:
            if tour.get('role') == 'user':
                messages.append(HumanMessage(content=tour.get('content', '')))
            elif tour.get('role') == 'assistant':
                messages.append(AIMessage(content=tour.get('content', '')))
        resultat = self._creer_chaine(fournisseur).invoke({
            'historique': messages,
            'texte': texte,
        })
        if not isinstance(resultat, dict):
            raise ErreurIntention('LangChain n’a pas retourné un objet JSON.')
        return Intention.depuis_json(json.dumps(resultat, ensure_ascii=False))

    def _appeler_local(self, texte, historique):
        """Interroge le modèle Ollama local avec LangChain."""
        return self._interpreter_avec('local', texte, historique)

    def _appeler_api(self, texte, historique):
        """Interroge une API compatible OpenAI avec LangChain."""
        return self._interpreter_avec('api', texte, historique)

    def interpreter(self, texte, historique=()):
        """Interprète la phrase de l'utilisateur et retourne une Intention structurée.

        Essaie le fournisseur configuré (local ou API), puis bascule si nécessaire.
        """
        fournisseurs = {
            'local': [self._appeler_local],
            'api': [self._appeler_api],
            'auto': [self._appeler_local, self._appeler_api],
        }.get(self._configuration_llm.provider, [self._appeler_local])
        erreurs = []
        for fournisseur in fournisseurs:
            for _ in range(self._configuration_llm.max_retries + 1):
                try:
                    return fournisseur(texte, historique)
                except Exception as erreur:
                    erreurs.append(str(erreur))
        raise LLMIndisponible('; '.join(erreurs) or 'Fournisseur indisponible.')
