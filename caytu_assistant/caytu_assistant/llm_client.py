"""Client de chat structuré pour Ollama et les API compatibles OpenAI, sans commande moteur."""

import json
import os
import urllib.error
import urllib.request

from caytu_assistant.models import ErreurIntention, Intention


SCHEMA_INTENT = {
    'type': 'object',
    'properties': {
        'intentions': {
            'type': 'array', 'items': {'type': 'string'}, 'minItems': 1,
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

    def _appeler_local(self, texte, historique):
        """Interroge le serveur Ollama local via son API HTTP /api/chat."""
        adresse_base = os.environ.get(
            'FOSA_OLLAMA_URL', self._configuration_llm.local_base)
        url = adresse_base.rstrip('/') + '/api/chat'
        messages = [{'role': 'system', 'content': self._construire_prompt_systeme()}]
        messages.extend(historique)
        messages.append({'role': 'user', 'content': texte})
        corps_json = {
            'model': self._configuration_llm.local_model,
            'messages': messages,
            'format': SCHEMA_INTENT,
            'stream': False,
            'options': {'temperature': self._configuration_llm.temperature},
        }
        reponse = self._envoyer_requete_json(url, corps_json, None)
        contenu = reponse.get('message', {}).get('content', '')
        return Intention.depuis_json(contenu)

    def _appeler_api(self, texte, historique):
        """Interroge une API compatible OpenAI (ChatGPT, Groq, etc.)."""
        token = (self._secrets.fosa_llm_api_key
                 or self._secrets.openai_api_key)
        if not token:
            raise LLMIndisponible('FOSA_LLM_API_KEY ou OPENAI_API_KEY absent.')
        adresse_base = self._configuration_llm.api_base
        url = adresse_base.rstrip('/') + '/chat/completions'
        messages = [{'role': 'system', 'content': self._construire_prompt_systeme()}]
        messages.extend(historique)
        messages.append({'role': 'user', 'content': texte})
        corps_json = {
            'model': self._configuration_llm.api_model,
            'messages': messages,
            'temperature': self._configuration_llm.temperature,
            'response_format': {
                'type': 'json_schema',
                'json_schema': {'name': 'intent', 'strict': True,
                                'schema': SCHEMA_INTENT},
            },
        }
        reponse = self._envoyer_requete_json(url, corps_json, token)
        contenu = reponse['choices'][0]['message']['content']
        return Intention.depuis_json(contenu)

    def _envoyer_requete_json(self, url, corps_json, token):
        """Effectue une requête POST HTTP avec sérialisation/désérialisation JSON."""
        entetes = {'Content-Type': 'application/json'}
        if token:
            entetes['Authorization'] = f'Bearer {token}'
        requete = urllib.request.Request(
            url, data=json.dumps(corps_json).encode('utf-8'), headers=entetes)
        try:
            with urllib.request.urlopen(
                    requete, timeout=self._configuration_llm.timeout_sec) as reponse:
                return json.loads(reponse.read().decode('utf-8'))
        except (OSError, urllib.error.URLError, TimeoutError,
                json.JSONDecodeError, KeyError) as erreur:
            raise LLMIndisponible(str(erreur)) from erreur

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
                except (LLMIndisponible, ErreurIntention) as erreur:
                    erreurs.append(str(erreur))
                    break
        raise LLMIndisponible('; '.join(erreurs) or 'Fournisseur indisponible.')
