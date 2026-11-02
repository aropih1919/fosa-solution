"""Client de chat structuré Ollama / API compatible OpenAI, sans tool moteur."""

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
    def __init__(self, configuration):
        self._config = configuration.dialogue.llm
        self._secrets = configuration.secrets
        self._prompts = configuration.prompts
        self._locations = configuration.semantic_map.locations

    def _system_prompt(self):
        lieux = [
            {'name': item.name, 'aliases': list(item.aliases)}
            for item in self._locations
        ]
        return (self._prompts.system + '\n\n'
                + '\n'.join(self._prompts.rules)
                + '\nLieux autorisés (noms et alias seulement): '
                + json.dumps(lieux, ensure_ascii=False))

    def _local(self, text, history):
        base = os.environ.get('FOSA_OLLAMA_URL', self._config.local_base)
        url = base.rstrip('/') + '/api/chat'
        messages = [{'role': 'system', 'content': self._system_prompt()}]
        messages.extend(history)
        messages.append({'role': 'user', 'content': text})
        body = {
            'model': self._config.local_model,
            'messages': messages,
            'format': SCHEMA_INTENT,
            'stream': False,
            'options': {'temperature': self._config.temperature},
        }
        response = self._post_json(url, body, None)
        content = response.get('message', {}).get('content', '')
        return Intention.depuis_json(content)

    def _api(self, text, history):
        token = (self._secrets.fosa_llm_api_key
                 or self._secrets.openai_api_key)
        if not token:
            raise LLMIndisponible('FOSA_LLM_API_KEY ou OPENAI_API_KEY absent.')
        base = self._config.api_base
        url = base.rstrip('/') + '/chat/completions'
        messages = [{'role': 'system', 'content': self._system_prompt()}]
        messages.extend(history)
        messages.append({'role': 'user', 'content': text})
        body = {
            'model': self._config.api_model,
            'messages': messages,
            'temperature': self._config.temperature,
            'response_format': {
                'type': 'json_schema',
                'json_schema': {'name': 'intent', 'strict': True,
                                'schema': SCHEMA_INTENT},
            },
        }
        response = self._post_json(url, body, token)
        content = response['choices'][0]['message']['content']
        return Intention.depuis_json(content)

    def _post_json(self, url, body, token):
        headers = {'Content-Type': 'application/json'}
        if token:
            headers['Authorization'] = f'Bearer {token}'
        request = urllib.request.Request(
            url, data=json.dumps(body).encode('utf-8'), headers=headers)
        try:
            with urllib.request.urlopen(
                    request, timeout=self._config.timeout_sec) as response:
                return json.loads(response.read().decode('utf-8'))
        except (OSError, urllib.error.URLError, TimeoutError,
                json.JSONDecodeError, KeyError) as error:
            raise LLMIndisponible(str(error)) from error

    def interpret(self, text, history=()):
        """Tente le fournisseur configuré puis l’autre en mode `auto`."""
        providers = {
            'local': [self._local],
            'api': [self._api],
            'auto': [self._local, self._api],
        }.get(self._config.provider, [self._local])
        erreurs = []
        for provider in providers:
            for _ in range(self._config.max_retries + 1):
                try:
                    return provider(text, history)
                except (LLMIndisponible, ErreurIntention) as error:
                    erreurs.append(str(error))
                    break
        raise LLMIndisponible('; '.join(erreurs) or 'Fournisseur indisponible.')
