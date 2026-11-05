# caytu_config — configuration unique

Toute valeur modifiable de la solution Fosa vit dans ce paquet :

| Fichier | Contenu |
|---|---|
| `config/fosa.yaml` | système, navigation, perception, audio, dialogue, mission |
| `config/locations.yaml` | carte sémantique : noms, alias, tags, poses d'approche |
| `config/knowledge.yaml` | programme et FAQ du site |
| `config/prompts.yaml` | prompt système, intentions, règles, exemples |

## Règles

- Aucun nœud Python ne code une coordonnée, un seuil, un topic, un modèle ou
  une durée en dur : il lit `load_config()`.
- Les arguments ROS (`--ros-args -p`) servent seulement d'override de debug.
- Les secrets (clés API) sont lus dans l'environnement, jamais dans un YAML :
  `OPENAI_API_KEY`, `ELEVENLABS_API_KEY`, `FOSA_LLM_API_KEY`,
  `FOSA_STT_API_KEY`, `FOSA_TTS_API_KEY`.
- `FOSA_CONFIG_DIR` remplace le dossier complet.
- `FOSA_CONFIG_OVERRIDE` pointe un YAML fusionné par-dessus `fosa.yaml`.
- `FOSA_DEPLOYMENT` force `local`, `remote_pc` ou `simulation`.

## Utilisation

```python
from caytu_config import load_config

configuration = load_config()
print(configuration.navigation.time_limit_sec)
lieu = configuration.semantic_map.resolve("WC")
```

Le chargement échoue avec `ConfigurationError` et le chemin exact de la clé si
un fichier manque, si une clé est inconnue ou si une valeur est hors bornes.

## Tests

```bash
python3 -m pytest caytu_config/test -q
```
