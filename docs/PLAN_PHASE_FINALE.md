# Plan de combinaison et de migration — Phase finale PARC 2026

> Document de référence pour l'équipe et pour toute IA qui reprend le travail.
> Dernière mise à jour : 2026-11-02.
> Branche de travail : `integration/phase-finale`.
> Base : `refactor/franglais` (soumission v2 + refactor lisible) + `feat/navigation-benchmark`.

---

## 0. Décisions actées

1. **Base de code** : `refactor/franglais`, parce que c'est la version stable de
   `soumission-v2` avec un code lisible en français.
2. **Laboratoire navigation** : `feat/navigation-benchmark` est conservé. Il
   contient le travail de comparaison `profil + localisation + contrôleur` et
   permet de choisir les réglages par défaut et les fallbacks.
3. **Pas de port séparé de `origin/imu`** : le travail IMU/EKF de Liantsoa est
   déjà couvert par `feat/navigation-benchmark`.
4. **Pas de port de `feat/v4`, `feat/v2-ameliored`, `feat/apports-v3`** : ces
   branches ne font pas partie du périmètre retenu. On garde uniquement ce qui
   est déjà dans `feat/navigation-benchmark` et dans `refactor/franglais`.
5. **Assistant vocal** : le module `pole_nlp` est porté dans des paquets ROS 2.
   Il ne reste pas un script Python à part.
6. **Audio** : le robot officiel n'a pas encore de micro/haut-parleur dans
   l'URDF, mais il sera équipé. Le traitement peut tourner :
   - sur le robot,
   - ou sur un PC distant connecté au robot (ROS 2 sur réseau) ; dans ce cas
     seule l'exécution des mouvements reste sur le robot.
   Les deux modes doivent être configurables, sans changement de code.
7. **Configuration** : tous les réglages modifiables (durée, carte, taille des
   AprilTags, topics, modèles, seuils, vitesses, langues, devices audio, clés
   d'API via variables d'environnement, etc.) vivent dans **un seul endroit** :
   le paquet `caytu_config`. Aucune valeur métier en dur dans le code Python.
8. **Sécurité** : le LLM ne produit jamais de coordonnées, de vitesse ou de
   commande moteur. Il produit une intention et un nom de lieu. Le dispatcher
   valide, puis appelle Nav2. Aucun agent LLM n'appelle directement Nav2.

---

## 1. État actuel du dépôt

### Branches utiles

| Branche | Rôle |
|---|---|
| `pre-final` / tag `soumission-v2` | version réellement soumise |
| `refactor/franglais` | base lisible en français, tests, guide de modification |
| `feat/navigation-benchmark` | laboratoire profils/contrôleurs/localisations, benchmarks |
| `integration/phase-finale` | branche de combinaison (courante) |

### Ce que contient déjà `feat/navigation-benchmark`

- `caytu_nav_bringup/config/profiles/` : profils `balanced`, `fast`, `safe`,
  `tight_clearance`, `ultra_fast`, `ultra_safe`.
- `caytu_nav_bringup/config/controllers/` : `mppi.yaml`, plus les contrôleurs
  `rpp` et `dwb`.
- `caytu_nav_bringup/config/localization/` : `odom_imu.yaml`, `odom_only.yaml`,
  `amcl.yaml`, `ekf.yaml`.
- `caytu_nav_bringup/config/scenarios.yaml` : scénarios de benchmark.
- `caytu_nav_solution/benchmark_*.py`, `compare_cli.py` : outils de mesure.
- `caytu_nav_solution/test/test_navigation_lab.py` : tests du laboratoire.
- `docs/navigation-lab/` : documentation du laboratoire.
- `cmd.sh` : lancement Gazebo/benchmark.

### Conflits de fusion connus

Une première tentative de fusion `feat/navigation-benchmark` dans
`integration/phase-finale` a produit quatre conflits :

- `caytu_nav_bringup/launch/navigation.launch.py`
- `caytu_nav_bringup/launch/solution_bringup.launch.py`
- `caytu_nav_bringup/test/test_navigation_config.py`
- `caytu_nav_solution/caytu_nav_solution/task_solution.py`

Résolution attendue :

- garder la **lisibilité française** de `refactor/franglais` ;
- reprendre les **fonctionnalités** de `feat/navigation-benchmark` (profils,
  contrôleurs, localisations, benchmark) ;
- ne pas réintroduire de valeurs en dur : elles partiront dans `caytu_config` ;
- relancer les tests après résolution.

Les autres fichiers (profils, modules de benchmark, docs, `setup.py`,
`package.xml`) se fusionnent automatiquement.

### Artefacts à ne pas committer

- `results/` (résultats de benchmark),
- `log/`, `build/`, `install/`,
- `.pytest_cache/`, `__pycache__/`,
- fichiers audio de test, journaux du dispatcher,
- modèles lourds (Whisper, YOLO, Piper, Ollama) : ils se téléchargent ou se
  copient hors dépôt.

---

## 2. Architecture cible

### Vue d'ensemble

```
                        PC / robot
┌─────────────────────────────────────────────────────────────────────┐
│  caytu_assistant                                                    │
│  ┌──────────┐   ┌──────────────────┐   ┌─────────────────────────┐  │
│  │ STT      │──>│ Dialogue Manager │──>│ TTS                     │  │
│  │ VAD +    │   │ état, mémoire,   │   │ Piper local / API       │  │
│  │ Whisper  │   │ confirmation     │   └─────────────────────────┘  │
│  └──────────┘   └───────┬──────────┘                                │
│                         │ intention + lieu                          │
│                         ▼                                           │
│                 ┌──────────────────┐                                │
│                 │ Validateur       │                                │
│                 │ Pydantic + JSON  │                                │
│                 └────────┬─────────┘                                │
│                          ▼                                          │
│                 ┌──────────────────┐        ┌────────────────────┐  │
│                 │ Dispatcher       │───────>│ Nav2 action client │  │
│                 │ action déterministe       └─────────┬──────────┘  │
│                 └────────┬─────────┘                  │             │
│                          │                            ▼             │
│                          │                    robot / Gazebo        │
│                          ▼                                          │
│                 ┌──────────────────┐                                │
│                 │ Carte sémantique │<── AprilTags + lieux YAML       │
│                 └──────────────────┘                                │
│                                                                     │
│  caytu_perception                                                   │
│  caméras ──> YOLO / géométrie ──> personnes, objets, tags           │
└─────────────────────────────────────────────────────────────────────┘
```

### Paquets

| Paquet | Rôle | Dépend de |
|---|---|---|
| `caytu_config` | source unique de configuration + loader Pydantic | — |
| `caytu_interfaces` | messages et services ROS 2 typés | — |
| `caytu_semantic_map` | lieux, alias, tags, poses d'approche | `caytu_config`, `caytu_interfaces` |
| `caytu_perception` | AprilTags, YOLO, géométrie, suivi personne | `caytu_config`, `caytu_interfaces` |
| `caytu_assistant` | STT, LLM, dispatcher, TTS, machine à états | `caytu_config`, `caytu_interfaces`, `caytu_semantic_map` |
| `caytu_nav_solution` | navigation phase 1 + client Nav2 réutilisable | Nav2 |
| `caytu_nav_bringup` | capteurs, localisation, Nav2, launch | Nav2 |

### Principe de déploiement

Trois modes, tous configurables :

| Mode | Où tourne quoi | Usage |
|---|---|---|
| `local` | tout sur le robot | robot autonome |
| `remote_pc` | assistant + perception sur PC, navigation sur robot | PC plus puissant, micro/haut-parleur du PC |
| `simulation` | Gazebo + assistant sur PC | mise au point et jury simulation |

En mode `remote_pc` :
- ROS 2 DDS relie le PC et le robot ;
- le PC envoie les buts Nav2 et reçoit les détections ;
- le robot exécute les mouvements ;
- un watchdog arrête le robot si le lien tombe ;
- `ROS_DOMAIN_ID`, l'interface réseau et `use_sim_time` sont dans la config.

---

## 3. Configuration centralisée

### Emplacement unique

Paquet `caytu_config` :

```
caytu_config/
├── caytu_config/
│   ├── __init__.py
│   ├── loader.py          # charge, fusionne, valide
│   └── models.py          # schémas Pydantic
├── config/
│   ├── fosa.yaml          # tous les réglages du système
│   ├── locations.yaml     # carte sémantique (données)
│   ├── knowledge.yaml     # programme / FAQ
│   └── prompts.yaml       # prompt système, règles, exemples
├── package.xml
└── setup.py
```

### Règle

- **Aucune valeur métier en dur** dans les nœuds Python : ni coordonnée, ni
  durée, ni taille de tag, ni nom de modèle, ni topic, ni seuil.
- Les valeurs par défaut vivent dans `fosa.yaml`.
- Les arguments ROS (`--ros-args -p`) servent uniquement d'**override** de
  debug, jamais de source principale.
- Les secrets (clés API) vivent uniquement dans des variables
  d'environnement, jamais dans le YAML ni dans Git.
- Le loader échoue au démarrage avec un message clair si une clé obligatoire
  manque ou si une valeur est invalide.

### Exemple de `fosa.yaml`

```yaml
system:
  deployment: simulation          # local | remote_pc | simulation
  use_sim_time: true
  global_frame: map
  base_frame: base_footprint
  ros_domain_id: 42
  clock_topic: /clock

navigation:
  map_yaml: ""                    # vide = carte du bringup
  profile: balanced               # balanced | fast | safe | tight_clearance | ...
  controller: rpp                 # rpp | dwb | mppi
  localization: odom_imu          # odom_imu | odom_only | amcl
  fallback_profile: safe
  fallback_controller: dwb
  fallback_localization: odom_imu
  time_limit_sec: 600.0
  stop_margin_sec: 1.0
  success_radius: 0.20
  arrival_recheck_radius: 0.35
  retry_pause_sec: 1.0
  nav2_startup_timeout_sec: 180.0
  guiding_speed_mps: 0.30
  guiding_wait_sec: 15.0
  scan_topic: /scan
  cmd_vel_topic: /robot_base_controller/cmd_vel_unstamped

perception:
  top_color_topic: /top_camera_color/image_raw
  top_depth_topic: /top_camera_depth/points
  top_info_topic: /top_camera_color/camera_info
  bottom_color_topic: /bottom_camera_color/image_raw
  bottom_depth_topic: /bottom_camera_depth/points
  apriltag:
    family: 36h11
    size_m: 0.18
    max_distance_m: 4.0
  yolo:
    enabled: true
    model_path: models/yolo11n.onnx
    confidence: 0.45
    max_fps: 8.0
    device: cpu
  person:
    min_distance_m: 0.8
    max_distance_m: 2.5
    min_confidence: 0.50
    consecutive_frames: 3
    cooldown_sec: 25.0
    dynamic_speed_mps: 0.10

audio:
  input_device: default
  output_device: default
  sample_rate: 16000
  vad_threshold: 0.50
  silence_ms: 700

dialogue:
  languages: [fr, en]
  stt:
    provider: local               # local | api | auto
    local_model: small
    api_model: whisper-1
    timeout_sec: 8.0
  tts:
    provider: local               # local | api | auto
    local_voice: fr_FR-siwis-medium
    api_voice: ""
    timeout_sec: 5.0
  llm:
    provider: local               # local | api | auto
    local_model: qwen2.5:3b
    api_model: ""
    api_base: ""
    timeout_sec: 8.0
    max_retries: 2
    temperature: 0.2
  confirmation_required: true
  max_turns_memory: 5
  unknown_location_reply: "Je ne connais pas ce lieu, l'accueil pourra vous renseigner."

mission:
  greeting_enabled: true
  emergency_location: infirmerie
  default_wait_sec: 20.0
```

### `locations.yaml`

```yaml
world: stade_dakar
frame: map
locations:
  - name: accueil
    aliases: [reception, information, entree]
    tag_ids: [10]
    description: "au nord du stade, près de l'escalier"
    approach_pose: {x: 1.2, y: 3.4, yaw: 0.0}
    category: service
  - name: toilettes
    aliases: [wc, sanitaires, latrines]
    tag_ids: [11]
    description: "à droite de l'accueil, derrière la buvette"
    approach_pose: {x: 4.0, y: 2.0, yaw: 1.57}
    category: service
```

- `world` permet d'avoir plusieurs cartes sémantiques (café phase 1, stade
  phase 2).
- `tag_ids` peut contenir plusieurs tags pour un même lieu.
- `approach_pose` est la pose Nav2 d'approche, pas la pose exacte du tag.
- Le LLM ne voit que les `name` et `aliases` ; il ne voit jamais les
  coordonnées.

### `knowledge.yaml`

Programme, horaires, FAQ. Le LLM ne fait que reformuler les données présentes.
S'il ne trouve pas l'information, il répond qu'il ne la connaît pas et renvoie
vers l'accueil. Pas de RAG vectoriel au départ : un YAML + recherche par mots
clés est plus fiable et plus rapide pour un petit programme.

### `prompts.yaml`

Prompt système, intentions, règles, exemples. Les exemples de localisation
peuvent être générés depuis `locations.yaml` pour rester cohérents.

---

## 4. Interfaces ROS 2

### Messages

```
caytu_interfaces/msg/DetectedEntity.msg
  std_msgs/Header header
  uint32 track_id
  string class_name
  float32 confidence
  geometry_msgs/Point position       # base_footprint ou map
  float32 distance_m
  float32 bearing_rad
  geometry_msgs/Vector3 velocity
  bool is_person
  bool is_dynamic
  string source                      # yolo | geometry | fusion

caytu_interfaces/msg/DetectedEntityArray.msg
  std_msgs/Header header
  DetectedEntity[] entities

caytu_interfaces/msg/TagDetection.msg
  std_msgs/Header header
  int32 tag_id
  geometry_msgs/PoseStamped pose_map
  float32 distance_m
  float32 confidence

caytu_interfaces/msg/TagDetectionArray.msg
  std_msgs/Header header
  TagDetection[] detections

caytu_interfaces/msg/SemanticLocation.msg
  string name
  string[] aliases
  int32[] tag_ids
  string description
  geometry_msgs/PoseStamped approach_pose
  string category

caytu_interfaces/msg/DialogueState.msg
  std_msgs/Header header
  string state
  string detail
```

### Services

```
caytu_interfaces/srv/ResolveLocation.srv
  string query
  ---
  bool success
  SemanticLocation location
  string error

caytu_interfaces/srv/GetLocationByTag.srv
  int32 tag_id
  ---
  bool success
  SemanticLocation location
  string error
```

### Topics

| Topic | Type | Producteur | Consommateur |
|---|---|---|---|
| `/assistant/transcript` | `std_msgs/String` (ou message enrichi) | STT | dialogue |
| `/assistant/response` | `std_msgs/String` | dialogue | TTS |
| `/assistant/state` | `DialogueState` | dialogue | monitoring |
| `/perception/entities` | `DetectedEntityArray` | perception | dialogue, monitoring |
| `/perception/tags` | `TagDetectionArray` | perception | semantic map, dialogue |
| `/semantic_map/locations` | `SemanticLocation[]` (latched) | semantic map | RViz, debug |

### Actions

- `navigate_to_pose` (Nav2) : seul moyen de déplacer le robot.
- Pas d'action custom pour l'instant ; le dialogue manager utilise le client
  Nav2 extrait de `task_solution.py`.

---

## 5. Dialogue et LLM

### Flux

```
micro -> VAD -> STT -> texte
      -> LLM (intent + slots) -> Pydantic -> validateur
      -> dispatcher -> action ROS
      -> réponse texte -> TTS -> haut-parleur
```

### Schéma d'intention

```python
class Intent(BaseModel):
    intentions: list[Literal[
        "navigation", "localiser", "information", "confirmation",
        "salutation", "fin", "autre"
    ]]
    lieux: list[str]
    confirmation: Literal["oui", "non", ""]
    reponse: str
    langue: Literal["fr", "en", ""]
```

- `urgence` n'est pas laissé au LLM : des mots-clés (`mal`, `blessé`,
  `secours`, `tomber`) déclenchent une règle déterministe vers l'infirmerie.
- `lieux` ne contient que des noms/alias connus ; le validateur et le registre
  ramènent aux noms officiels.
- `reponse` est vide uniquement si l'intention est `navigation` seule.

### LangChain

LangChain **ne garantit pas** le JSON à lui seul. La chaîne est :

1. décodage contraint Ollama (`format=schema`) ;
2. validation Pydantic ;
3. retry si invalide ;
4. dispatcher déterministe ;
5. fallback rule-based si le LLM est indisponible.

LangChain apporte :
- l'abstraction local/API,
- les prompts,
- les tools,
- la mémoire de conversation,
- les retries.

### Tool calling

Tools en lecture seule :
- `resoudre_lieu(nom)` ;
- `infos_lieu(nom)` ;
- `chercher_programme(question)`.

Tools de proposition :
- `proposer_navigation(lieu)` ;
- `annuler_navigation()`.

L'exécution reste dans le dispatcher. Un agent LLM avec `max_iterations`
borné peut être utilisé pour la démo, mais le chemin par défaut est :
`intent structurée -> dispatcher`.

### Providers

| Service | Local | API | Mode |
|---|---|---|---|
| LLM | Ollama (`qwen2.5:3b` ou `qwen3:4b`) | OpenAI-compatible | `local`, `api`, `auto` |
| STT | `faster-whisper` (`small`/`base` int8) ou Whisper | API STT | `local`, `api`, `auto` |
| TTS | Piper (`fr_FR-siwis-medium`) | API TTS | `local`, `api`, `auto` |

- `auto` : essaie le provider préféré avec timeout, puis bascule.
- Clés API dans `OPENAI_API_KEY`, `ELEVENLABS_API_KEY`, etc. Jamais dans Git.
- Mode hors ligne obligatoire pour le jury : modèles pré-téléchargés, TTS
  local, LLM local, knowledge base locale.
- Cache de phrases fréquentes (`Bonjour`, `Je vous emmène...`) pour réduire la
  latence.

### Langues

- FR/EN pour la démo.
- Wolof : Whisper local ne le gère pas correctement et Piper n'a pas de voix
  wolof. Ne pas promettre une voix wolof. Si le visiteur parle wolof, répondre
  en français/anglais ou basculer sur un provider API si disponible.
- Détecter la langue et l'indiquer dans le transcript.

---

## 6. Perception

### AprilTags

- Détection : `apriltag_ros` si disponible pour Jazzy, sinon OpenCV
  `cv2.aruco` avec `DICT_APRILTAG_36h11`.
- Taille, famille, distance max : `fosa.yaml` (`perception.apriltag`).
- Sortie : `TagDetectionArray` avec `tag_id`, pose `map`, distance.
- `caytu_semantic_map` traduit `tag_id -> lieu -> approach_pose`.
- `tag_calibrator` : outil de calibration si les tags sont placés par les
  organisateurs et que leurs poses ne sont pas connues.
- Les tags servent d'ancrage sémantique. La correction de localisation
  (`map -> odom`) est optionnelle et ne doit pas casser le localisateur
  existant.

### Personne vs objet

```
RGB + depth + camera_info
  ├─ YOLO ONNX (yolov8n / yolo11n) -> classe "person" + bbox
  └─ géométrie profondeur -> cluster 3D (hauteur, largeur, distance)
        │
        ▼
  fusion -> DetectedEntity
```

- `is_person` : YOLO **ou** géométrie (vote), avec confiance.
- `is_dynamic` : déplacement du centroïde entre deux frames.
- `track_id` : suivi simple IoU/centroïde.
- Cadence 5–10 FPS, pas besoin de 30 FPS.
- Fallback géométrique obligatoire : si YOLO est absent/lent, le système
  continue.
- **Nav2 ne dépend pas de YOLO** : les costmaps continuent d'utiliser les
  nuages de points existants.

### Déclenchement du dialogue

- personne à 0,8–2,5 m,
- dans le champ avant,
- 3 frames consécutives,
- robot à l'arrêt ou en attente,
- cooldown 25 s,
- pas de déclenchement pendant que le robot parle.

---

## 7. Machine à états du dialogue

```
IDLE
  └─ personne détectée -> GREETING
GREETING
  └─ écoute -> LISTENING
LISTENING
  └─ transcript -> THINKING
THINKING
  ├─ information -> ANSWERING -> IDLE
  ├─ localiser -> ANSWERING (propose accompaniment) -> CONFIRMING
  ├─ navigation -> CONFIRMING
  └─ fin -> IDLE
CONFIRMING
  ├─ oui -> GUIDING
  └─ non -> IDLE
GUIDING
  ├─ arrivée -> ARRIVED -> IDLE
  ├─ stop -> IDLE
  └─ échec -> ERROR -> IDLE
EMERGENCY
  └─ navigation immédiate vers infirmerie
```

- L'état est explicite, pas dans la mémoire du LLM.
- Mémoire de session : dernière proposition de lieu, dernière confirmation,
  langue, nombre de tours.
- `SPEAKING` coupe le STT pour éviter l'auto-écoute.
- Timeouts : pas de parole 10 s -> IDLE ; LLM 8 s -> phrase d'attente ou
  fallback ; Nav2 échec -> excuse + IDLE.

### Guidage

- Vitesse réduite (`guiding_speed_mps`, 0,3 m/s par défaut).
- Le robot mène, s'arrête périodiquement, vérifie que la personne suit.
- Si la personne est perdue : attendre, dire « Je vous attends », puis
  reprendre ou annuler après `guiding_wait_sec`.
- À l'arrivée : annoncer le lieu, proposer autre chose, retour IDLE.

---

## 8. Migration et lots de travail

### Lot 0 — Branche et plan

- [x] Branche `integration/phase-finale` créée depuis `refactor/franglais`.
- [x] Modifications non commitées sauvegardées sur
  `feat/navigation-benchmark` (commit `af90b98`).
- [ ] Ce document `docs/PLAN_PHASE_FINALE.md` commité.
- [ ] `results/`, `log/`, caches ajoutés au `.gitignore` si nécessaire.

### Lot 1 — Fusion du laboratoire navigation

- [x] Fusionner `feat/navigation-benchmark` dans `integration/phase-finale`.
- [x] Résoudre les 4 conflits en gardant la lisibilité française + les
  fonctionnalités du lab.
- [x] Vérifier `setup.py`, `package.xml`, entry points.
- [x] `colcon build` + `colcon test` verts.
- [x] Scénario phase 1 officiel toujours réussi (but atteint en 39,9 s
  simulées, 0,13 m du centre, code 0).

### Lot 2 — `caytu_config`

- [x] Créer le paquet `caytu_config`.
- [x] Écrire `fosa.yaml`, `locations.yaml`, `knowledge.yaml`, `prompts.yaml`.
- [x] Loader typé (dataclasses, sans dépendance externe) + overrides env +
  validation stricte.
- [x] Tests : chargement, clé manquante, valeur invalide, override, secrets.
- [x] Brancher les launch files et `task_solution.py` sur le loader.
- [x] Audit initial : les valeurs métier de navigation/perception sont lues
  dans `fosa.yaml` (reste à faire pour les lots 3-8).

### Lot 3 — Extraction navigation

- [x] Extraire `BringupManager`, `Nav2Navigator`, `TaskTimer` de
  `task_solution.py`.
- [ ] `task_solution.py` conserve exactement le comportement phase 1
  (simulation officielle à rejouer après intégration).
- [x] Tests unitaires des classes extraites.
- [x] Le client Nav2 extrait est réutilisable depuis un autre nœud ROS 2.

**Essai de reprise (2026-11-02)** : le build et les 70 tests du paquet passent.
La simulation officielle a été lancée, mais son facteur temps réel était
d'environ 0,07 ; à 14 s simulées, Nav2 a expiré deux attentes de
`compute_path_to_pose` (statut d'action 6) et la navigation n'était pas
terminée. Le test a été interrompu proprement ; la non-régression phase 1 reste
à confirmer sur un simulateur assez rapide.

### Lot 4 — Interfaces + carte sémantique

- [x] Créer `caytu_interfaces`.
- [x] Créer `caytu_semantic_map` + services.
- [x] Résolution nom/alias/tag.
- [ ] Tests sans ROS (registre) + test ROS du service.

### Lot 5 — Cœur assistant

- [x] Porter la validation d'intention, le registre et le dispatcher sûr.
- [ ] Porter les tests `pole_nlp`.
- [x] Créer `llm_client.py` (Ollama + API compatible OpenAI).
- [x] Dispatcher : navigation, localisation, information, confirmation,
  urgence, annulation et fin.
- [x] Fallback rule-based si LLM indisponible.

Note : le client utilise directement les API HTTP structurées; LangChain n'est
pas nécessaire au chemin d'exécution actuel. Les scénarios restent à valider.

### Lot 6 — Voix

- [x] Nœud STT (VAD + Whisper/faster-whisper local ou API).
- [x] Nœud TTS (Piper ou API; sans cache de synthèse).
- [x] Mode `text` et mode `audio_file`.
- [x] Configuration des périphériques audio.
- [ ] Tests : transcript simulé, TTS indisponible, mode hors ligne.

### Lot 7 — Perception

- [x] `tag_detector` + `tag_calibrator`.
- [x] `obstacle_classifier` (YOLO facultatif + géométrie).
- [x] `person_tracker` + événements configurables.
- [ ] Tests : tag synthétique, personne synthétique, pas de YOLO.
- [ ] Vérifier que Nav2 n'est pas impacté.

### Lot 8 — Mission et intégration

- [x] `dialogue_manager` + états de dialogue.
- [x] `assistant_solution.py` + `assistant.launch.py`.
- [x] Guidage Nav2, confirmation, urgence et stop au niveau code.
- [ ] Scénarios de test dans Gazebo.
- [ ] Mode `remote_pc` testé.

Le watchdog réseau indiqué dans l'architecture cible n'est pas implémenté;
le mode `remote_pc` n'est donc pas prêt pour une validation sur robot.

### Lot 9 — Durcissement

- [ ] Latence mesurée (STT, LLM, TTS, bout en bout).
- [ ] Mode hors ligne complet.
- [x] Guide de validation manuelle : `docs/GUIDE_TESTS_PHASE_FINALE.md`.
- [x] Contexte persistant pour les IA suivantes : `AGENTS.md` et
  `docs/REPRISE_IA_PHASE_FINALE.md`.
- [ ] Docs complémentaires : `GUIDE_CONFIG.md`, `GUIDE_ASSISTANT.md`, README.
- [ ] Vidéo de démo.
- [ ] Zip de soumission sans `build/`, `install/`, `log/`, `results/`,
  modèles lourds, clés.

---

## 9. Scénarios d'acceptation

### Phase 1 — non-régression

1. `ros2 launch parc_robot_bringup task.launch.py`
2. `ros2 run caytu_nav_solution task_solution.py`
3. Le robot atteint le but officiel dans la limite de temps, sans collision.
4. Les tests `colcon test` sont verts.

### Phase 2 — dialogue

| Scénario | Attendu |
|---|---|
| « Où sont les toilettes ? » | localiser + proposer l'accompagnement |
| « Oui » | navigation vers les toilettes |
| « Emmène-moi à la buvette » | confirmation puis navigation |
| « À quelle heure est le match ? » | information depuis `knowledge.yaml` |
| « J'ai mal à la tête » | navigation immédiate vers l'infirmerie |
| « Stop » | annulation Nav2 + arrêt |
| « Où est la piscine ? » | lieu inconnu, pas de navigation, réponse accueil |
| Personne devant le robot | salutation, écoute, pas de répétition toutes les 2 s |
| API LLM coupée | fallback local ou rule-based |
| TTS coupé | réponse texte dans les logs, pas de blocage |
| Rien détecté | le robot reste en IDLE, ne parle pas aux chaises |

### Phase 2 — tags

1. Le robot détecte le tag 11 et le résout en `toilettes`.
2. La pose d'approche est celle de `locations.yaml`.
3. Si le tag est inconnu, aucun déplacement n'est déclenché.
4. Si le tag est détecté mais la carte sémantique indisponible, fallback nom.

### Phase 2 — réseau

1. Mode `remote_pc` : le PC envoie un but, le robot l'exécute.
2. Coupure réseau : le robot s'arrête ou annule le but, pas de mouvement
   fantôme.
3. `use_sim_time` correct des deux côtés.

---

## 10. Risques et parades

| Risque | Parade |
|---|---|
| YOLO ne détecte pas les visiteurs Gazebo | fallback géométrique + test synthétique |
| LLM local trop lent | modèle 1.7B–3B quantifié, cache, TTS par phrase, API en option |
| LLM hallucine un lieu | registre + validation stricte + dispatcher |
| JSON invalide | `format=schema` + Pydantic + retry + fallback |
| Tool calling instable | tools en lecture seule, dispatcher déterministe |
| Wolof non supporté | FR/EN pour la démo, fallback explicite |
| Pas de micro dans Gazebo | mode texte + audio hôte |
| Tags sans pose connue | calibration + `tag_map.yaml` |
| Réseau PC/robot instable | watchdog, arrêt sûr, mode local |
| Clé API dans le dépôt | variables d'environnement + `.gitignore` |
| Régression navigation | tests phase 1 après chaque lot |
| Configuration éparpillée | `caytu_config` unique + audit |

---

## 11. Conventions

- Commits en français : `feat:`, `fix:`, `refactor:`, `docs:`, `test:`,
  `chore:`.
- Code métier en français, termes techniques en anglais (comme
  `refactor/franglais`).
- Voir `AGENTS.md` pour les règles détaillées de nommage en français et la
  règle de configuration centralisée; toute IA qui reprend le dépôt doit le
  lire avant de coder.
- Un lot = un commit cohérent, tests verts avant commit.
- Ne jamais committer `results/`, `log/`, `build/`, `install/`, caches,
  modèles lourds, clés.
- Documentation obligatoire pour chaque nouveau paquet.

---

## 12. Commandes utiles

```bash
# ROS 2
source /opt/ros/jazzy/setup.bash

# Build ciblé
colcon build --symlink-install --packages-select caytu_config caytu_interfaces

# Tests
colcon test --packages-select caytu_config caytu_interfaces
colcon test-result --verbose

# Phase 1
ros2 launch parc_robot_bringup task.launch.py
ros2 run caytu_nav_solution task_solution.py

# Phase 2 (après lots 4-8)
ros2 run caytu_assistant assistant_solution.py

# Mode texte (sans micro)
ros2 run caytu_assistant dialogue_manager --ros-args -p input_mode:=text
```

---

## 13. Reprise par une autre IA

Ordre de lecture :

1. ce document ;
2. `AGENTS.md` et `docs/REPRISE_IA_PHASE_FINALE.md` ;
3. `docs/GUIDE_MODIFICATION.md` (refactor/franglais) ;
4. `docs/navigation-lab/ARCHITECTURE.md` (benchmark) ;
5. `caytu_config/config/fosa.yaml` ;
6. `docs/GUIDE_TESTS_PHASE_FINALE.md` pour les scénarios manuels.
7. Dépôt NLP d'origine : [Faneva01/pole_nlp](https://github.com/Faneva01/pole_nlp)
   pour les anciens validateurs, règles, actions et prompts.

Règles de reprise :

- ne pas repartir de `main` ;
- ne pas réintroduire `feat/v4`, `v2-ameliored`, `apports-v3` ;
- ne pas dupliquer `lieux.yaml` ;
- ne pas mettre de coordonnées ou de seuils dans le code ;
- ne pas faire appeler Nav2 directement par le LLM ;
- vérifier la phase 1 après chaque lot.
- ne pas lancer Gazebo ni annoncer une acceptation manuelle sans confirmation
  explicite de l'équipe.

État des lots : voir la section 8. Cocher les cases au fur et à mesure.
