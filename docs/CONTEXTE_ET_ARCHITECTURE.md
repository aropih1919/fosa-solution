# Solution Fosa — Contexte final & Architecture

Ce document synthétise l'architecture complète, les choix techniques et les règles de configuration de la solution développée par l'équipe Fosa pour la compétition PARC 2026 (Engineers League).

---

## 1. Vue d'ensemble du projet

La solution transforme le robot de service CAYTU Sito-É en un assistant autonome intelligent et sécurisé pour les visiteurs d'un stade :
- **Phase 1 — Navigation autonome** : Déplacement fiable et réactif en milieu encombré (tables, chaises, obstacles dynamiques), sans cartographie préalable requise lors du déploiement, en s'appuyant sur Nav2 et les capteurs de distance (LiDAR et caméras de profondeur).
- **Phase 2 — Assistant multimodal & Accueil** : Détection des personnes, salutation proactive, dialogue vocal fluide (STT Whisper + LLM contraint + TTS Piper), carte sémantique ancrée par balises AprilTag, et guidage physique déterministe vers les stands et services.

---

## 2. Les 7 paquets ROS 2

Le projet est découpé en 7 paquets modulaires et maintenables :

| Paquet | Description & Rôle | Dépendances principales |
|---|---|---|
| `caytu_config` | **Source unique de vérité** : tous les réglages modifiables (topics, vitesses, seuils, modèles, lieux, FAQ) vivent dans 4 fichiers YAML validés au chargement par des schémas stricts. | Python YAML |
| `caytu_interfaces` | **Contrats d'échange** : définitions des messages et services ROS 2 personnalisés (`SemanticLocation`, `DetectedEntity`, `TagDetection`, `DialogueState`, etc.). | `geometry_msgs`, `std_msgs` |
| `caytu_semantic_map` | **Carte des lieux** : résolution des noms de stands, alias et balises AprilTag, calcul de la pose d'approche devant chaque point d'intérêt. | `caytu_config`, `caytu_interfaces` |
| `caytu_perception` | **Vision & Obstacles** : détection des balises AprilTag (OpenCV), classification 3D des obstacles (personnes vs meubles) via nuages de points caméras, et suivi de visiteurs. | `caytu_config`, OpenCV, TF2 |
| `caytu_assistant` | **Intelligence & Voix** : reconnaissance vocale (Whisper), compréhension contrainte (Ollama local ou API compatible OpenAI), dispatcher d'actions déterministe et synthèse vocale (Piper/API). | `caytu_config`, `caytu_interfaces`, Nav2 |
| `caytu_nav_solution` | **Exécution navigation** : point d'entrée officiel Phase 1 (`task_solution.py`) et briques réutilisables (`Nav2Navigator`, `BringupManager`, `TaskTimer`). | Nav2, TF2 |
| `caytu_nav_bringup` | **Matériel & Nav2** : filtres des capteurs (châssis LiDAR, caméras), profils de navigation et fichiers de launch. | Nav2, laser_filters |

---

## 3. Règle fondamentale de sûreté

Le modèle de langage (LLM) ne dispose **d'aucun accès direct** aux commandes moteurs du robot :
1. Le LLM analyse la phrase de l'utilisateur et extrait uniquement une **intention** (`navigation`, `localiser`, `information`, `confirmation`, etc.) et un **nom de lieu**.
2. Le **dispatcher déterministe** valide le lieu dans la carte sémantique (`caytu_semantic_map`).
3. Toute situation d'urgence (« j'ai mal », « secours », « blessé ») court-circuite le LLM et déclenche la navigation immédiate vers le poste de secours.
4. L'ordre de mouvement est transmis à Nav2 via un `ActionClient` sécurisé avec arrêt d'urgence configurable.

---

## 4. Configuration centralisée (`caytu_config`)

Aucune valeur métier (coordonnée, seuil, durée, topic, modèle) n'est codée en dur dans les scripts Python. Tout est configurable dans le dossier `caytu_config/config/` :

- **`fosa.yaml`** : paramètres système, profils de navigation, réglages audio (périphériques, seuils VAD), fournisseurs LLM/STT/TTS et seuils de perception.
- **`locations.yaml`** : liste des lieux du site avec leurs noms officiels, leurs synonymes/alias, leurs identifiants AprilTag et leur distance d'approche.
- **`knowledge.yaml`** : programme officiel de la journée et foire aux questions (FAQ).
- **`prompts.yaml`** : prompt système du modèle, consignes de réponse, mots-clés de secours et textes de confirmation.

### Gestion des secrets (clés d'API)
Les clés d'API ne sont **jamais écrites dans les fichiers YAML ni versionnées dans Git**. Elles sont lues depuis l'environnement système :
```bash
export OPENAI_API_KEY="votre-clé"          # Fournisseur LLM / Whisper / TTS API
export FOSA_LLM_API_KEY="votre-clé"        # Clé spécifique LLM si distincte
export FOSA_STT_API_KEY="votre-clé"        # Clé spécifique STT
export FOSA_TTS_API_KEY="votre-clé"        # Clé spécifique TTS
export ELEVENLABS_API_KEY="votre-clé"      # TTS ElevenLabs optionnel
export FOSA_OLLAMA_URL="http://127.0.0.1:11434" # URL Ollama local
```

---

## 5. Style de code & Convention « Franglais »

Le code est structuré pour être accessible à tout développeur :
- Les noms de variables métier, méthodes, classes et commentaires sont écrits en français avec des verbes clairs (`_recevoir_lieux`, `_resoudre_lieu`, `_detecter_marqueurs`, `_recevoir_entites`, `_parler_local`, `_transcrire_audio`, `interpreter`).
- Les termes consacrés par ROS 2, Python et les protocoles restent en anglais (`Node`, `Pose2D`, `Topic`, `QoSProfile`, `callback`, types de messages).
- Les classes et méthodes disposent d'alias systématiques garantissant une compatibilité totale sans régression de performance.
