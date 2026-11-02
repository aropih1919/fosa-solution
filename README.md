# ÉQUIPE FOSA — PARC Engineers League 2026

Solution globale pour le robot d'accueil et de guidage CAYTU Sito-É combinant navigation autonome robuste sous Nav2, perception multimodale (LiDAR, caméras de profondeur, balises AprilTag) et assistant vocal intelligent avec validation déterministe.

**Pays :** Madagascar
**Équipe FOSA**

---

## 1. Architecture des 7 paquets

Le projet est conçu de façon modulaire et lisible :

| Paquet | Description & Rôle |
|---|---|
| [`caytu_config`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_config) | **Source unique de configuration** : tous les seuils, vitesses, topics, modèles, lieux et FAQ sont centralisés dans 4 fichiers YAML validés par schémas stricts. |
| [`caytu_interfaces`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_interfaces) | **Messages et services ROS 2** : contrats d'échange typés (`SemanticLocation`, `DetectedEntity`, `TagDetection`, `DialogueState`, etc.). |
| [`caytu_semantic_map`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_semantic_map) | **Carte sémantique du site** : résolution des noms de stands, alias et tags AprilTag avec calcul de la pose d'approche devant chaque point d'intérêt. |
| [`caytu_perception`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_perception) | **Vision et détection d'obstacles** : détection de balises AprilTag, classification 3D par nuages de points (personnes vs mobilier) et suivi de visiteurs. |
| [`caytu_assistant`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_assistant) | **Intelligence et interaction vocale** : reconnaissance vocale (Whisper local/API), compréhension contrainte avec LangChain (Ollama local ou API compatible OpenAI), dispatcher d'actions déterministe et synthèse vocale (Piper/API). |
| [`caytu_nav_solution`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_nav_solution) | **Solution de navigation** : point d'entrée officiel Phase 1 (`task_solution.py`) et client d'action `Nav2Navigator` réutilisable. |
| [`caytu_nav_bringup`](file:///home/johny-olivier/ros2_ws/src/fosa-solution/caytu_nav_bringup) | **Bringup matériel et Nav2** : filtrage des capteurs (masquage châssis LiDAR, caméras), profils de contrôleurs (RPP, DWB) et localisation odom+IMU. |

> [!IMPORTANT]
> **Règle de sécurité absolue** : Le modèle de langage (LLM) ne génère **jamais** de coordonnées spatiales ni de vitesse moteur. Il extrait une intention et un nom de lieu. Le dispatcher déterministe valide le lieu et pilote Nav2 en toute sécurité.

---

## 2. Guide des commandes — Que lancer et comment ?

### 2.1 Installation sur un nouveau PC
```bash
cd ~/ros2_ws/src/fosa-solution
./install.sh
```
L'installateur prépare Ubuntu 24.04, ROS 2 Jazzy, le dépôt de simulation PARC,
les dépendances Python et tous les paquets du workspace. Il demande si les
modèles IA locaux (YOLO, LLM, Whisper et voix Piper) doivent être téléchargés
immédiatement. Pour les récupérer plus tard : `./install.sh --models`.

Pour partager les modèles lourds sans les retélécharger :

Sur le premier poste, créez `modeles-fosa.tar.gz` :
```bash
./exporter_modeles.sh exporter
```

Sur le poste destinataire, lancez d'abord `./install.sh --no-models`, copiez
`modeles-fosa.tar.gz` à la racine du projet, puis importez les modèles :
```bash
./exporter_modeles.sh importer
```
L'archive contient les modèles YOLO/Piper, Whisper et tout le dépôt de modèles Ollama.
Les modèles YOLO/Piper sont installés dans `caytu_config/config/models/` du
projet. Whisper est placé dans le cache Hugging Face de l’utilisateur et Ollama
dans son répertoire de modèles habituel. `run.sh` charge la configuration du
dépôt afin que les modèles restent disponibles après une reconstruction.

`run.sh` source automatiquement ROS, le workspace compilé et l'environnement Python.

---

### 2.2 Solution complète — Simulation et assistant
```bash
./run.sh complet
```
Par défaut, l'assistant utilise le mode microphone configuré. Pour saisir le
texte au clavier :
```bash
./run.sh complet input_mode:=text
```

### 2.3 Navigation autonome officielle
La simulation doit d'abord être lancée dans un terminal :

**Terminal 1 — Lancement de la simulation officielle :**
```bash
./run.sh gazebo
```

**Terminal 2 — Lancement de la solution Fosa :**
```bash
./run.sh navigation
```

Ce mode lance `task_solution.py` avec le moniteur actif et enregistre un rapport
par navigation dans `results/`. Le contrôleur, le profil et la localisation
proviennent de `caytu_config/config/fosa.yaml`. Les sorties détaillées restent
dans chaque dossier de rapport; les erreurs et le suivi du robot restent visibles.

---

### 2.4 Assistant seul et test TTS

**Mode interactif en console (Idéal pour tester sans microphone) :**
```bash
./run.sh assistant input_mode:=text
```
*Tapez directement vos phrases dans le terminal (`Où sont les toilettes ?`, `Oui`, `Stop`, `À quelle heure est la finale ?`, `J'ai mal`).*

**Mode vocal complet (avec microphone et haut-parleur) :**
```bash
./run.sh assistant input_mode:=microphone
```

**Essai direct de la synthèse vocale :**
```bash
./run.sh tts "Bonjour, ceci est un essai de synthèse vocale."
```

---

### 2.5 Outil de diagnostic en direct
Permet de mesurer en temps réel la dérive de localisation, les faux obstacles et les contacts :
```bash
./run.sh diagnostic --ros-args -p use_sim_time:=true
```

---

Les outils du laboratoire restent accessibles avec `./run.sh benchmark` et
`./run.sh compare`.

### 2.6 Lancement des tests unitaires
Vérifie la robustesse de tous les modules sans nécessiter de simulateur :
```bash
colcon test --packages-select caytu_config caytu_nav_bringup caytu_nav_solution caytu_semantic_map caytu_assistant caytu_perception
colcon test-result --verbose
```

---

## 3. Documentation de référence

Pour approfondir, deux guides complets sont à votre disposition :
1. **[Contexte & Architecture détaillée](file:///home/johny-olivier/ros2_ws/src/fosa-solution/docs/CONTEXTE_ET_ARCHITECTURE.md)** : description approfondie des 7 paquets, de la configuration centralisée (`fosa.yaml`) et des conventions de code `refactor/franglais`.
2. **[Guide des tests pas-à-pas](file:///home/johny-olivier/ros2_ws/src/fosa-solution/docs/GUIDE_TESTS.md)** : scénarios complets de validation manuelle pour la Phase 1 et la Phase 2.
