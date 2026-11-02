# Guide des tests — Procédure manuelle & validation

Ce guide décrit l'ensemble des étapes et commandes à exécuter pour valider la solution Fosa en simulation Gazebo ou sur le matériel réel.

---

## 1. Préparation de l'environnement

### 1.1 Source des espaces de travail
Dans chaque terminal :
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 1.2 Modèles de langage et voix (si test local)
Si vous utilisez les fournisseurs locaux configurés par défaut :
```bash
# Vérifier qu'Ollama tourne et télécharger le modèle recommandé :
ollama pull qwen2.5:3b

# Vérifier que le lecteur audio et Piper sont présents (si test avec haut-parleur) :
which aplay piper
```

---

## 2. Test Phase 1 — Navigation autonome officielle

Ce test vérifie que le robot atteint le but officiel dans le restaurant du stade sans intervention :

**Terminal 1 — Lancement de la simulation officielle :**
```bash
ros2 launch parc_robot_bringup task.launch.py
```

**Terminal 2 — Lancement de la solution de navigation Fosa :**
```bash
ros2 run caytu_nav_solution task_solution.py
```

**Critères de succès :**
1. Le bringup Nav2 démarre automatiquement sans blocage.
2. Le robot se déplace de son point de départ jusqu'au cercle vert.
3. Aucune collision avec les obstacles (tables, chaises, murs).
4. Le nœud s'arrête proprement avec le code de sortie 0 une fois le but atteint.

---

## 3. Test Phase 2 — Assistant vocal & Dialogue interactif

### 3.1 Test en mode texte interactif (Recommandé en premier — sans micro)
Permet de valider immédiatement toute la chaîne de décision, la perception et Nav2 sans dépendre d'un micro :

```bash
ros2 launch caytu_assistant assistant.launch.py input_mode:=text
```

Dans l'invite `Vous>`, saisissez successivement les scénarios suivants pour valider le comportement :

| Entrée utilisateur | Comportement attendu du robot |
|---|---|
| `Bonjour` | L'assistant salue cordialement le visiteur. |
| `Où sont les toilettes ?` | L'assistant décrit l'emplacement (`locations.yaml`) et propose d'accompagner le visiteur. |
| `Oui` | L'assistant confirme et déclenche le déplacement Nav2 vers la pose d'approche des toilettes. |
| `Stop` | Annulation immédiate de la trajectoire Nav2 et arrêt du robot en sécurité. |
| `À quelle heure est la finale ?` | Réponse formulée à partir du programme de la journée (`knowledge.yaml`). |
| `J'ai mal / Urgence !` | Détection d'urgence immédiate $\rightarrow$ départ direct vers le poste de secours (`infirmerie`). |
| `Où est la piscine ?` | Lieu inconnu $\rightarrow$ aucun déplacement, invitation polie à s'adresser à l'accueil. |

### 3.2 Test avec microphone et haut-parleur réels
```bash
ros2 launch caytu_assistant assistant.launch.py input_mode:=microphone
```
Parlez distinctement devant le micro. Le VAD détecte la parole, Whisper transcrit la phrase, le LLM extrait l'intention, le dispatcher planifie l'action et Piper répond vocalement.

---

## 4. Outil de diagnostic en direct (`diag_nav.py`)

Pendant que la navigation ou l'assistant tourne, vous pouvez observer les grandeurs critiques du robot en temps réel dans un autre terminal :

```bash
python3 tools/diag_nav.py --ros-args -p use_sim_time:=true
```

Colonnes affichées toutes les 2 secondes :
- `ecart` : dérive entre la localisation estimée et la pose réelle dans Gazebo (doit rester faible, $< 0.15\text{ m}$).
- `top` : nombre d'obstacles vus par la caméra haute (doit être proche de 0 sur sol plat dégagé).
- `sol_lidar` : nombre de rayons du LiDAR supprimés car ils intersectent le sol.
- `contacts` : nombre de collisions physiques détectées (doit impérativement rester à 0).

---

## 5. Exécution des tests unitaires automatiques

Pour vérifier l'intégrité de tous les modules sans simulateur :

```bash
# Test de tous les paquets de la solution :
colcon test --packages-select caytu_config caytu_nav_bringup caytu_nav_solution caytu_semantic_map caytu_assistant caytu_perception

# Affichage des résultats détaillés :
colcon test-result --verbose
```
