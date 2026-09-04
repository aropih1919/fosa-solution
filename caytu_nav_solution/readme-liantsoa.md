## Vue d'ensemble : 3 fichiers, 3 responsabilités

```
task_solution.py   → CHEF D'ORCHESTRE (le node ROS2 lui-même)
goal_locator.py    → "Où dois-je aller ?"
nav2_client.py     → "Comment j'y vais et comment je sais si j'ai réussi ?"
```

`task_solution.py` ne fait aucune navigation lui-même — il **délègue** aux deux autres classes et se contente de coordonner le timing et les transitions d'état.

---

## Le flux complet, étape par étape

### 1. Démarrage du node

`TaskSolution.__init__()` fait 3 choses en parallèle dès le lancement :
- s'abonne au topic `/localization_ready` (ce que Trinôme 1 doit publier)
- instancie `GoalLocator` et `Nav2Client` (mais ne les utilise pas encore)
- démarre un **timer de fallback** de 5 secondes

Puis `rclpy.spin(node)` — le node attend, événement par événement, sans rien bloquer.

### 2. La double condition de démarrage (le point clé du J1/J2)

Il y a **deux chemins possibles** pour déclencher `_start_navigation()`, et un seul doit gagner :

```
        ┌─ /localization_ready reçu (True) ──► _start_navigation()
        │
Timer 5s┤
        └─ rien reçu après 5s + test_mode=True ──► _start_navigation() (mode test)
```

Le garde-fou `self._navigation_started` empêche que les deux chemins déclenchent la navigation deux fois si jamais le signal réel arrive juste après le fallback.

**Pourquoi ce design ?** Le plan impose de pouvoir travailler "en parallèle avec des données de test" tant que Trinôme 1 n'a pas livré. Ce mécanisme permet de tester `task_solution.py` tout seul, sans dépendre de l'avancement de l'autre trinôme, tout en étant prêt à basculer sur le vrai signal dès qu'il existe.

### 3. `_start_navigation()` — la transition

Une fois déclenchée (peu importe par quel chemin) :
1. `nav2_client.wait_for_server(timeout_sec=10.0)` — vérifie que Nav2 tourne. Si non → échec propre, pas de crash.
2. `goal_locator.get_goal()` — récupère la pose cible (actuellement toujours le **goal de test**, via les paramètres `goal_x`/`goal_y`, car la vraie source n'est pas encore identifiée).
3. `nav2_client.send_goal(goal, self._on_navigation_result)` — envoie l'action, avec une **callback** (`_on_navigation_result`) qui sera appelée plus tard, quand tout sera fini.

### 4. Dans `Nav2Client.send_goal()` — l'action Nav2 elle-même

```
send_goal()
   │
   ├─► envoie NavigateToPose (async) + démarre un timer 600s
   │
   ├─► [feedback en continu] _on_feedback() ──► log distance restante
   │
   ├─► [Nav2 accepte/refuse] _on_goal_response()
   │        ├─ refusé ──► _finish(FAILURE)
   │        └─ accepté ──► attend le résultat final
   │
   ├─► [résultat final arrive] _on_result()
   │        ├─ SUCCEEDED ──► _finish(SUCCESS)
   │        └─ autre       ──► _finish(FAILURE)
   │
   └─► [600s écoulées avant tout résultat] _on_timeout()
            ──► annule le goal + _finish(TIMEOUT)
```

Trois événements asynchrones indépendants (`_on_result`, `_on_timeout`, éventuellement un refus) peuvent tous vouloir "terminer" la navigation. C'est pour ça que `_finish()` a un verrou `self._finished` : **le premier qui arrive gagne**, les autres sont ignorés. Ça garantit que la callback finale n'est appelée **qu'une seule fois**, jamais deux.

### 5. Retour à `task_solution.py` — la fin

Quel que soit le résultat (`SUCCESS`, `FAILURE`, `TIMEOUT`), `_on_navigation_result()` est appelée exactement une fois :
- log du résultat
- `_shutdown()` → arrêt propre du node (`rclpy.shutdown()`)

## Schéma récapitulatif

```
task_solution.py (node)
   │
   ├── abonnement /localization_ready ──┐
   │                                     ├──► _start_navigation()
   ├── timer fallback 5s ────────────────┘         │
   │                                                ▼
   │                                   goal_locator.get_goal()
   │                                                │
   │                                                ▼
   │                                   nav2_client.send_goal(goal, callback)
   │                                                │
   │                                   ... action NavigateToPose ...
   │                                                │
   │                                                ▼
   │                          callback(SUCCESS | FAILURE | TIMEOUT)
   │                                                │
   ▼                                                ▼
_on_navigation_result() ◄─────────────────────────┘
   │
   ▼
_shutdown()
```

## Ce qui reste volontairement "test" à ce stade

- `use_test_goal=True` par défaut dans `goal_locator.py` → **doit être remplacé** une fois la vraie source du goal identifiée sur The Construct.
- `test_mode=True` par défaut dans `task_solution.py` → utile pour développer sans Trinôme 1, à repasser en réflexion pour le run final (le garder actif comme filet de sécurité, ou le désactiver pour forcer l'attente du vrai signal — à décider en J3 selon la fiabilité de Trinôme 1).