# Reprise IA — phase finale

Mise à jour : 2026-11-02. Workspace : `src/fosa-solution`.

## Objectif utilisateur

Continuer les lots de code de `docs/PLAN_PHASE_FINALE.md` laissés incomplets
par OpenCode. Ne pas interrompre la migration. L'utilisateur effectuera les
essais Gazebo; fournir les guides manuels, sans prétendre que les essais sont
réussis. Les scénarios LLM, audio, perception et réseau sont aussi consignés
dans `docs/GUIDE_TESTS_PHASE_FINALE.md`.

## Référence NLP externe

Le dépôt source de l'ancien assistant est
[https://github.com/Faneva01/pole_nlp](https://github.com/Faneva01/pole_nlp).
Une IA sur un autre poste peut le consulter pour retrouver le validateur, le
registre, l'action et les prompts d'origine. Le code de cette branche reste
autonome et ne doit pas dépendre d'un clone local de ce dépôt.

## Consignes de code non négociables

- Lire `AGENTS.md` avant de modifier le code. Il définit le style
  `refactor/franglais` pour toutes les IA : noms métier, fonctions, méthodes,
  classes et commentaires en français (`get_nom`, `naviguer_robot`); termes
  ROS/Python/protocoles/API restent en anglais.
- Toute valeur métier modifiable doit être dans `caytu_config`, jamais codée
  en dur en Python : URL fournisseur, topic, seuil, durée, périphérique, modèle,
  taille d'entrée, phrase de réponse ou mot-clé. Les suffixes d'API et contrats
  externes restent ceux du protocole.
- Garder les champs des interfaces ROS et du JSON LLM inchangés.
- Le LLM propose intention et noms de lieux uniquement. Le dispatcher valide
  les lieux et pilote le chemin déterministe Nav2.
- L'utilisateur garde la validation Gazebo. Ne pas lancer de simulation ni de
  suite de tests. Une compilation ou `py_compile` est une vérification statique,
  pas un test d'acceptation.

## Changements déjà présents dans l'arbre de travail

- `caytu_config`: schémas et YAML pour navigation, lieux, perception, audio,
  dialogue, mots-clés de secours et réponses.
- `caytu_interfaces`: messages et services sémantiques, état dialogue,
  détections personne/obstacle et tags.
- `caytu_semantic_map`: registre configurable, résolution nom/alias/tag,
  publication durable des lieux, calcul de pose d'approche depuis tag.
- `caytu_assistant`: validation d'intention, client Ollama/API, fallback
  déterministe, dialogue/Nav2, STT texte/fichier/micro, TTS Piper/API.
- `caytu_perception`: détecteur/calibrateur AprilTag, classification géométrique
  avec fusion YOLO facultative, suivi et événements personne.
- Lot navigation déjà extrait dans `caytu_nav_solution/navigation_runtime.py`;
  la validation phase 1 reste en attente.
- `AGENTS.md` fixe les règles de migration pour les futurs agents.

## État technique immédiat / prochains gestes

1. Vérification déjà faite dans cette reprise : `python3 -m compileall` passe
   pour les nouveaux modules; `colcon build` ciblé a construit les 7 paquets
   concernés dans `/tmp`; `load_config()` charge les YAML et retourne le mode
   `microphone`, l'URL locale, 11 réponses et 5 lieux. Aucun test ni Gazebo
   n'a été lancé. `git diff --check` passe.
2. Relire les derniers changements dans `dispatcher.py`, `models.py`, les
   launch files et le YAML. Une correction de flux de confirmation vient d'être
   faite; le balayage des anciens noms de classes n'a trouvé aucune référence
   dans les trois nouveaux paquets.
3. Continuer l'audit des valeurs modifiables dans les nouveaux modules :
   vérifier que les valeurs de topic viennent du loader et repérer les valeurs
   métier restantes qui méritent une clé de configuration.
4. Mettre à jour les cases des lots de `docs/PLAN_PHASE_FINALE.md` avec soin :
   distinguer l'implémentation de l'acceptation manuelle. Ajouter un lien vers
   le présent guide et ce fichier de reprise.
5. Les clés et valeurs YAML `yes` et `no` dans `prompts.yaml` doivent rester
   entre guillemets pour éviter que YAML les transforme en booléens.
6. Les constantes purement mathématiques, protocolaires et les numéros de
   champs API n'ont pas à devenir des réglages.
7. Watchdog `remote_pc` : l'architecture et le guide signalent clairement qu'il
   reste à implémenter. Ne pas déclarer le lot réseau fait. Avant toute
   utilisation robot réel, concevoir un heartbeat robot sûr, configurable et
   validé manuellement; ne pas publier à l'aveugle des commandes vitesse qui
   entreraient en concurrence avec Nav2.

## État Git connu avant cette reprise

Changements préexistants à conserver : `caytu_config` et ses YAML,
`caytu_nav_solution/task_solution.py`, `docs/PLAN_PHASE_FINALE.md`,
`navigation_runtime.py`, ses tests unitaires déjà écrits, les nouveaux paquets,
`docs/LEÇONS-APPRISES.md`. Des fichiers sont non suivis : ne pas les supprimer.
