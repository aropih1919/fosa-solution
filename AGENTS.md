# Contexte de code — refactor/franglais

Ce dépôt suit le style de `refactor/franglais`. Toute nouvelle contribution
doit respecter ces règles, quel que soit l’agent qui la produit.

## Langue du code

- Écrire en français les noms des variables métier, fonctions, méthodes,
  classes métier, commentaires, docstrings et messages destinés à l’équipe.
- Employer des verbes précis : `get_nom()`, `naviguer_robot()`,
  `charger_carte()`, `recevoir_scan()`.
- Garder en anglais les termes consacrés par ROS 2, Nav2, Python, les
  bibliothèques et les protocoles : `Node`, `Pose2D`, `Nav2Navigator`,
  `ActionClient`, `topic`, `frame_id`, `header`, `callback`, `JSON`, `LLM`.
- Les noms imposés par une API externe ou une interface ROS ne sont jamais
  traduits : types de messages, champs, services, topics, paramètres, noms de
  paquets, entry points et clés YAML déjà publiées.
- Les identifiants métier d’un adaptateur interne peuvent être en français,
  même si la classe qu’il enveloppe porte un nom anglais.
- Garder les commentaires et docstrings en français, sauf les quelques termes
  techniques sans équivalent utile.

## Règles de migration

- Quand un module est porté ou refactoré, traduire ses noms métier dans la
  même modification ; ne pas laisser deux conventions parallèles.
- Ne pas renommer une interface publique utilisée par ROS, un launch file,
  un autre paquet, un fichier de configuration ou un script documenté sans
  adapter tous ses consommateurs.
- Les champs structurés d’intention (`intentions`, `lieux`, `confirmation`,
  `reponse`, `langue`) restent stables sur le fil JSON, même si les fonctions
  Python qui les construisent sont nommées en français.
- Ne pas traduire le vocabulaire de configuration ou les chaînes de protocole
  que des outils externes attendent.

## Configuration et sûreté

- Les seuils, topics, noms de modèles, durées, périphériques et autres valeurs
  métier viennent de `caytu_config`; ne pas les recopier comme constantes dans
  les nœuds.
- Cela inclut les mots-clés de secours, les textes de réponse, les chemins et
  URL de fournisseurs, les tailles d’entrée des modèles et les paramètres de
  classification : si l’équipe peut vouloir les modifier, ils vont dans
  `caytu_config`.
- Les valeurs fixes d’un protocole ou les détails d’algorithme ne sont pas des
  réglages métier ; les valeurs par défaut qui influencent le comportement le
  sont et doivent être déclarées dans les fichiers de configuration.
- Le LLM propose uniquement une intention et des noms de lieux. La validation
  et le dispatcher déterministe décident des actions ; aucun code LLM n’envoie
  directement des coordonnées ou des commandes moteur.
