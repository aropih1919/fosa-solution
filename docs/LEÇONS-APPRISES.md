# Leçons apprises

## [2026-11-03] — Construire le chemin de voix Piper depuis le format réel du modèle
- Timing : revue d’implémentation, avant exécution de l’installateur.
- Symptôme : le chemin de téléchargement aurait ajouté une composante inexistante au nom de la voix.
- Cause racine : le découpage supposait quatre champs séparés par des tirets, alors que le nom configuré contient la langue régionale, la voix et la qualité.
- Détecteur manquant : comparer le chemin généré à l’arborescence du dépôt des voix et à l’exemple configuré.
- Garde-fou : dériver le dossier du nom configuré, puis vérifier qu’il suit `langue/langue_REGION/voix/qualité` avant de télécharger.

## [2026-11-04] — Une échéance doit suivre la même horloge que le chronomètre
- Timing : premier essai complet en simulation de la course.
- Symptôme : un tag sauté « budget épuisé » après 42 s de course pour un budget de 149 s.
- Cause racine : budget calculé sur l'horloge ROS (temps simulé) mais échéance comparée à `time.monotonic()` ; la simulation tournait à 0,3× le temps réel.
- Détecteur manquant : test ou essai en simulation plus lente que le temps réel.
- Garde-fou : toute échéance de mission passe par l'horloge du chronomètre (`_temps()`), jamais par l'horloge système.

## [2026-11-04] — Tester une expression régulière bash sur des entrées réelles
- Timing : essai 3 en simulation, commande `./run.sh ordre-test`.
- Symptôme : tout ordre valide (« 9,4,7,1 ») refusé ; la course attendait un ordre qui ne venait pas.
- Cause racine : dans `[[ =~ ]]`, `\[\]` ferme l'ensemble entre crochets trop tôt.
- Détecteur manquant : essai du motif sur quelques entrées valides et invalides.
- Garde-fou : motif dans une variable (`motif='^[][0-9 ,]+$'`) et vérification sur des entrées attendues avant de livrer un script.
