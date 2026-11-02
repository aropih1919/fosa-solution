# Leçons apprises

## [2026-11-03] — Construire le chemin de voix Piper depuis le format réel du modèle
- Timing : revue d’implémentation, avant exécution de l’installateur.
- Symptôme : le chemin de téléchargement aurait ajouté une composante inexistante au nom de la voix.
- Cause racine : le découpage supposait quatre champs séparés par des tirets, alors que le nom configuré contient la langue régionale, la voix et la qualité.
- Détecteur manquant : comparer le chemin généré à l’arborescence du dépôt des voix et à l’exemple configuré.
- Garde-fou : dériver le dossier du nom configuré, puis vérifier qu’il suit `langue/langue_REGION/voix/qualité` avant de télécharger.
