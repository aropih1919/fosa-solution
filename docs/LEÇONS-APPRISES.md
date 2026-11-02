# Leçons apprises

## [2026-11-02] — Les mots oui/non doivent être cités dans les mappings YAML
- Timing : validation de configuration
- Symptôme : le chargeur strict rejette les clés ou les mots `yes` et `no`.
- Cause racine : YAML 1.1 convertit les formes non citées `yes` et `no` en
  booléens au lieu de chaînes.
- Détecteur manquant : le chargement réel des fichiers YAML n'avait pas été
  exécuté après l'ajout des mots-clés de secours.
- Garde-fou : citer ces mots comme clés et valeurs, puis appeler `load_config()`
  sur la configuration assemblée avant de considérer le changement terminé.

## [2026-11-02] — Les loggers factices doivent couvrir les chemins d'erreur
- Timing : test
- Symptôme : le test du réessai Nav2 échoue quand il journalise un avertissement.
- Cause racine : le logger factice ne fournissait que `info()` alors que le
  navigateur appelle aussi `warn()` et `error()` selon le résultat.
- Détecteur manquant : les tests du chemin nominal n'exerçaient pas les appels
  de journalisation déclenchés par les résultats récupérables ou les erreurs.
- Garde-fou : faire parcourir aux tests les résultats succès, réessai et refus,
  puis lancer `colcon test --packages-select caytu_nav_solution`.
