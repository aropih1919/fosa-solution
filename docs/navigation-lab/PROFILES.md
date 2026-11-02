# Profils de départ

Les fichiers `config/profiles/<profil>/<contrôleur>.yaml` sont des surcharges récursives du YAML commun, du plugin contrôleur, puis de la localisation. Les listes remplacent les listes précédentes; les mappings sont fusionnés. Le laboratoire exporte le résultat final complet dans `config_used.yaml` avant le lancement.

| Profil affiché | RPP | DWB | MPPI | Costmaps |
|---|---:|---:|---:|---|
| Très prudent | 0,15 m/s; collision anticipée à 2,0 s | 0,15 m/s; accélération 0,4 m/s² | 0,15 m/s; échantillonnage resserré | globale 1,20 m, locale 0,80 m |
| Prudent | 0,25 m/s; collision anticipée à 1,5 s | 0,25 m/s; accélération 0,6 m/s² | 0,25 m/s; batch réduit à 600 | globale 1,0 m, locale 0,65 m |
| Équilibré | 0,50 m/s, valeurs de `pre-final` | valeurs de `pre-final` | configuration MPPI initiale du dépôt | valeurs de référence inchangées |
| Rapide | 0,65 m/s | 0,65 m/s; accélération modérée | 0,65 m/s; accélération modérée | valeurs de référence inchangées |
| Très rapide | 0,85 m/s | 0,85 m/s; accélération plafonnée à 1,0 m/s² | 0,85 m/s; accélération plafonnée à 1,0 m/s² | valeurs de référence inchangées |
| Marge minimale | 0,25 m/s | 0,25 m/s | 0,25 m/s | globale 0,35 m, locale 0,30 m; essai de proximité Gazebo |

Les fichiers de vitesse ne changent pas les empreintes, les couches lidar/caméra, la détection de collision RPP ni le modèle de coût MPPI. Les facteurs d'inflation restent à 4,0 pour la costmap locale et RPP; les nouvelles distances de régulation RPP restent à l'intérieur du rayon d'inflation local. DWB conserve ses échantillons et critics de référence.

Ces valeurs sont des points de départ à comparer, pas des réglages optimisés. `balanced` reproduit les références RPP/DWB. MPPI n'existait pas dans la version soumise, donc son `balanced` est seulement sa configuration de base documentée. Tous les profils n'ont été préparés que pour Gazebo; aucun n'est validé sur le robot physique.

`tight_clearance` (affiché « Marge minimale ») sert à étudier en simulation le comportement à faible marge : le rayon local de 0,30 m ne laisse qu'environ 0,06 m de champ de coût après le rayon inscrit documenté du robot (0,24 m). Le coût se prolonge jusqu'au rayon d'inflation, mais seule la collision géométrique est bloquante; ce profil peut donc passer très près et provoquer des contacts. Il garde le footprint, le padding de 0,02 m, les couches de détection et les vérifications de collision RPP/ObstacleFootprint/CostCritic. Il est réservé à Gazebo et ne doit pas être essayé sur le robot physique.

Les valeurs d'inflation sont des distances en mètres, pas des pourcentages. Nav2 applique un coût décroissant autour des cellules d'obstacle jusqu'au rayon d'inflation; cela pénalise les chemins proches, sans créer une frontière binaire « interdit/recommandé ». La zone à coût létal dépend du footprint et des contrôles de collision du planificateur/contrôleur. La grille a une résolution de 0,05 m, donc la marge de `tight_clearance` correspond à environ une cellule au-delà du rayon inscrit. Voir la [documentation Jazzy de la couche d'inflation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/) et le [contrôle de collision RPP](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/).
