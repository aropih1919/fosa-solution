# Fosa — notes techniques (version 5)

Ce document est pour l'équipe. Il dit ce qui a changé depuis la version 2
(mesurée à 2 min 33 s dans Gazebo), pourquoi, avec quelles preuves, comment
revenir en arrière réglage par réglage, et ce qui reste à vérifier dans Gazebo
avant la soumission.

## 1. Ce qui change par rapport à la version 2

| Changement | Pourquoi | Preuve | Revenir en arrière |
|---|---|---|---|
| Vitesse de croisière 0,5 → 0,7 m/s | gagner du temps en ligne droite | banc : roulage 26 → 18 s, mêmes marges | `-p cruise_speed:=0.5` |
| Chemin stabilisé (`path_keeper`) | supprimer les allers-retours gauche/droite devant une table | banc : 0 pivot d'hésitation en 15 essais du trajet officiel (v2 réglages : jusqu'à 7 s de pivots) | `-p path_keeper:=false` |
| Essieu pris en compte dans la localisation | position juste après les virages | banc : distance finale réelle 0,10-0,24 m (v2) → 0,02-0,09 m | `-p axle_offset:=0.0` |
| Ralentissement en virage sous 0,6 m de rayon (0,9 avant) | virages larges pris plus vite | banc : environ −1 s à 0,5 m/s | `-p curve_radius:=0.9` |
| Arrivée : 0,15 m/s sur les derniers 0,4 m (0,05 sur 0,6) | arrivée plus franche | banc : neutre en temps, distance finale un peu meilleure | `-p approach_speed:=0.05 -p approach_distance:=0.6` |
| Rapport de trajet en fin de course | savoir où part le temps | — | `-p write_report:=false` |
| Cause de chaque interruption, suite adaptée, repli si le but est occupé | robustesse | 140 tests unitaires ; banc : boîte sur le but | (sans objet) |
| Relance de Nav2 si son démarrage est bloqué | robustesse | banc : 2 blocages sur environ 120 démarrages, rattrapés | `-p nav2_startup_restarts:=0` |
| Délais de l'arbre de comportement 20 → 500 ms | une réponse lente ne doit pas interrompre la navigation | lecture du code de Nav2 | (dans `nav2_params.yaml`) |
| Nœuds relancés s'ils s'arrêtent (`respawn`) | robustesse | — | (dans les launch) |
| Fichiers inutiles retirés, identités anonymisées | présentation | — | — |

`-p ...` s'ajoute à la commande officielle :
`ros2 run caytu_nav_solution task_solution.py --ros-args -p cruise_speed:=0.5`.
`-p drive_profile:=reference` remet d'un coup tous les réglages de conduite de
la version 2 (vitesse, arrivée, virages, essieu, chemin stabilisé). Le rapport
de fin de trajet rappelle les réglages utilisés.

Essayés puis écartés, mesures à l'appui (section 8) : portée de la caméra haute
3,5 m, pivot à 0,95 rad/s, seuil de changement de route à 0,3 ou 1,0 m,
reconnaissance des tables par le lidar.

## 2. Architecture

```
task_solution.py  (point d'entrée officiel)
 ├─ lit task_params.yaml, vérifie la carte des murs avec un scan
 ├─ lance solution_bringup.launch.py, le relance si Nav2 reste bloqué au démarrage
 ├─ envoie le but, décide de la suite à chaque interruption (retry_policy.py)
 └─ écrit le rapport du trajet (run_report.py)

solution_bringup.launch.py
 ├─ scan_to_scan_filter_chain   /scan -> /scan_filtered (secteurs où le lidar voit le robot)
 ├─ lidar_floor_filter          /scan_filtered -> /scan_clean, /scan_clear ; repère horizontal
 ├─ top_cam_to_scan             nuage caméra haute -> /top_camera_scan (0,60-1,40 m)
 ├─ bottom_cam_to_scan          nuage caméra basse -> /bottom_camera_scan (0,25-1,40 m)
 ├─ odom_imu_localizer          /odom + /imu -> TF map -> odom, /localization_pose
 ├─ path_keeper                 action compute_path_stable (si path_keeper:=true)
 └─ navigation.launch.py        Nav2 : carte, planificateur Smac 2D, contrôleur RPP,
                                behaviors, bt_navigator, lifecycle manager
```

Deux arbres de comportement, identiques à une ligne près :
`navigate_bounded_recovery.xml` (défaut de Nav2) demande le chemin au
planificateur ; `navigate_keep_path.xml` le demande à `path_keeper`.
`task_solution.py` choisit le second pour chaque but **seulement si
`path_keeper` répond** ; après deux interruptions inexpliquées avec lui, il
revient au premier pour le reste du trajet. Un `path_keeper` absent ou en panne
ne peut donc pas empêcher le robot de rouler.

## 3. Chemin stabilisé (`path_keeper.py`, `path_choice.py`)

**Le problème, observé sur le banc.** Devant la première table du trajet
officiel, le robot pivotait parfois de gauche à droite pendant 5 à 16 s. Les
images de la costmap montrent pourquoi : la caméra haute ne voit que ±45° vers
l'avant et son bruit de profondeur (10 cm) fait « respirer » les obstacles
qu'elle regarde ; les pieds de chaise (un ou deux rayons du lidar) apparaissent
et disparaissent selon l'angle. Le côté regardé paraît plus encombré que
l'autre, le chemin suivant prend l'autre côté, le robot pivote, et ainsi de
suite.

**La règle.** À chaque chemin calculé (2 Hz) :

1. même route que le chemin suivi (aucun point à plus de 1 m de l'autre
   chemin) : le nouveau chemin est pris tel quel. En conduite normale, le nœud
   est donc transparent ;
2. autre route : elle n'est prise que si
   * la route suivie traverse une cellule interdite (coupée), ou
   * elle frôle un obstacle (cellule ≥ 90 dans l'OccupancyGrid au-delà de
     0,5 m devant le robot) deux calculs de suite, ou
   * le robot s'en est écarté de plus de 0,5 m, ou
   * la nouvelle route fait gagner plus de **1,5 m + 1 m par radian** de pivot
     qu'elle imposerait, et la route suivie a été choisie il y a plus de 5 s.

Le gain se calcule avec la formule de coût du planificateur Smac 2D
(longueur × (1 + 3 × coût/252)), en mètres équivalents, sur la partie du
chemin suivi encore devant le robot. Après plus de 2 s sans demande (une
récupération a eu lieu) ou un nouveau but, la mémoire est effacée.

**Coût.** O(n) par chemin de n points (quelques centaines) pour les coûts,
120 × 120 distances au plus pour comparer deux routes, deux fois par seconde.

**Paramètres** (nœud `path_keeper`) : `min_gain` 1,5 ; `turn_cost` 1,0 ;
`same_route_dist` 1,0 ; `min_dwell_sec` 5,0 ; `max_offset` 0,5 ;
`reset_after_sec` 2,0. `min_gain` se règle aussi depuis la commande :
`-p keeper_gain:=2.0`.

## 4. Localisation et essieu

L'odométrie du plugin DiffDrive (gz-sim, `DiffDrive.cc`) est calculée à partir
de la rotation des deux roues : elle décrit le **milieu de l'essieu**. Dans le
URDF officiel, les roues sont à y = −0,095 m de `base_link`, lui-même tourné de
+90° par rapport à `base_footprint` : l'essieu est 0,095 m **devant**
`base_footprint`. Quand le robot pivote de Δθ sur place, l'odométrie ne voit
aucune translation alors que `base_footprint` se déplace de
0,095 × 2 sin(Δθ/2) : 13 cm après un quart de tour, 19 cm après un demi-tour.

`odom_imu_localizer` intègre donc la distance des roues au point « essieu »,
avec le cap de l'IMU, puis place `base_footprint` 0,095 m derrière
(`OdomImuFusion`, paramètre `axle_offset`). Tests unitaires : demi-tour sur
place (erreur 0,19 m sans correction, < 1 mm avec), slalom.

**À vérifier dans Gazebo** (une fois) : lancer `tools/diag_nav.py` pendant un
trajet. La colonne `ecart` compare la position estimée à la position réelle
(`/sitoe_robot/pose`) ; elle doit rester sous 5 cm, y compris après les
virages. Avec `-p axle_offset:=0.0`, elle doit grandir à chaque changement de
cap. Si c'est l'inverse, remettre `axle_offset` à 0 et nous le signaler.

## 5. Quand quelque chose se passe mal

| Situation | Ce que fait `task_solution.py` |
|---|---|
| Nav2 refuse le but (pas encore actif) | renvoi toutes les 0,2 s, sans rien compter |
| Échec `TF_ERROR`, ou échec sans cause moins d'une seconde après l'envoi, robot immobile | renvoi après 0,3 s, sans rien effacer (cas typique : premier but juste après le démarrage) |
| Autre échec, avec progrès depuis le dernier | costmap globale effacée au-delà de 2,5 m du robot, renvoi |
| Deuxième échec sans progrès | point libre le plus proche du but (repli), sinon tout effacer |
| `START_OCCUPIED` | costmap globale effacée autour du robot |
| But occupé dans la costmap | repli immédiat ; arrivée déclarée dès qu'une partie du robot est dans le cercle de 0,6 m |
| Nav2 muet 30 s pendant un but | but annulé puis renvoyé |
| Nav2 pas prêt 60 s après le lancement | arrêt et relance de tout le bringup (2 fois au plus, délai doublé à chaque fois) |
| Deux échecs inexpliqués avec le chemin stabilisé | retour à l'arbre par défaut |

Le journal donne la cause de chaque interruption en clair, avec le code
d'erreur de Nav2.

**Démarrage bloqué.** Le gestionnaire de cycle de vie de Nav2 appelle
`change_state` sur chaque nœud sans limite de temps. Si la réponse se perd
(« failed to send response to /planner_server/change_state (timeout) » dans le
journal), il attend pour toujours. Vu deux fois sur environ 120 démarrages du
banc. Le délai de 60 s est volontairement large : le journal donne à chaque
trajet « Nav2 prêt et robot localisé après X s réelles » ; si X reste petit sur
vos machines, `-p nav2_startup_timeout_sec:=30.0` réduit la perte en cas de
blocage.

## 6. Rapport de trajet

À la fin de chaque trajet (même interrompu par Ctrl-C) :

```
===== Rapport du trajet =====
résultat : success    réglages : default
durée totale : 21.4 s simulées (23.0 s réelles)
  démarrage (avant le premier mouvement) : 4.1 s, dont Nav2 lancé à 3.6 s, premier but accepté à 3.9 s
  roulage : 16.9 s    pivots : 0.0 s (0)    arrêts : 0.0 s (0)
chemin : 11.52 m    vitesse moyenne en roulage : 0.68 m/s    vitesse max : 0.70 m/s
distance finale au centre du but : 0.11 m
tentatives : 1    interruptions : aucune    repli : non
contacts : 0
```

(exemple issu du banc). Le même contenu est écrit en JSON dans
`~/.ros/fosa_runs/run_<date>.json` et ajouté à `~/.ros/fosa_runs/trajets.csv`
(une ligne par trajet : pratique pour comparer des réglages). La distance
finale est celle estimée par la solution ; la distance réelle se lit avec
`tools/diag_nav.py`.

## 7. Réglages pour les essais comparatifs

| Paramètre | Défaut (v5) | Version 2 | Effet |
|---|---|---|---|
| `cruise_speed` | 0.7 | 0.5 | vitesse de croisière (m/s) |
| `turn_speed` | 0.8 | 0.8 | pivot sur place (rad/s) |
| `approach_speed` | 0.15 | 0.05 | vitesse minimale à l'arrivée (m/s) |
| `approach_distance` | 0.4 | 0.6 | distance de ralentissement avant le but (m) |
| `curve_radius` | 0.6 | 0.9 | rayon sous lequel le robot ralentit en virage (m) |
| `camera_range` | 2.5 | 2.5 | portée de marquage de la caméra haute (m) |
| `axle_offset` | 0.095 | 0.0 | essieu devant `base_footprint` (m) |
| `path_keeper` | true | false | chemin stabilisé |
| `keeper_gain` | 1.5 | — | gain exigé pour changer de route (m) |

Les valeurs par défaut sont dans `nav2_params.yaml` (marquées `[essai ...]`),
`solution_bringup.launch.py` et `drive_settings.py`. Un paramètre donné en
ligne de commande ne modifie aucun fichier : `navigation.launch.py` écrit une
copie temporaire de `nav2_params.yaml` pour cet essai seulement. Une valeur
hors des limites du robot arrête tout de suite avec un message clair.

**Protocole conseillé dans Gazebo.** Trois trajets `-p drive_profile:=reference`
et trois trajets sans paramètre, en alternant, puis comparer les lignes de
`trajets.csv`. Lire le temps **simulé** (« BUT ATTEINT en ... s », colonne
`sim_duration_s`) : c'est lui qui est comparable d'une machine à l'autre. Un
réglage n'est gardé que s'il fait mieux sur les trois.

## 8. Mesures sur le banc d'essai

**Le banc.** Le vrai Nav2 Jazzy (compilé depuis les sources de la branche
jazzy), la vraie solution, `robot_state_publisher` avec le URDF officiel, et un
simulateur simple à la place de Gazebo : cinématique du plugin DiffDrive
(limites, entraxe et rayon réels contre déclarés, essieu), lidar et caméras
calculés dans des coupes 2D du monde officiel, avec le bruit de profondeur de
10 cm sur x, y et z. Il ne simule ni glissement, ni chocs, ni rendu 3D des
caméras. Il est fourni à part (`banc_essai_fosa`, hors soumission).

**Calage.** Version 2 : 29,8 s simulées en moyenne sur le trajet officiel
(6 essais, 27,8 à 31,2 s). L'équipe a mesuré 2 min 33 s dans Gazebo, qui
tourne à environ 20 % du temps réel sur ses machines : environ 31 s simulées.

**Un changement à la fois** (trajet officiel, temps « BUT ATTEINT » en secondes
simulées, moyenne [min-max] ; « pivots » = temps passé à pivoter sur place) :

| Réglages | Essais | Temps | Pivots | Marge mini |
|---|---|---|---|---|
| version 2 | 6 | 29,8 [27,8-31,2] | 0,4 [0-1,7] | 0,35-0,38 m |
| version 5, réglages de la v2 (`reference`) | 7 | 32,5 [29,7-39,5] | 1,8 [0-7,4] | 0,28-0,38 m |
| + vitesse 0,7 | 3 | 24,0 [21,5-26,0] | 0,3 [0-0,8] | 0,34-0,39 m |
| + portée caméra 3,5 m | 3 | 1 démarrage bloqué ; 32,1 et 47,6 | jusqu'à 15,9 | — |
| + pivot 0,95 rad/s | 3 | 32,9 [30,0-38,3] | 2,0 [0,1-5,1] | — |
| + arrivée 0,15 / 0,4 | 3 | 30,5 [29,2-32,0] | 0,8 | — |
| + virages 0,6 m | 3 | 29,8 [29,0-30,8] | 0,1 | — |
| + essieu | 3 | 30,1 [29,5-30,9] | 0 | — |
| + chemin stabilisé, seuil 0,3 m (1re version) | 3 | 34,3 [32,1-36,1] | 2,1 | — |
| + chemin stabilisé, seuil 1,0 m | 3 | 31,4 [30,2-33,3] | 1,3 | — |
| + chemin stabilisé, seuil 1,5 m | 4 | 32,4 [29,0-34,7] | 0 | — |
| + chemin stabilisé + vitesse 0,7 | 7 | 22,5 [22,1-23,5] | 0 | 0,34-0,37 m |
| + idem + essieu, virages, arrivée | 4 | 22,8 [21,8-23,2] | 0,2 | 0,35-0,37 m |
| + chemin stabilisé + vitesse 0,85 | 4 | 20,4 [18,7-21,4] | 0,2 | 0,32-0,37 m |

Les lignes « chemin stabilisé » ont été mesurées avec des versions
successives du nœud ; la version livrée ajoute le délai de 5 s entre deux
changements de route, la confirmation du « trop près » et le prix du pivot.

*Validation finale du code livré (trajets variés, boîtes, démarrage bloqué) : tableau à venir dans la prochaine livraison.*

## 9. Ce qui a été écarté, et pourquoi

* **Portée de la caméra haute à 3,5 m.** Idée : voir les tables un mètre plus
  tôt. Sur le banc, les points lointains, bruités, referment des passages par
  intermittence : jusqu'à 16 s de pivots, avec ou sans chemin stabilisé.
* **Pivot à 0,95 rad/s.** Aucun gain mesurable ; le robot pivote déjà peu.
* **Reconnaître les tables par le lidar** (réserver le plateau dès qu'un pied
  est vu). Le lidar a 360 rayons : un pied de 4 cm n'en reçoit qu'un ou deux
  au-delà de 1,2 m, comme un pied de chaise ou une jambe. Trop de fausses
  tables possibles, qui fermeraient des passages. Remplacé par le chemin
  stabilisé, qui traite la conséquence (l'hésitation) sans deviner.
* **Replanifier seulement si le chemin devient invalide, Smac Hybrid.**
  Mesuré plus lent dans Gazebo par l'équipe (3 min 18 s contre 2 min 33 s).
* **Vitesse 0,85 m/s.** Plus rapide encore sur le banc, sans contact, marges
  à peine plus faibles. Gardée à 0,7 par prudence : le banc ne simule ni le
  tangage d'un robot de 1,25 m de haut au freinage, ni le glissement. À essayer
  dans Gazebo : `-p cruise_speed:=0.85`.

## 10. Ce qui est vérifié, et ce qui ne l'est pas

Vérifié :

* 138 tests unitaires du paquet `caytu_nav_solution` et 23 de
  `caytu_nav_bringup` (`colcon test` ou `python3 -m pytest` dans chaque paquet).
  `task_solution.py` y est déroulé en entier contre un faux Nav2 (`fake_ros.py`) ;
* sur le banc, avec le vrai Nav2 : trajet officiel, trajet inversé, quatre
  départs et buts tirés au hasard, boîte sur la route, boîte sur le but,
  machine chargée (deux processus qui occupent les deux cœurs), démarrage
  bloqué rattrapé.

Pas vérifié :

* **rien n'a encore tourné dans Gazebo dans cette version.** Le temps de
  référence reste celui de Gazebo ;
* le comportement au freinage à 0,7 m/s (tangage, glissement) ;
* la cause exacte des interruptions que l'équipe a vues dans Gazebo : le
  rapport de trajet la donnera désormais.

## 11. Avant la soumission

1. Compiler (`colcon build`), lancer les tests.
2. Dans Gazebo : trois trajets par défaut et trois `drive_profile:=reference`,
   en alternant ; comparer `trajets.csv` (temps simulé, pivots, contacts).
3. Une fois : `tools/diag_nav.py` pendant un trajet (section 4).
4. Garder un réglage seulement s'il fait mieux dans Gazebo ; sinon le remettre
   à la valeur de la version 2 dans `nav2_params.yaml`.
5. Compléter les noms dans le README et l'adresse des mainteneurs dans les deux
   `package.xml` (`equipe.fosa@example.com` est provisoire).
6. Ne pas soumettre le banc d'essai.
