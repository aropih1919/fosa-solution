# PARC 2026 – Phase réelle : Guide des équipes

## Présentation

Vendredi, le code de chaque équipe conduit le CAYTU Sito-É physique jusqu'à quatre AprilTags, dans un ordre aléatoire donné au départ, sans heurter le mobilier ni les personnes.

Une course parfaite rapporte **100 points**, plus un bonus de **5 points** pour un démarrage vocal.

Le robot fonctionne de manière entièrement autonome pendant **10 minutes au maximum**.

Les équipes trouvent elles-mêmes les tags : aucune carte ni coordonnée de tag n'est fournie. Les essais sur le robot ont lieu du mardi au jeudi, sur des créneaux réservés.

---

## Programme de la semaine

Il n'y a qu'un seul robot. Les équipes le partagent sur des créneaux d'essai réservés jusqu'au gel du code, et les courses notées ont lieu le vendredi 6 novembre.

| Jour | Déroulement |
|---|---|
| Mar. 3 – jeu. 5 nov. | Essais : les équipes réservent des créneaux pour cartographier l'arène, trouver les tags et tester leur code. |
| Gel du code (avant la première course notée) | Chaque équipe soumet son code final. Seul ce code sera exécuté vendredi. |
| Ven. 6 nov. | Courses notées : une course de 10 minutes par équipe. |

Chaque équipe dispose de **deux créneaux d'essai de 15 minutes**, jamais consécutifs.

S'il reste du temps, il est réparti en créneaux supplémentaires.

Soyez à l'heure : un créneau non utilisé n'est pas reporté.

---

## L'arène

L'arène est un espace intérieur de **5 m × 5 m**, aménagé comme un restaurant de stade avec des tables, des chaises et d'autres meubles.

- Les obstacles fixes restent en place toute la semaine. Vous pouvez les cartographier pendant les essais.
- Les obstacles mobiles seront déplacés pour votre course notée. Votre robot doit détecter et éviter les obstacles absents de sa carte.
- Aucune carte n'est fournie. Chaque équipe construit la sienne.
- Les organisateurs pourraient interdire les cartes pré-construites vendredi (à confirmer) : soyez prêts à cartographier en direct pendant la course.
- Des personnes peuvent se trouver dans l'arène ou à proximité. Traitez toute personne comme un obstacle qui ne doit jamais être touché.

---

## La tâche

Votre robot doit visiter les tags **1, 4, 7 et 9** dans l'ordre publié pour votre robot au début de votre course.

### Les tags

| Caractéristique | Valeur |
|---|---|
| Famille | AprilTag 36h11 |
| Identifiants utilisés | 1, 4, 7, 9 (aucun autre tag dans l'arène) |
| Taille | 16,5 cm × 16,5 cm |
| Hauteur de fixation | Environ 1 m au-dessus du sol |
| Position et orientation | Non communiquées. Votre robot doit trouver et identifier chaque tag. |

### Ce qui compte comme une visite

Un tag est visité lorsque le robot s'arrête et que toutes les conditions suivantes sont réunies :

1. Son centre est à **0,5 m du tag**.
2. Il est face au tag, aligné sur la surface du tag, à **±12° près**.
3. Il garde cette position jusqu'à ce que les juges confirment la visite.

Lorsque le robot atteint un tag, votre code doit publier l'identifiant du tag et une image de la caméra sur les topics de statut des organisateurs.

Ces informations s'affichent sur un écran pour les juges et le public, et pilotent le chronomètre.

Les détails du paquet de statut suivront.

---

## Déroulement d'une course

1. Le robot est placé à la position de départ.
2. Les organisateurs publient l'ordre aléatoire des quatre tags sur un topic. Votre code le lit à partir de ce topic ; rien n'est saisi à la main.
3. Vous lancez la course, soit par une commande, soit en prononçant le mot de départ (voir Bonus de démarrage vocal). Le chronomètre de 10 minutes démarre à ce moment.
4. Dès le départ, il est interdit d'intervenir : ne touchez ni le robot, ni l'ordinateur, ni la session SSH.
5. La course se termine lorsque le robot signale qu'il a terminé, au bout de 10 minutes, ou en cas d'arrêt immédiat.

---

## Bonus de démarrage vocal

Les équipes gagnent **5 points de bonus** si le robot démarre sa course en entendant un mot de départ prononcé.

Le démarrage vocal est facultatif.

### Mots de départ

| Langue | Mot de départ |
|---|---|
| Anglais | `go` |
| Français | `allez` |
| Portugais | `vai` |

Votre robot n'a besoin de réagir qu'à un seul mot. En gérer plusieurs ne rapporte pas de points supplémentaires.

### Contraintes techniques

- Utilisez le réseau de **4 micros ReSpeaker** du robot.
- La reconnaissance vocale doit fonctionner entièrement hors ligne, sur le robot.
- Aucun service vocal en ligne ou dans le cloud n'est autorisé.
- La voix sert uniquement au démarrage. L'ordre des tags est publié sur un topic par les organisateurs.
- Si le démarrage vocal échoue, l'équipe lance le robot par commande et le bonus n'est pas accordé.
- Le robot doit attendre le mot de départ. S'il bouge avant, par exemple déclenché par le bruit du public, la course est recommencée et le bonus n'est pas accordé.

### Solutions de reconnaissance vocale proposées

Parmi les solutions hors ligne qui tournent sans difficulté sur le Jetson :

- **Vosk** avec un petit modèle limité à votre mot de départ.
- **whisper.cpp**.

Installez vos modèles sur le robot pendant les essais, pour que rien ne soit téléchargé pendant la course.

---

## Notation

Une course rapporte jusqu'à **100 points**.

Chaque tag visité rapporte 20 points, plus 5 points de navigation sûre : seul un robot qui effectue réellement des visites gagne ces points.

Les pénalités de contact sont déduites des points de navigation sûre.

### Barème

| Élément | Points |
|---|---:|
| Chaque tag visité | +20 (max. 80) |
| Navigation sûre | +5 par tag visité (max. 20) |
| Contact léger avec un meuble | −2 par contact, sur les points de navigation sûre |
| Contact dangereux | −5 par contact, sur les points de navigation sûre |
| Démarrage vocal | +5 de bonus |
| **Maximum** | **105** |

Un contact léger est un contact sans risque de dommage, par exemple un frôlement sur le côté.

Un contact dangereux est un contact qui pourrait endommager un meuble, un équipement ou le robot.

### Ordre de visite et tags sautés

Les tags doivent être visités dans l'ordre donné, mais le robot peut sauter un tag qu'il ne trouve pas.

- Un tag sauté ne rapporte rien.
- Le robot ne peut pas revenir plus tard sur un tag sauté.
- Les tags visités ensuite rapportent des points tant qu'ils respectent l'ordre donné.

**Exemple :** l'ordre est `7, 1, 9, 4`.

| Tags visités, dans cette séquence | Tags qui comptent | Points de tags |
|---|---|---:|
| 7, 1, 9, 4 | Les quatre | 80 |
| 7, 9, 4 (1 sauté) | 7, 9, 4 | 60 |
| 9, puis 7 | 9 seulement (7 vient avant 9 dans l'ordre) | 20 |

### Arrêt immédiat

La course s'arrête immédiatement, et tous les points de navigation sûre sont perdus, dans l'un de ces cas :

- **Perte de contrôle :** le robot se déplace d'une manière qu'il ne commande manifestement pas, ou ne peut pas s'arrêter.
- **Contact avec une personne.**
- **Arrêt de sécurité :** un organisateur peut arrêter le robot à tout moment pour des raisons de sécurité. Cela compte comme un arrêt immédiat, et non comme une intervention manuelle.

Les règles de départage seront annoncées avant vendredi.

---

## Règles de course

Une fois la course lancée, toute intervention est interdite jusqu'à la fin.

- **Durée maximale :** 10 minutes à partir de la commande ou du mot de départ.
- **Aucun contact :** aucun membre de l'équipe ne peut toucher le robot, ses commandes ou la session SSH après le départ.
- **Intervention manuelle :** toute intervention de l'équipe (soulever le robot, arrêt à distance, redémarrage d'un logiciel) oblige à recommencer la course.
- **Nouvelles tentatives :** le chronomètre continue de tourner depuis le premier départ. Recommencer ne donne pas de temps supplémentaire, et seul le score de la dernière tentative compte.
- **Fin de course :** la course se termine lorsque le robot signale qu'il a terminé, au bout de 10 minutes, ou en cas d'arrêt immédiat (voir Notation).
- **Juges :** la décision des juges sur les visites, les contacts et les arrêts est définitive.

---

## Le robot et le logiciel

Les organisateurs fournissent le robot et les données de ses capteurs.

Les équipes fournissent tout le reste :

- Cartographie.
- Détection des tags.
- Navigation.
- Reconnaissance vocale.

### Matériel

| Composant | Détails |
|---|---|
| Ordinateur | NVIDIA Jetson AGX Orin, 32 Go |
| Logiciel | ROS 2 Jazzy |
| LiDAR | RPLIDAR C1 |
| Caméras | 2 × Intel RealSense D435i RGB-D (haute et basse) |
| Autres capteurs | Centrales inertielles (IMU), odométrie des roues |
| Microphone | Réseau de 4 micros ReSpeaker |

### Ce qui tourne sur le robot

Les pilotes de la base et de tous les capteurs démarrent automatiquement comme services au démarrage du robot. Vous n'avez pas à les lancer.

Les noms des topics suivent la documentation de la Phase 1 :

- `/scan`
- `/imu`
- `/top_camera_color/image_raw`
- Etc.

L'URDF, l'arbre TF et la calibration des caméras sont ceux de la Phase 1.

Demandez aux organisateurs si vous avez besoin des versions du robot réel.

**Aucun paquet de navigation, de cartographie, d'AprilTag ou de reconnaissance vocale n'est fourni.**

Apportez et installez les vôtres.

Régler Nav2 pour le robot réel relève de votre équipe.

---

## Accès et déploiement

Toutes les équipes partagent un même compte utilisateur sur l'ordinateur du robot, avec un accès SSH pendant leurs créneaux d'essai.

### Exécution du code

- Exécutez votre code sur le robot.
- Exécuter des nœuds depuis votre ordinateur via le réseau est autorisé mais déconseillé : la latence du Wi-Fi peut provoquer des collisions.

### Organisation des fichiers

- Gardez tout votre travail dans un dossier au nom de votre équipe.
- Construisez-y votre espace de travail par-dessus l'espace de travail de base du robot.
- Ne modifiez ni les dossiers des autres équipes, ni l'espace de travail de base, ni les services système.
- Demandez aux organisateurs avant d'installer des paquets système.

**Important :** le robot est redémarré entre les créneaux, donc rien de ce que vous laissez tourner ne subsiste.

---

## Gel du code

Au gel du code, chaque équipe soumet son code final.

**Seul ce code est chargé et exécuté vendredi.**

Incluez donc des instructions claires pour :

1. Installer les dépendances.
2. Lancer votre solution.
3. Lancer la course, par commande et par la voix si vous visez le bonus.

L'heure du gel du code et le mode de soumission seront annoncés.

Votre code doit aussi signaler :

- Le départ.
- Chaque visite de tag.
- La fin de la course.

Ces événements doivent être signalés via le paquet de statut des organisateurs.

Les détails suivront.