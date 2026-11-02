# Guide de validation manuelle — phase finale

Ce guide décrit les essais à effectuer après compilation. Aucun essai Gazebo,
LLM, perception ou audio n'est réputé validé par sa seule présence ici : noter
les résultats dans le tableau final. La validation Gazebo reste à effectuer par
l'équipe.

## 1. Préparer le workspace

Depuis la racine du workspace ROS 2 :

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to caytu_assistant caytu_perception caytu_semantic_map
source install/setup.bash
```

Les paquets officiels du robot/Gazebo doivent également être installés et
sourcés. Les réglages utilisés viennent de `caytu_config/config/`; adapter un
poste avec `FOSA_CONFIG_DIR`, `FOSA_CONFIG_OVERRIDE` ou les variables de secrets
plutôt qu'en modifiant les nœuds Python.

## 2. Carte sémantique et services

Terminal 1 :

```bash
ros2 run caytu_semantic_map semantic_map_node
```

Terminal 2 :

```bash
ros2 topic echo /semantic_map/locations
ros2 service call /semantic_map/resolve_location caytu_interfaces/srv/ResolveLocation "{query: toilettes}"
ros2 service call /semantic_map/get_location_by_tag caytu_interfaces/srv/GetLocationByTag "{tag_id: 11}"
```

Vérifier les noms/alias, le tag associé, et `has_approach_pose`. Sans pose
statique ni détection du tag, le service doit retourner un lieu connu sans
inventer une pose d'approche.

## 3. Dialogue, LLM et secours déterministe

Pour une session texte sans micro, lancer le launch complet :

```bash
ros2 launch caytu_assistant assistant.launch.py input_mode:=text
```

Le mode texte affiche une invite dans le terminal. Essayer successivement une
salutation, une question FAQ, une demande de localisation, une demande de
navigation avec confirmation, une destination inconnue, une urgence et `stop`.
Observer les états et réponses :

```bash
ros2 topic echo /assistant/state
ros2 topic echo /assistant/response
```

Le dispatcher ne doit envoyer un but Nav2 qu'après validation de l'intention,
du lieu du registre et de la pose d'approche. Vérifier que le LLM ne fournit ni
coordonnées ni commande moteur.

Pour essayer Ollama, régler `dialogue.llm.provider: local` et `local_model` dans
`fosa.yaml`, démarrer Ollama et télécharger le modèle configuré. Pour une API
compatible OpenAI, régler `provider: api`, `api_base` et `api_model`, puis fournir
`FOSA_LLM_API_KEY` ou `OPENAI_API_KEY` dans l'environnement. `provider: auto`
essaie le local puis l'API. Couper volontairement le fournisseur permet de
vérifier le secours par règles; ce scénario ne nécessite pas de service externe.

Pour publier un transcript sans utiliser le micro :

```bash
ros2 topic pub --once /assistant/transcript std_msgs/msg/String "{data: 'Où sont les toilettes ?'}"
```

## 4. Audio STT et TTS

- **Audio enregistré** : `ros2 launch caytu_assistant assistant.launch.py input_mode:=audio_file audio_file:=/chemin/vers/extrait.wav`.
- **Microphone** : utiliser `input_mode:=microphone`, puis vérifier le périphérique et le seuil VAD dans `fosa.yaml`.
- **TTS local** : vérifier le chemin de `dialogue.tts.local_voice`, l'exécutable Piper et le lecteur audio configurés sous `audio`.
- **API** : configurer fournisseur, URL, modèle, voix et clé d'API. Tester aussi l'indisponibilité du fournisseur : la réponse texte doit rester publiée et le nœud ne doit pas bloquer.

Les dépendances audio (par exemple `faster-whisper`, `sounddevice`, Piper et un
lecteur WAV) doivent être installées sur la machine qui exécute les nœuds.

## 5. AprilTags et perception

Avec les caméras, TF et le robot disponibles :

```bash
ros2 topic echo /perception/tags
ros2 topic echo /perception/entities
ros2 topic echo /perception/person_events
```

Vérifier qu'un tag configuré produit une pose dans le frame sémantique et que
le service par tag retrouve le lieu. Pour enregistrer les observations de
calibration dans le fichier configuré, lancer séparément :

```bash
ros2 run caytu_perception tag_calibrator
```

La calibration écrit un YAML distinct; elle ne modifie pas la configuration
versionnée. Pour la classification, vérifier d'abord le mode géométrique avec
`perception.yolo.enabled: false`, puis essayer le modèle YOLO configuré si
disponible. Confirmer qu'un événement personne exige les critères de distance,
confiance, stabilité, état du robot et délai de réarmement configurés.

## 6. Essais Gazebo et navigation — à confirmer par l'équipe

Ne considérer aucun essai de navigation comme accepté avant l'exécution réelle.
Suivre le scénario phase 1 du plan et utiliser le monde officiel du projet :

```bash
./src/fosa-solution/cmd.sh gazebo
```

Dans un autre terminal, lancer le bringup/navigation du workspace selon le
scénario officiel, puis contrôler séparément : but atteint dans le temps,
absence de collision, arrêt sur `stop`, pose d'approche sémantique, tag visible
et absence de déplacement pour un lieu inconnu. Garder les journaux ROS en cas
d'échec. Le facteur temps réel bas ou les délais de `compute_path_to_pose`
doivent être notés comme résultat non concluant, pas comme réussite.

## 7. Réseau `remote_pc`

Configurer le même `ROS_DOMAIN_ID` et le bon middleware/interface réseau des
deux côtés; vérifier les topics et services avec `ros2 topic list` et
`ros2 node list`. Confirmer que `use_sim_time` est identique et cohérent avec
la source de `/clock`. Vérifier la navigation depuis le PC puis tester une
coupure réseau dans une zone sûre. **Le watchdog réseau robot décrit dans le
plan n'est pas encore implémenté** : ne pas faire d'essai de coupure avec un
robot en mouvement avant d'avoir livré et validé cette protection.

## 8. Résultats à remplir

| Scénario | Résultat (réussi/échoué/non exécuté) | Notes / journal |
|---|---|---|
| Build des nouveaux paquets | | |
| Registre et services sémantiques | | |
| Dialogue texte + secours sans LLM | | |
| LLM local | | |
| LLM API | | |
| STT fichier / microphone | | |
| TTS local / API | | |
| AprilTag et résolution par tag | | |
| Géométrie sans YOLO | | |
| YOLO et suivi personne | | |
| Gazebo phase 1 | | |
| Navigation sémantique en simulation | | |
| Réseau `remote_pc` | | |
