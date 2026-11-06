"""Phrases qui expliquent ce que fait le robot (sans ROS).

Chaque décision de la course est publiée en clair sur le topic des
événements et affichée par `./run.sh suivi` : un spectateur doit comprendre
pourquoi un tag est visité, raté ou sauté sans lire le code.
"""

RAISONS_ECHEC = {
    'nav2': "Nav2 n'a pas pu atteindre le point d'approche (obstacle ou chemin bloqué)",
    'perdu': 'le tag a été perdu de vue',
    'alignement': "le placement à 0,5 m n'a pas été stable dans le délai",
    'introuvable': "le tag n'était pas à sa position mémorisée",
    'biais': 'le tag était vu trop de biais ou de trop loin pour finir à la caméra',
}


RESULTATS_APPROCHE = {
    'arrive': 'arrivé devant',
    'reessayer': "Nav2 n'a pas atteint le point d'approche",
    'perdu': "tag non revu à l'arrivée",
    'echec': 'approche interrompue',
    'temps_ecoule': 'temps écoulé',
}


def _secondes(valeur):
    return f'{valeur:.0f} s'


def _suite(tags):
    return ' → '.join(str(tag) for tag in tags) if tags else 'aucun tag'


def phrase_ordre(ordre):
    return f"Ordre des tags reçu de l'arbitre : {_suite(ordre)}."


def phrase_depart(source, entendu):
    if source == 'voix':
        return f'Départ donné à la voix (« {entendu} ») : chronomètre lancé.'
    return 'Départ donné par commande : chronomètre lancé.'


def phrase_preparation(directory, duree_max):
    return (f'Préparation : nouvelle carte et recherche des 4 tags '
            f'({duree_max / 60:.0f} min au plus). Données enregistrées dans {directory}.')


def phrase_donnees_match(carte, tags):
    if carte is None:
        return ('Aucune préparation enregistrée : la carte est construite en direct '
                'et les tags devront être cherchés.')
    if not tags:
        return 'Carte de la préparation chargée, mais aucun tag mémorisé : recherche nécessaire.'
    return (f'Carte de la préparation chargée ; tags déjà localisés : '
            f'{", ".join(str(tag) for tag in sorted(tags))}.')


def phrase_tag_repere(tag_id, distance, cible):
    if cible is None:
        return f'Tag {tag_id} repéré à {distance:.1f} m.'
    if tag_id == cible:
        return f'Tag {tag_id} repéré à {distance:.1f} m : c\'est la cible, on y va.'
    return f'Tag {tag_id} repéré à {distance:.1f} m : position mémorisée pour plus tard.'


def phrase_cible(tag_id, connu, distance=None):
    if connu:
        return f'Nouvelle cible : tag {tag_id}, déjà localisé (vu à {distance:.1f} m).'
    return f'Nouvelle cible : tag {tag_id}, encore jamais vu : recherche.'


def phrase_plan(plan, temps_restant):
    if not plan.tags:
        return (f'Plus aucun tag ne tient dans les {_secondes(temps_restant)} restantes : '
                'on continue la cible actuelle, il n\'y a rien de mieux à tenter.')
    return (f'Plan : {_suite(plan.tags)}, environ {_secondes(plan.duree_s)} de travail '
            f'pour {_secondes(temps_restant)} restantes ({plan.valeur:.0f} points espérés).')


def phrase_saut(cible, cout_cible, temps_restant, plan_sans, plan_avec):
    """Pourquoi la cible est abandonnée, chiffres à l'appui."""
    debut = (f'Tag {cible} sauté : il faudrait encore environ {_secondes(cout_cible)} '
             f'pour l\'atteindre et le visiter, et il reste {_secondes(temps_restant)}.')
    if not plan_avec.tags:
        avec = ' Même seul, il ne tient plus dans le temps restant.'
    else:
        avec = (f' En le gardant, le mieux possible est {_suite(plan_avec.tags)} '
                f'({plan_avec.valeur:.0f} points espérés).')
    sans = (f' En le sautant : {_suite(plan_sans.tags)} '
            f'({plan_sans.valeur:.0f} points espérés). Il ne sera jamais revisité.')
    return debut + avec + sans


def phrase_echec(tag_id, raison, essais):
    explication = RAISONS_ECHEC.get(raison, raison)
    return f'Visite du tag {tag_id} ratée ({essais}e échec) : {explication}. On réessaie.'


def phrase_visite(tag_id, rang, total, distance, cap_deg):
    return (f'Tag {tag_id} VISITÉ ({rang}/{total}) : centre à {distance:.2f} m, '
            f'face au tag à {abs(cap_deg):.1f}° près. Photo envoyée à l\'arbitre.')


def resume_fin(ordre, visites, raisons):
    """Une ligne par tag de l'ordre : visité, ou pourquoi il ne l'a pas été."""
    lignes = []
    for tag in ordre:
        if tag in visites:
            lignes.append(f'Tag {tag} : visité ({visites.index(tag) + 1}e visite).')
        else:
            lignes.append(f'Tag {tag} : non visité — {raisons.get(tag, "temps écoulé avant d’y arriver")}.')
    points = 25 * len(visites)
    lignes.append(f'Bilan : {len(visites)} tag(s) sur {len(ordre)}, '
                  f'{points} points espérés (hors bonus vocal et pénalités).')
    return lignes
