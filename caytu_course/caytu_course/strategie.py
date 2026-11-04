"""Stratégie de visite : quels tags tenter, compte tenu du temps restant.

Règlement : l'ordre est imposé, un tag peut être sauté mais jamais revisité,
et les tags suivants rapportent toujours leurs points. Les plans possibles
sont donc les sous-séquences des tags restants : 16 au plus pour 4 tags. On
les évalue toutes (solution exacte, en quelques microsecondes) :

- durée estimée de chaque plan : trajets (distance x détour / vitesse),
  temps de visite, temps de recherche pour un tag jamais vu ;
- valeur : points par tag x probabilité de réussite, qui baisse à chaque
  échec et avec le temps déjà passé à chercher un tag introuvable ;
- un plan n'est retenu que si sa durée, majorée d'une marge, tient dans le
  temps restant ; on garde le plus rentable, puis le plus court.

Le nœud de course replanifie toutes les quelques secondes (horizon
glissant). La cible courante n'est sautée que si le meilleur plan sans elle
rapporte nettement plus (`gain_min_saut`) que le meilleur plan avec elle : à
valeur égale, on ne change pas d'avis. Le temps déjà dépensé sur une cible
n'entre pas dans la décision : seul compte ce qui peut encore être gagné.
"""

from dataclasses import dataclass
from itertools import combinations
import math


@dataclass(frozen=True)
class ModeleStrategie:
    points_par_tag: float     # 20 points de visite + 5 de navigation sûre
    vitesse_mps: float        # vitesse moyenne réelle, virages compris
    facteur_detour: float     # longueur du trajet / distance à vol d'oiseau
    temps_visite_s: float     # approche finale, alignement et maintien
    temps_recherche_s: float  # trouver un tag jamais vu
    trajet_inconnu_m: float   # trajet depuis une position inconnue
    marge: float              # durées majorées de ce ratio
    reussite_connu: float     # probabilité de visiter un tag déjà localisé
    reussite_inconnu: float   # probabilité de trouver puis visiter un tag
    baisse_par_echec: float   # facteur appliqué à chaque échec
    gain_min_saut: float      # points à gagner pour sauter la cible courante


@dataclass(frozen=True)
class TagRestant:
    tag_id: int
    position: tuple = None    # point d'approche (x, y) dans map, ou None
    echecs: int = 0           # tentatives de visite ratées
    temps_passe_s: float = 0.0  # déjà passé sur ce tag (cible courante)


@dataclass(frozen=True)
class Plan:
    tags: tuple
    duree_s: float
    valeur: float


def probabilite(tag, modele):
    if tag.position is not None:
        return modele.reussite_connu * modele.baisse_par_echec ** tag.echecs
    # Chercher pendant le temps prévu est normal ; au-delà, chaque durée de
    # recherche supplémentaire sans le trouver compte comme un échec.
    depassement = max(0.0, tag.temps_passe_s - modele.temps_recherche_s)
    echecs = tag.echecs + depassement / modele.temps_recherche_s
    return modele.reussite_inconnu * modele.baisse_par_echec ** echecs


def duree_tag(depuis, tag, modele):
    """(durée estimée, position après la visite) pour aller visiter `tag`."""
    if tag.position is None:
        return modele.temps_recherche_s + modele.temps_visite_s, None
    if depuis is None:
        distance = modele.trajet_inconnu_m
    else:
        distance = math.dist(depuis, tag.position)
    trajet = distance * modele.facteur_detour / modele.vitesse_mps
    return trajet + modele.temps_visite_s, tag.position


def evaluer(depuis, tags, modele):
    """(durée, valeur) d'une suite de tags visités dans cet ordre."""
    duree = valeur = 0.0
    position = depuis
    for tag in tags:
        duree_visite, position = duree_tag(position, tag, modele)
        duree += duree_visite
        valeur += modele.points_par_tag * probabilite(tag, modele)
    return duree, valeur


def planifier(position_robot, restants, temps_restant, modele, avec_premier=False):
    """Meilleur plan (sous-séquence des tags restants) tenant dans le temps.

    avec_premier : ne considérer que les plans qui commencent par la cible.
    """
    meilleur = Plan((), 0.0, 0.0)
    for nombre in range(len(restants), 0, -1):
        for choix in combinations(restants, nombre):
            if avec_premier and choix[0] is not restants[0]:
                continue
            duree, valeur = evaluer(position_robot, choix, modele)
            if duree * (1.0 + modele.marge) > temps_restant:
                continue
            if (valeur, -duree) > (meilleur.valeur, -meilleur.duree_s):
                meilleur = Plan(tuple(tag.tag_id for tag in choix), duree, valeur)
    return meilleur


def decider(position_robot, restants, temps_restant, modele):
    """(garder la cible ?, plan suivi). La cible est `restants[0]`.

    Un plan vide (plus rien ne tient dans le temps) ne fait pas sauter : il
    n'y a rien de mieux à tenter.
    """
    meilleur = planifier(position_robot, restants, temps_restant, modele)
    if not meilleur.tags or meilleur.tags[0] == restants[0].tag_id:
        return True, meilleur
    avec_cible = planifier(position_robot, restants, temps_restant, modele, True)
    if avec_cible.tags and meilleur.valeur - avec_cible.valeur < modele.gain_min_saut:
        return True, avec_cible
    return False, meilleur
