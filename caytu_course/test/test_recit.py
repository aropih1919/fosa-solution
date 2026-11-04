"""Phrases d'explication de la course."""

from caytu_course.recit import (
    phrase_cible,
    phrase_depart,
    phrase_donnees_match,
    phrase_echec,
    phrase_plan,
    phrase_saut,
    phrase_tag_repere,
    phrase_visite,
    resume_fin,
)
from caytu_course.strategie import Plan


def test_le_saut_est_explique_avec_les_chiffres():
    phrase = phrase_saut(7, 160.0, 120.0, Plan((1, 9, 4), 106.0, 67.5),
                         Plan((7, 1), 107.0, 45.0))
    assert 'Tag 7 sauté' in phrase
    assert '160 s' in phrase and '120 s' in phrase
    assert '7 → 1 (45 points' in phrase
    assert '1 → 9 → 4 (68 points' in phrase
    assert 'jamais revisité' in phrase


def test_un_tag_qui_ne_tient_plus_seul_est_explique():
    phrase = phrase_saut(4, 90.0, 30.0, Plan((), 0.0, 0.0), Plan((), 0.0, 0.0))
    assert 'Même seul' in phrase


def test_le_plan_et_le_plan_vide_sont_lisibles():
    assert phrase_plan(Plan((7, 1), 100.0, 45.0), 300.0) == (
        'Plan : 7 → 1, environ 100 s de travail pour 300 s restantes '
        '(45 points espérés).')
    assert 'rien de mieux' in phrase_plan(Plan((), 0.0, 0.0), 20.0)


def test_le_resume_explique_chaque_tag_non_visite():
    lignes = resume_fin((7, 1, 9, 4), [7, 9], {1: 'sauté : trop loin'})
    assert lignes[0] == 'Tag 7 : visité (1e visite).'
    assert lignes[1] == 'Tag 1 : non visité — sauté : trop loin.'
    assert lignes[2] == 'Tag 9 : visité (2e visite).'
    assert 'temps écoulé' in lignes[3]
    assert lignes[4].startswith('Bilan : 2 tag(s) sur 4, 50 points')


def test_les_autres_phrases():
    assert 'voix' in phrase_depart('voix', 'allez') and 'allez' in phrase_depart('voix', 'allez')
    assert 'commande' in phrase_depart('commande', '')
    assert 'plus tard' in phrase_tag_repere(9, 2.34, 7)
    assert "c'est la cible" in phrase_tag_repere(7, 2.0, 7)
    assert 'jamais vu' in phrase_cible(7, False)
    assert '1.2 m' in phrase_cible(7, True, 1.2)
    assert 'Nav2' in phrase_echec(7, 'nav2', 1)
    assert 'VISITÉ (2/4)' in phrase_visite(1, 2, 4, 0.503, -1.2)
    assert 'en direct' in phrase_donnees_match(None, ())
    assert '1, 4, 7, 9' in phrase_donnees_match('carte', (9, 1, 7, 4))
