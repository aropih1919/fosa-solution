"""Mise en forme du suivi en direct."""

from caytu_course.suivi import formater


def test_un_evenement_porte_l_horloge_et_le_temps_restant():
    lignes = formater({'type': 'plan', 'texte': 'Plan : 7 → 1.', 'temps_ecoule': 75.4,
                       'temps_restant': 519.6}, couleurs=False)
    assert lignes == ['[01:15 | reste 08:39] Plan : 7 → 1.']


def test_la_fin_affiche_une_ligne_par_tag():
    lignes = formater({'type': 'fin', 'texte': 'x', 'temps_ecoule': 600, 'temps_restant': 0,
                       'lignes': ['Tag 7 : visité (1e visite).', 'Bilan : 1 tag(s)']},
                      couleurs=False)
    assert lignes[0] == '[10:00 | reste 00:00] === FIN ==='
    assert lignes[1:] == ['    Tag 7 : visité (1e visite).', '    Bilan : 1 tag(s)']


def test_les_couleurs_entourent_le_texte():
    ligne = formater({'type': 'visite', 'texte': 'Tag 7 VISITÉ'})[0]
    assert '\033[1;32mTag 7 VISITÉ\033[0m' in ligne
