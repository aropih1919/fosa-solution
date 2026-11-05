#!/usr/bin/env python3
"""Pilote l'arbitre officiel PARC (parc_referee) par son API web, sur NOTRE PC.

    ros2 run parc_referee referee --ros-args -p odom_topic:=/botwheel_explorer/odom
    python3 simulation/outils/arbitre_officiel.py armer --ordre 7,1,9,4
    ... course, départ ...
    python3 simulation/outils/arbitre_officiel.py attendre --sortie bilan.json

« armer » fait ce que fait le juge sur le tableau de bord (bouton Arm) : l'ordre
part alors sur /parc/tag_order. « attendre » suit la course jusqu'à FINISHED,
TIMED_OUT ou TERMINATED, puis écrit l'état final et rend 0 si la course est
propre : départ sans faux départ, chaque tag signalé dans l'ordre, image où
l'arbitre lit le bon tag, robot immobile pendant la pause.

À ne jamais lancer contre l'arbitre des organisateurs sur le robot : c'est le
juge qui arme. Jeton par défaut : celui du dépôt officiel (parc2026).
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

FINAUX = ('FINISHED', 'TIMED_OUT', 'TERMINATED')
# Pas de mandataire pour l'arbitre local, même si http_proxy est défini.
OUVREUR = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def requete(args, chemin, donnees=None):
    url = f'http://{args.hote}:{args.port}{chemin}'
    entetes = {'Content-Type': 'application/json'}
    corps = None
    if donnees is not None:
        entetes['X-Judge-Token'] = args.jeton
        corps = json.dumps(donnees).encode()
    demande = urllib.request.Request(url, data=corps, headers=entetes,
                                     method='POST' if corps is not None else 'GET')
    with OUVREUR.open(demande, timeout=5) as reponse:
        return json.loads(reponse.read().decode() or '{}')


def etat(args):
    return requete(args, '/api/state')


def attendre_serveur(args, delai):
    fin = time.monotonic() + delai
    while True:
        try:
            return etat(args)
        except (urllib.error.URLError, OSError):
            if time.monotonic() > fin:
                sys.exit(f'Arbitre injoignable sur http://{args.hote}:{args.port}/ '
                         '(ros2 run parc_referee referee lancé ?).')
            time.sleep(1)


def armer(args):
    courant = attendre_serveur(args, args.delai_serveur)
    if courant.get('state') in ('ARMED', 'RUNNING'):
        requete(args, '/api/judge/terminate', {'reason': 'nouvel essai Fosa'})
    if courant.get('state') != 'IDLE':
        requete(args, '/api/judge/reset', {})
    ordre = args.ordre.replace(',', ' ') if args.ordre != 'aleatoire' else ''
    try:
        reponse = requete(args, '/api/judge/arm', {'team': args.equipe, 'order': ordre})
    except urllib.error.HTTPError as erreur:
        sys.exit(f'Armement refusé : {erreur.code} {erreur.read().decode(errors="replace")}')
    print(f'Arbitre armé pour {args.equipe} : ordre {reponse.get("order")}', flush=True)


def bilan(resultat):
    lignes, propre = [], True
    lignes.append(f'état {resultat["state"]} ({resultat.get("end_reason") or "-"}) '
                  f'après {resultat["elapsed_s"]} s ; départ {resultat["start_method"] or "aucun"}'
                  + (f' (« {resultat["heard"]} »)' if resultat.get('heard') else ''))
    if resultat['state'] != 'FINISHED' or not resultat['start_method']:
        propre = False
    if resultat.get('false_start'):
        lignes.append('FAUX DÉPART')
        propre = False
    for visite in resultat['visits']:
        ok = (visite['order_status'] == 'in_order' and visite['image_check'] == 'match'
              and visite['hold'] in ('ok', 'pending'))
        propre &= ok
        lignes.append(f'{"VISITE" if ok else "ERREUR"} tag {visite["tag_id"]} à {visite["elapsed_s"]} s : '
                      f'{visite["order_status"]}, image {visite["image_check"]} '
                      f'{visite["detected_ids"]}, pause {visite["hold"]}')
    signales = [v['tag_id'] for v in resultat['visits'] if v['order_status'] == 'in_order']
    if signales != list(resultat['order']):
        propre = False
        lignes.append(f'tags dans l\'ordre {signales} au lieu de {resultat["order"]}')
    score = resultat.get('score') or {}
    if score:
        lignes.append('score ' + json.dumps(score, ensure_ascii=False))
    return propre, lignes


def attendre(args):
    attendre_serveur(args, args.delai_serveur)
    fin = time.monotonic() + args.delai
    resultat = etat(args)
    while resultat['state'] not in FINAUX and time.monotonic() < fin:
        time.sleep(2)
        resultat = etat(args)
    time.sleep(3.5)                       # dernière pause de 3 s, verdict « hold »
    resultat = etat(args)
    if args.sortie:
        with open(args.sortie, 'w', encoding='utf-8') as fichier:
            json.dump(resultat, fichier, ensure_ascii=False, indent=2)
    propre, lignes = bilan(resultat)
    print(('RÉUSSI ' if propre else 'ÉCHEC  ') + lignes[0])
    for ligne in lignes[1:]:
        print('    ' + ligne)
    sys.exit(0 if propre else 1)


def main():
    lecteur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    lecteur.add_argument('action', choices=('armer', 'attendre', 'etat'))
    lecteur.add_argument('--ordre', default='7,1,9,4', help='« 7,1,9,4 » ou « aleatoire »')
    lecteur.add_argument('--equipe', default='Fosa')
    lecteur.add_argument('--hote', default='127.0.0.1')
    lecteur.add_argument('--port', type=int, default=8080)
    lecteur.add_argument('--jeton', default='parc2026')
    lecteur.add_argument('--delai', type=float, default=720.0, help='durée max de la course (s)')
    lecteur.add_argument('--delai-serveur', type=float, default=30.0)
    lecteur.add_argument('--sortie', help='fichier JSON de l\'état final')
    args = lecteur.parse_args()
    if args.action == 'armer':
        armer(args)
    elif args.action == 'attendre':
        attendre(args)
    else:
        print(json.dumps(attendre_serveur(args, args.delai_serveur), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
