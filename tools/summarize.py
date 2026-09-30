#!/usr/bin/env python3
"""Résume tous les runs de tools/results/ : une ligne par label.

Utilisation :
    python3 tools/summarize.py

Écrit aussi tools/results/summary.csv (à coller dans le README pour la
comparaison AVANT/APRÈS). Ce script n'a pas besoin de ROS.
"""

import argparse
import glob
import json
import os

import bench_core as core

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(description='Résumé des runs du benchmark.')
    parser.add_argument('--dir', default=os.path.join(SCRIPT_DIR, 'results'),
                        help='Dossier contenant les run_*.json')
    parser.add_argument('--csv', default=None,
                        help='Fichier CSV de sortie (défaut : <dir>/summary.csv)')
    args = parser.parse_args()

    runs = []
    for path in sorted(glob.glob(os.path.join(args.dir, 'run_*.json'))):
        with open(path, encoding='utf-8') as handle:
            runs.append(json.load(handle))

    if not runs:
        print(f'Aucun fichier run_*.json dans {args.dir}')
        return

    rows = core.summarize_runs(runs)
    print(core.format_table(rows))

    csv_path = args.csv or os.path.join(args.dir, 'summary.csv')
    core.write_summary_csv(rows, csv_path)
    print(f'\nCSV écrit : {csv_path}')


if __name__ == '__main__':
    main()
