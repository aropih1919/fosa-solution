#!/usr/bin/env python3
"""Vérifie les imports puis la création d'un nœud, sans commande au robot."""

import os
import sys
import traceback


def main():
    print(f'Python : {sys.executable} ({sys.version.split()[0]})', flush=True)
    for nom in ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                'CYCLONEDDS_URI', 'VIRTUAL_ENV', 'PYTHONPATH', 'LD_LIBRARY_PATH'):
        print(f'{nom}={os.environ.get(nom, "<non défini>")}', flush=True)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.utilities import get_rmw_implementation_identifier
    except ImportError:
        traceback.print_exc()
        print('[ÉCHEC IMPORT] Vérifier le Python utilisé et le chargement de ROS.',
              file=sys.stderr)
        return 1

    print(f'[OK] rclpy importé : {rclpy.__file__}', flush=True)
    noeud = None
    try:
        print(f'RMW effectif : {get_rmw_implementation_identifier()}', flush=True)
        rclpy.init()
        noeud = Node('diagnostic_dds_fosa', enable_rosout=False,
                     start_parameter_services=False)
        print('[OK] Nœud ROS créé ; ressources DDS libérées à la sortie.', flush=True)
    except Exception:
        traceback.print_exc()
        print(
            '[ÉCHEC DDS/ROS] Les imports ont réussi, mais la création du nœud a échoué.\n'
            'Si le journal indique « Failed to find a free participant index »,\n'
            'essayer FOSA_DDS_ETENDU=1 ./run.sh diagnostic-dds.\n'
            'Si cet essai échoue aussi, examiner les processus, ports et la\n'
            'configuration CycloneDDS du robot (voir DIAGNOSTIC.md).',
            file=sys.stderr)
        return 1
    finally:
        if noeud is not None:
            noeud.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
