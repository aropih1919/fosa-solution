#!/usr/bin/env python3
"""Enregistre la carte en cours : ros2 run caytu_nav_solution save_map

Appelle le service /fosa/save_map de mapping_session (lancé par
mapping.launch.py) et affiche où la carte a été écrite.
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None):
    rclpy.init(args=args)
    node = Node('fosa_save_map')
    client = node.create_client(Trigger, '/fosa/save_map')
    code = 1
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            print('Service /fosa/save_map introuvable : mapping.launch.py tourne-t-il ?',
                  file=sys.stderr)
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=40.0)
        result = future.result() if future.done() else None
        if result is None:
            print('Pas de réponse de mapping_session.', file=sys.stderr)
            return
        print(result.message)
        code = 0 if result.success else 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(code)


if __name__ == '__main__':
    main()
