#!/usr/bin/env bash
# À sourcer après les workspaces ROS ; ne crée aucun participant DDS.

configurer_dds() {
    case "${FOSA_DDS_ETENDU:-0}" in
        0) return 0 ;;
        1) ;;
        *) echo "FOSA_DDS_ETENDU doit valoir 0 ou 1." >&2; return 2 ;;
    esac

    local rmw="${RMW_IMPLEMENTATION:-}" fichier uri
    if [[ -z "$rmw" ]]; then
        rmw="$(python3 -c 'from rclpy.utilities import get_rmw_implementation_identifier; print(get_rmw_implementation_identifier())')" || return 1
    fi
    if [[ "$rmw" != rmw_cyclonedds_cpp ]]; then
        echo "FOSA_DDS_ETENDU concerne CycloneDDS ; RMW actif : $rmw." >&2
        return 1
    fi

    fichier="$FOSA_CONFIG_DIR/cyclonedds.xml"
    [[ -r "$fichier" ]] || {
        echo "Configuration DDS introuvable : $fichier" >&2
        return 1
    }
    uri="file://$fichier"
    # Le dernier fichier prime pour les valeurs scalaires. Ne pas découper
    # CYCLONEDDS_URI : il peut contenir du XML avec des virgules.
    case "${CYCLONEDDS_URI:-}" in
        "$uri"|*,"$uri") return 0 ;;
    esac
    export CYCLONEDDS_URI="${CYCLONEDDS_URI:+${CYCLONEDDS_URI},}$uri"
}
