FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Les manifestes seuls permettent de garder les dépendances en cache quand
# le code change. Le code testé sera copié au lancement du conteneur.
COPY caytu_config/package.xml /dependances/caytu_config/package.xml
COPY caytu_course/package.xml /dependances/caytu_course/package.xml
COPY caytu_interfaces/package.xml /dependances/caytu_interfaces/package.xml
COPY caytu_nav_bringup/package.xml /dependances/caytu_nav_bringup/package.xml
COPY caytu_nav_solution/package.xml /dependances/caytu_nav_solution/package.xml
COPY caytu_perception/package.xml /dependances/caytu_perception/package.xml
COPY simulation/parc_msgs/package.xml /dependances/parc_msgs/package.xml

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-pytest \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /dependances --ignore-src \
       --rosdistro jazzy --skip-keys ament_python -y \
    && rm -rf /var/lib/apt/lists/*

COPY docker/tests/executer.sh /opt/fosa-tests/executer.sh
COPY docker/tests/test_installation_ros.py /opt/fosa-tests/test_installation_ros.py

ENV HOME=/tmp/fosa-home
WORKDIR /tmp
ENTRYPOINT ["/bin/bash", "/opt/fosa-tests/executer.sh"]
