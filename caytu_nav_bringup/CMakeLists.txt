cmake_minimum_required(VERSION 3.8)
project(caytu_nav_bringup)

find_package(ament_cmake REQUIRED)

# Paramètres Nav2, filtre du lidar et arbre de comportement de la course.
install(DIRECTORY config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
