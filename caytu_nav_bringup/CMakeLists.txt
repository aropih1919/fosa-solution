cmake_minimum_required(VERSION 3.8)
project(caytu_nav_bringup)

find_package(ament_cmake REQUIRED)

# Configuration, launch, behavior tree et carte : tout est lu depuis le dossier
# share du package installé (aucun chemin absolu dans la solution).
install(DIRECTORY config launch behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_config test/test_navigation_config.py)
endif()

ament_package()
