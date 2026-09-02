cmake_minimum_required(VERSION 3.8)
project(caytu_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
