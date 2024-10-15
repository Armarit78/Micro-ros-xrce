cmake_minimum_required(VERSION 3.5)

project(ping_node LANGUAGES C)

# Trouver les packages nécessaires
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)

# Ajouter l'exécutable pour le programme ping
add_executable(ping_node ping.c)

# Ajouter les dépendances
ament_target_dependencies(ping_node
  rcl
  rclc
  std_msgs
  builtin_interfaces
  rmw_microxrcedds
)

# Installation de la cible
install(TARGETS ping_node
  DESTINATION lib/${PROJECT_NAME}
)

