cmake_minimum_required(VERSION 3.5)

project(rosbagsub_node LANGUAGES C)

# Trouver les packages nécessaires
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(std_msgs REQUIRED) # Ajouté pour std_msgs

# Ajouter l'exécutable pour le programme rosbagsub
add_executable(rosbagsub_node rosbagsub.c)

# Ajouter les dépendances
ament_target_dependencies(rosbagsub_node
  rcl
  rclc
  sensor_msgs
  std_msgs
  rmw_microxrcedds
)

# Installation de la cible
install(TARGETS rosbagsub_node
  DESTINATION lib/${PROJECT_NAME}
)

