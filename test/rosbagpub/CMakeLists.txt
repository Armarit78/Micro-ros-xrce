cmake_minimum_required(VERSION 3.5)

project(rosbagpub_node LANGUAGES C)

# Trouver les packages nécessaires
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)

# Ajouter l'exécutable pour le programme rosbagpub
add_executable(rosbagpub_node rosbagpub.c)

# Ajouter les dépendances
ament_target_dependencies(rosbagpub_node
  rcl
  rclc
  sensor_msgs
  std_msgs
  builtin_interfaces
  rmw_microxrcedds
)

# Installation de la cible
install(TARGETS rosbagpub_node
  DESTINATION lib/${PROJECT_NAME}
)

