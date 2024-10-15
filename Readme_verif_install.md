# Testing ROS 2 Iron on Ubuntu 22.04

## Preliminary Setup
Before testing, ensure that you have ROS 2 Iron installed. If you have not installed it yet, refer to the installation section.

## Running Example Nodes
ROS 2 Iron includes several demonstration nodes that you can use to verify the installation and functionality. These examples demonstrate basic ROS 2 concepts such as publishing and subscribing to topics.

### Setup Environment
Before running the examples, you need to source the ROS 2 environment setup script in each terminal session. This script sets up the necessary environment variables for ROS 2:

```bash
source /opt/ros/iron/setup.bash
```

- Running the Talker (C++)

The C++ talker is a simple publisher node that continuously broadcasts a message.

Open a new terminal window.

```bbash
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_cpp talker
```

This node will publish messages to the topic /chatter.
- Running the Listener (Python)

Open another terminal window.

```bash
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py listener
```

This node will receive and print messages published by the talker.

# Test de l'application micro-ROS

Ce guide détaille comment démarrer l'agent micro-ROS, exécuter un nœud micro-ROS, et tester son interaction avec des sujets ROS 2. Ces étapes garantissent que l'installation et la configuration ont été réalisées correctement et que l'application micro-ROS fonctionne comme prévu dans un environnement simulé.

## 1. Démarrage de l'agent micro-ROS
L'agent micro-ROS permet la communication entre le nœud micro-ROS et l'espace de données ROS 2. Pour le démarrer sur votre système hôte :

```bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

2. Exécution du nœud micro-ROS

Pour exécuter le nœud micro-ROS :

```bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc ping_pong
```

3. Test de l'application

Dans un nouveau terminal : Écoute du sujet ping

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic echo /microROS/ping
```

Vous devriez voir des messages du sujet publiés toutes les 5 secondes.

Dans un nouveau terminal : Tester la capacité de réponse du nœud micro-ROS aux pings

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic echo /microROS/pong
```

Dans un nouveau terminal : Publiez un faux ping pour stimuler une réponse :

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Surveillez les deux terminaux (ping et pong). Vous devriez voir le faux ping dans la console du souscripteur ping et, en conséquence, le nœud micro-ROS devrait répondre avec un pong que vous verrez dans la console pong.
