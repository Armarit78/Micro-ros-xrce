# README pour les Tests des Programmes micro-ROS

# I/ Programme Ping et Pong

## I. Tests des programmes micro-ROS sur la même machine

Cette section guide à travers les étapes pour tester les noeuds micro-ROS "ping" et "pong" sur une seule machine.

### Préparation de l'environnement de test

1. **Préparation des répertoires** :
    - Place les répertoires "ping" et "pong" ainsi que le fichier CMake global dans `microros_ws/src/uros/micro-ROS-demos/rclc`.

2. **Compilation** :
    - À chaque modification de l'environnement de travail dans `micro_ws`, exécute les commandes suivantes pour recompiler le firmware :

    ```bash
    source install/local_setup.bash
    ros2 run micro_ros_setup build_firmware.sh
    source install/local_setup.bash
    ```

- **Note :** Faites-le deux fois pour être sûr que les warnings de compilation disparaissent bien.

### Lancement des tests

Pour tester les interactions entre les noeuds "ping" et "pong", ouvre trois terminaux et exécute les commandes suivantes :

**Note :** Toutes les commandes suivantes doivent être exécutées depuis le répertoire microros_ws.

#### Terminal 1 : Démarrage de l'agent micro-ROS

```bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Ceci lance l'agent micro-ROS qui permet la communication entre les noeuds micro-ROS et le reste de l'espace de données ROS 2.

#### Terminal 2 : Exécution du noeud Ping

```bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc ping_node
```

Ce terminal lance le noeud "ping", qui enverra des messages ping.

#### Terminal 3 : Exécution du noeud Pong

```bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc pong_node
```

Ce terminal lance le noeud "pong", qui écoute les messages ping et répond avec des messages pong.

## II. Tests des programmes micro-ROS sur deux machines

Cette section explique comment configurer et tester les interactions micro-ROS entre deux machines distinctes, l'une agissant comme client et l'autre comme agent.
Configuration du réseau

### Préparation de l'environnement de test

1. **Création de la connexion pont entre les deux machines** :

    Définir les adresses IP statiques :

        Machine client (exécutant le noeud micro-ROS) :
            IP : 169.0.2.18
            Masque de sous-réseau : 255.255.255.0

        Machine agent (exécutant l'agent micro-ROS) :
            IP : 169.0.2.19
            Masque de sous-réseau : 255.255.255.0

2. **Modification de la configuration micro-ROS** 

Pour permettre au noeud micro-ROS sur la machine client de communiquer avec l'agent sur la machine agent, modifiez le fichier de métadonnées de colcon pour configurer le transport UDP approprié.

Modifier le fichier colcon.meta :     Emplacement : `microros_ws/src/`
    
Ajoutez ou modifiez les arguments CMake pour le transport micro XRCE-DDS comme suit :

```json
    "rmw_microxrcedds": {
        "cmake-args": [
            "-DRMW_UXRCE_TRANSPORT=udp",
            "-DRMW_UXRCE_DEFAULT_UDP_IP=169.0.2.19",
            "-DRMW_UXRCE_DEFAULT_UDP_PORT=8888",
            "-DRMW_UXRCE_MAX_NODES=15",
            "-DRMW_UXRCE_MAX_PUBLISHERS=15",
            "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=15",
            "-DRMW_UXRCE_MAX_SERVICES=15",
            "-DRMW_UXRCE_MAX_CLIENTS=15",
            "-DRMW_UXRCE_STREAM_HISTORY=32",
            "-DRMW_UXRCE_MAX_HISTORY=10"
        ]
    }
```

**Note :** Pensez à recompiler l'environnement de travail après chaque modification en exécutant les commandes de compilation appropriées.


### Configuration des Machines

- Machine Agent : Cette machine exécutera à la fois l'agent micro-ROS et le noeud ping.
    
- Machine Client : Cette machine exécutera le noeud pong.

### Surveillance des interactions

- Sur la Machine Agent : Vous devriez voir des messages indiquant que des pings sont envoyés par le noeud ping.

- Sur la Machine Client : Le terminal exécutant le noeud pong devrait afficher des messages indiquant qu'il reçoit des pings et envoie des réponses pong.

--------------------------------------

# II/ Programme Rosbagsub et Rosbagpub

Cette section contient des exemples de noeuds ROS 2 pour la souscription (`rosbagsub_node`) et la publication (`rosbagpub_node`) de messages `PointCloud2` à l'aide de micro-ROS.

## Configuration

**Note :** Toutes les commandes suivantes doivent être exécutées depuis le répertoire microros_ws.

### Publication des données avec un fichier YAML

Pour publier des données PointCloud2 sur le topic /perf/points à partir d'un fichier YAML, utilisez la commande suivante :

```bash
source install/local_setup.bash
cat test.yaml | xargs -0 ros2 topic pub /perf/points sensor_msgs/msg/PointCloud2
```

#### Lancement de l'agent micro-ROS:

Pour lancer l'agent micro-ROS en utilisant le transport UDP sur le port 8888, utilisez la commande suivante :

```bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### Exécution du noeud de souscription

Pour exécuter le noeud de souscription rosbagsub_node, qui souscrit aux messages PointCloud2 sur le topic /perf/points, configurez l'implémentation RMW et lancez le noeud :

```bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc rosbagsub_node
```

#### Exécution du noeud de publication

Pour exécuter le noeud de publication rosbagpub_node, qui publie des messages formatés sur le topic /result_lidar, configurez l'implémentation RMW et lancez le noeud :

```bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_microxrcedds
ros2 run micro_ros_demos_rclc rosbagpub_node
```

### Publication des données avec un Rosbag et Plotjoggler

#### Allocation manuelle de la mémoire pour les messages ROS

Pour recevoir et publier des messages ROS de type `sensor_msgs/PointCloud2`, il est nécessaire d'allouer manuellement la mémoire pour les différents champs du message. 
Voici un exemple de configuration des paramètres essentiels de `PointCloud2` :

```c
// Allouer manuellement la mémoire pour le message PointCloud2
incoming_data.height = 1;  // Nombre de lignes de points
incoming_data.width = 502;  // Nombre de points par ligne
incoming_data.is_bigendian = false;  // Définir si les données sont en big-endian
incoming_data.point_step = 12;  // Nombre d'octets entre les points dans une ligne
incoming_data.row_step = incoming_data.width * incoming_data.point_step;  // Nombre d'octets dans une ligne de données
incoming_data.is_dense = true;  // Indiquer si tous les points sont valides
```

Ces paramètres doivent être définis pour chaque message PointCloud2 que vous recevez afin de garantir une allocation de mémoire correcte et éviter des erreurs de segmentation.

#### Téléchargement et utilisation de PlotJuggler

PlotJuggler est un outil puissant pour visualiser et manipuler les données de vos topics ROS. Il permet d'extraire des topics d'un Rosbag, de moduler leur vitesse et de les republier.

Voici comment télécharger et utiliser PlotJuggler :

```bash
sudo snap install plotjuggler
```

#### Utilisation de PlotJuggler avec un Rosbag :

Ouvrez PlotJuggler en exécutant la commande suivante dans un terminal :

```bash
plotjuggler
```

Pour charger un Rosbag, cliquer sur Data : "load data from file", selectioner votre rosbag (subset) et ouvrez le .yaml, ensuite selectioner les topics qui vous interesse (ici /perf:points).
Vous pouver ensuite augmenter ou diminuer leurs vitesse et noublier pas de cocher la case "ROS2 Topic republisher" pour publier votre topic.
