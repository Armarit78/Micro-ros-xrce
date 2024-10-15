#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRING_BUFFER_LEN 256
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;} }
#define RCCHECK_VOID(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); exit(EXIT_FAILURE);} }

rcl_subscription_t lidar_subscriber;
rcl_publisher_t result_publisher;
sensor_msgs__msg__PointCloud2 incoming_data;
std_msgs__msg__String result_msg;

void lidar_subscription_callback(const void * received_data) {
    const sensor_msgs__msg__PointCloud2 * msg = (const sensor_msgs__msg__PointCloud2 *)received_data;

    if (!msg) {
        printf("Received NULL data\n");
        return;
    }

    // Imprimer les informations demandées
    printf("Données reçues sur /new_topic\n");
    printf("Header Frame ID: %s\n", msg->header.frame_id.data);
    printf("Width: %u\n", msg->width);
    printf("Height: %u\n", msg->height);
    printf("Nombre de points: %u\n", msg->width * msg->height);

    // Formater les informations dans une chaîne de caractères
    char buffer[STRING_BUFFER_LEN];
    snprintf(buffer, STRING_BUFFER_LEN, "Header Frame ID: %s\nWidth: %u\nHeight: %u\nNombre de points: %u\n",
             msg->header.frame_id.data, msg->width, msg->height, msg->width * msg->height);

    // Publier les informations formatées
    if (result_msg.data.data != NULL) {
        free(result_msg.data.data);  // Libération de la mémoire précédemment allouée
    }
    result_msg.data.data = strdup(buffer);  // Utilisation de strdup pour allouer la mémoire nécessaire
    result_msg.data.size = strlen(buffer);
    result_msg.data.capacity = result_msg.data.size + 1;

    RCCHECK_VOID(rcl_publish(&result_publisher, &result_msg, NULL));
    printf("Informations publiées sur /result_lidar\n");
}

int main() {
    printf("Initializing ROS 2 node...\n");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "rosbagpub_node", "", &support));
    printf("Node initialized.\n");

    // Initialisation du subscriber
    RCCHECK(rclc_subscription_init_default(
        &lidar_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
        "/new_topic"));

    // Initialisation du publisher
    RCCHECK(rclc_publisher_init_default(
        &result_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/result_lidar"));

    // Initialisation du message PointCloud2 pour la réception
    sensor_msgs__msg__PointCloud2__init(&incoming_data);

    // Initialisation des champs
    incoming_data.fields.size = 3;
    incoming_data.fields.capacity = 3;
    incoming_data.fields.data = allocator.allocate(incoming_data.fields.size * sizeof(sensor_msgs__msg__PointField), allocator.state);

    sensor_msgs__msg__PointField x_field;
    x_field.name.data = allocator.allocate(2 * sizeof(char), allocator.state);
    strcpy(x_field.name.data, "x");
    x_field.name.size = 1;
    x_field.name.capacity = 2;
    x_field.offset = 0;
    x_field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    x_field.count = 1;
    incoming_data.fields.data[0] = x_field;

    sensor_msgs__msg__PointField y_field;
    y_field.name.data = allocator.allocate(2 * sizeof(char), allocator.state);
    strcpy(y_field.name.data, "y");
    y_field.name.size = 1;
    y_field.name.capacity = 2;
    y_field.offset = 4;
    y_field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    y_field.count = 1;
    incoming_data.fields.data[1] = y_field;

    sensor_msgs__msg__PointField z_field;
    z_field.name.data = allocator.allocate(2 * sizeof(char), allocator.state);
    strcpy(z_field.name.data, "z");
    z_field.name.size = 1;
    z_field.name.capacity = 2;
    z_field.offset = 8;
    z_field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    z_field.count = 1;
    incoming_data.fields.data[2] = z_field;

    // Initialisation des autres champs
    incoming_data.header.frame_id.data = allocator.allocate(4 * sizeof(char), allocator.state);
    strcpy(incoming_data.header.frame_id.data, "map");
    incoming_data.header.frame_id.size = 3;
    incoming_data.header.frame_id.capacity = 4;

    incoming_data.height = 1;
    incoming_data.width = 10;
    incoming_data.is_bigendian = false;
    incoming_data.point_step = 12;
    incoming_data.row_step = incoming_data.width * incoming_data.point_step;
    incoming_data.is_dense = true;

    // Allocation du tableau de données pour PointCloud2
    incoming_data.data.capacity = incoming_data.row_step * incoming_data.height;
    incoming_data.data.size = incoming_data.data.capacity;
    incoming_data.data.data = allocator.allocate(incoming_data.data.capacity * sizeof(uint8_t), allocator.state);

    // Initialisation du message de résultat
    std_msgs__msg__String__init(&result_msg);  // Initialiser le message de type String

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &lidar_subscriber, &incoming_data, &lidar_subscription_callback, ON_NEW_DATA));

    printf("Starting executor spin...\n");
    rclc_executor_spin(&executor);
    printf("Executor spin stopped.\n");

    // Nettoyage
    sensor_msgs__msg__PointCloud2__fini(&incoming_data);
    std_msgs__msg__String__fini(&result_msg);  // Nettoyer le message de type String

    RCCHECK(rcl_subscription_fini(&lidar_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&result_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    rclc_support_fini(&support);

    return 0;
}

