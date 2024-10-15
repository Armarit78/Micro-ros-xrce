#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/header.h>
#include <stdio.h>
#include <string.h>

#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCCHECK_VOID(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);exit(EXIT_FAILURE);}}

rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
std_msgs__msg__Header incoming_ping;

void ping_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

    // Créer une copie modifiée du message ping pour le renvoyer en tant que pong
    std_msgs__msg__Header pong_msg;
    memcpy(&pong_msg, msg, sizeof(std_msgs__msg__Header));

    // Ajouter des informations supplémentaires au frame_id pour indiquer qu'il a été traité
    char new_frame_id[STRING_BUFFER_LEN];  // Création d'une nouvelle variable pour éviter tout débordement
    snprintf(new_frame_id, STRING_BUFFER_LEN, "pong_%s", msg->frame_id.data);
    strncpy(pong_msg.frame_id.data, new_frame_id, STRING_BUFFER_LEN);
    pong_msg.frame_id.size = strlen(pong_msg.frame_id.data);

    printf("Ping received with seq %s. Answering with pong, modified seq %s.\n", msg->frame_id.data, pong_msg.frame_id.data);

    // Publier le message pong modifié
    RCCHECK_VOID(rcl_publish(&pong_publisher, &pong_msg, NULL));
}

int main()
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "pong_node", "", &support));

    RCCHECK(rclc_publisher_init_default(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));
    RCCHECK(rclc_subscription_init_best_effort(&ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));

    char incoming_ping_buffer[STRING_BUFFER_LEN];
    incoming_ping.frame_id.data = incoming_ping_buffer;
    incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

    rclc_executor_spin(&executor);

    RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
    RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
}

