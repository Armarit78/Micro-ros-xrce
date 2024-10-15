#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/header.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCCHECK_VOID(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); exit(EXIT_FAILURE); }}


rcl_publisher_t ping_publisher;
std_msgs__msg__Header outcoming_ping;

int device_id;
int seq_no;
int message_count = 0; // Compteur de messages ajouté pour suivre le nombre de messages envoyés

void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) {
        seq_no = rand(); // Génère un nouveau numéro de séquence aléatoire pour chaque ping
        message_count++; // Incrémente le compteur de messages à chaque appel

        sprintf(outcoming_ping.frame_id.data, "%d_%d", seq_no, device_id);
        outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        outcoming_ping.stamp.sec = ts.tv_sec;
        outcoming_ping.stamp.nanosec = ts.tv_nsec;

        RCCHECK_VOID(rcl_publish(&ping_publisher, &outcoming_ping, NULL));
        printf("Ping #%d sent with seq %s at time %ld.%ld\n", message_count, outcoming_ping.frame_id.data, ts.tv_sec, ts.tv_nsec); // Affichage amélioré pour chaque ping envoyé
    }
}

int main()
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "ping_node", "", &support));

    RCCHECK(rclc_publisher_init_default(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), ping_timer_callback));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    char outcoming_ping_buffer[STRING_BUFFER_LEN];
    outcoming_ping.frame_id.data = outcoming_ping_buffer;
    outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

    device_id = rand();

    rclc_executor_spin(&executor);

    RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
}

