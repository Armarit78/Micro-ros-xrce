#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;} }
#define RCCHECK_VOID(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); exit(EXIT_FAILURE);} }

rcl_publisher_t lidar_publisher;
rcl_subscription_t lidar_subscriber;
sensor_msgs__msg__PointCloud2 incoming_data;

void lidar_subscription_callback(const void * received_data) {
    const sensor_msgs__msg__PointCloud2 *msg = (const sensor_msgs__msg__PointCloud2 *)received_data;

    if (!msg) {
        printf("Received NULL data\n");
        return;
    }

    printf("Received PointCloud2 data\n");
    printf("  Frame ID: %s\n", msg->header.frame_id.data);
    printf("  Height: %u, Width: %u\n", msg->height, msg->width);
    printf("  Data: %u\n", msg->data.data[0]);
    printf("  Data_size: %lu\n", msg->data.size);

    // Copy received message to incoming_data
    incoming_data.header = msg->header;
    incoming_data.height = msg->height;
    incoming_data.width = msg->width;
    incoming_data.is_bigendian = msg->is_bigendian;
    incoming_data.point_step = msg->point_step;
    incoming_data.row_step = msg->row_step;
    incoming_data.is_dense = msg->is_dense;

    // Copy fields
    if (incoming_data.fields.capacity < msg->fields.size) {
        incoming_data.fields.data = realloc(incoming_data.fields.data, msg->fields.size * sizeof(sensor_msgs__msg__PointField));
        incoming_data.fields.capacity = msg->fields.size;
    }
    incoming_data.fields.size = msg->fields.size;
    for (size_t i = 0; i < msg->fields.size; ++i) {
        incoming_data.fields.data[i] = msg->fields.data[i];
    }

    // Ensure incoming_data has enough capacity for the data
    if (incoming_data.data.capacity < msg->data.size) {
        incoming_data.data.data = realloc(incoming_data.data.data, msg->data.size);
        incoming_data.data.capacity = msg->data.size;
    }
    incoming_data.data.size = msg->data.size;
    memcpy(incoming_data.data.data, msg->data.data, msg->data.size);

    // Publish the data
    rcl_ret_t ret = rcl_publish(&lidar_publisher, &incoming_data, NULL);
    if (ret != RCL_RET_OK) {
        printf("Failed to publish data: %s\n", rcl_get_error_string().str);
        rcl_reset_error();
    } else {
        printf("Data published on /new_topic\n");
    }
}

int main() {
    printf("Initializing ROS 2 node...\n");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "rosbagsub_node", "", &support));
    printf("Node initialized.\n");

    // Initialize PointCloud2 message
    sensor_msgs__msg__PointCloud2__init(&incoming_data);

    // Initialize fields array
    incoming_data.fields.size = 3;
    incoming_data.fields.capacity = 3;
    incoming_data.fields.data = allocator.allocate(incoming_data.fields.size * sizeof(sensor_msgs__msg__PointField), allocator.state);

    // Fill in the fields
    sensor_msgs__msg__PointField x_field;
    sensor_msgs__msg__PointField__init(&x_field);
    x_field.name.data = allocator.allocate(2 * sizeof(char), allocator.state);
    strcpy(x_field.name.data, "x");
    x_field.name.size = 1;
    x_field.name.capacity = 2;
    x_field.offset = 0;
    x_field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    x_field.count = 1;
    incoming_data.fields.data[0] = x_field;

    sensor_msgs__msg__PointField y_field;
    sensor_msgs__msg__PointField__init(&y_field);
    y_field.name.data = allocator.allocate(2 * sizeof(char), allocator.state);
    strcpy(y_field.name.data, "y");
    y_field.name.size = 1;
    y_field.name.capacity = 2;
    y_field.offset = 4;
    y_field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    y_field.count = 1;
    incoming_data.fields.data[1] = y_field;

    sensor_msgs__msg__PointField z_field;
    sensor_msgs__msg__PointField__init(&z_field);
    z_field.name.data = allocator.allocate(2 * sizeof(char), allocator.state);
    strcpy(z_field.name.data, "z");
    z_field.name.size = 1;
    z_field.name.capacity = 2;
    z_field.offset = 8;
    z_field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    z_field.count = 1;
    incoming_data.fields.data[2] = z_field;

    // Set other PointCloud2 fields
    incoming_data.header.frame_id.data = allocator.allocate(4 * sizeof(char), allocator.state);
    strcpy(incoming_data.header.frame_id.data, "map");
    incoming_data.header.frame_id.size = 3;
    incoming_data.header.frame_id.capacity = 4;
 
 
    incoming_data.height = 1;
    incoming_data.width = 10;
    incoming_data.is_bigendian = false;
    incoming_data.point_step = 12;
    incoming_data.row_step = incoming_data.width * incoming_data.point_step ;
    incoming_data.is_dense = true;

    // Allocate data array for point cloud
    incoming_data.data.capacity = incoming_data.row_step * incoming_data.height;
    incoming_data.data.size = incoming_data.data.capacity;
    incoming_data.data.data = allocator.allocate(incoming_data.data.capacity * sizeof(uint8_t), allocator.state);

    // Verification of the message type
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2);
    if (type_support == NULL) {
        printf("Type support for PointCloud2 is not found.\n");
        return 1;
    } else {
        printf("Type support for PointCloud2 is valid.\n");
    }

    // Set up publisher
    printf("Initializing publisher...\n");
    RCCHECK(rclc_publisher_init_default(&lidar_publisher, &node, type_support, "/new_topic"));
    printf("Publisher initialized.\n");

    // Set up subscriber
    printf("Initializing subscriber...\n");
    RCCHECK(rclc_subscription_init_default(&lidar_subscriber, &node, type_support, "/perf/points"));
    printf("Subscriber initialized.\n");

    // Executor setup
    printf("Initializing executor...\n");
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &lidar_subscriber, &incoming_data, lidar_subscription_callback, ON_NEW_DATA));
    printf("Executor initialized.\n");

    // Spin
    printf("Starting executor spin...\n");
    rclc_executor_spin(&executor);
    printf("Executor spin stopped.\n");

    // Cleanup
    sensor_msgs__msg__PointCloud2__fini(&incoming_data);
    RCCHECK(rcl_publisher_fini(&lidar_publisher, &node));
    RCCHECK(rcl_subscription_fini(&lidar_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    return 0;
}

