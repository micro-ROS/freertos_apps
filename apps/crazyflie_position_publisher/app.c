#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point32.h>
#include <micro_ros_utilities/type_utilities.h>

#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/ping.h>

#include "config.h"
#include "log.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include <time.h>

#include "microrosapp.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static uint8_t crtp_buffer[CRTP_BUFFER_SIZE];

rcl_publisher_t publisher_odometry;
rcl_publisher_t publisher_attitude;

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

float sign(float x){
    return (x >= 0) ? 1.0 : -1.0;
}

void appMain(){
    absoluteUsedMemory = 0;
    usedMemory = 0;

    // ####################### MICROROS INIT #######################
    DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
    vTaskDelay(50);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = __crazyflie_allocate;
    freeRTOS_allocator.deallocate = __crazyflie_deallocate;
    freeRTOS_allocator.reallocate = __crazyflie_reallocate;
    freeRTOS_allocator.zero_allocate = __crazyflie_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        DEBUG_PRINT("Error on default allocators (line %d)\n",__LINE__);
        vTaskSuspend( NULL );
    }

    transport_args custom_args = { .radio_channel = 65, .radio_port = 9, .crtp_buffer = &crtp_buffer[0] };
    rmw_uros_set_custom_transport( 
        true, 
        (void *) &custom_args, 
        crazyflie_serial_open, 
        crazyflie_serial_close, 
        crazyflie_serial_write, 
        crazyflie_serial_read
    ); 

    // Wait for available agent
    while(RMW_RET_OK != rmw_uros_ping_agent(1000, 10))
    {
        vTaskDelay(100/portTICK_RATE_MS);
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "crazyflie_node", "", &support));

	// create publishers
	RCCHECK(rclc_publisher_init_best_effort(&publisher_odometry, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/drone/odometry"));
	RCCHECK(rclc_publisher_init_best_effort(&publisher_attitude, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/drone/attitude"));

    // Init messages
    geometry_msgs__msg__Point32 pose;
    geometry_msgs__msg__Point32 odom;

    static micro_ros_utilities_memory_conf_t conf = {0};

    bool success = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
        &pose,
        conf);

    success &= micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
        &odom,
        conf);

    if (!success)
    {
        DEBUG_PRINT("Memory allocation for /drone messages failed\n");
        return;
    }

    //Get pitch, roll and yaw value
    pitchid = logGetVarId("stateEstimate", "pitch");
    rollid = logGetVarId("stateEstimate", "roll");
    yawid = logGetVarId("stateEstimate", "yaw");

    //Get X,Y and Z value
    Xid = logGetVarId("stateEstimate", "x");
    Yid = logGetVarId("stateEstimate", "y");
    Zid = logGetVarId("stateEstimate", "z");

    DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
    DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
    DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

	while(1){
        pose.x = logGetFloat(pitchid);
        pose.y = logGetFloat(rollid);
        pose.z = logGetFloat(yawid);
        odom.x = logGetFloat(Xid);
        odom.y = logGetFloat(Yid);
        odom.z = logGetFloat(Zid);

        RCSOFTCHECK(rcl_publish( &publisher_attitude, (const void *) &pose, NULL));
        RCSOFTCHECK(rcl_publish( &publisher_odometry, (const void *) &odom, NULL));

        vTaskDelay(10/portTICK_RATE_MS);
	}

    success = micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
        &pose,
        conf
    );

    success &= micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
        &odom,
        conf
    );

    if (!success)
    {
        DEBUG_PRINT("Memory release for /drone messages failed\n");
        return;
    }

	RCCHECK(rcl_publisher_fini(&publisher_attitude, &node))
	RCCHECK(rcl_publisher_fini(&publisher_odometry, &node))
	RCCHECK(rcl_node_fini(&node))

    vTaskSuspend( NULL );
}
