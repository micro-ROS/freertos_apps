#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point32.h>

#include <rcutils/allocator.h>
#include <rmw_uros/options.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include <time.h>

#include "microrosapp.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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

    // ####################### RADIO INIT #######################

    vTaskDelay(2000);
    int radio_connected = logGetVarId("radio", "isConnected");
    while(!logGetUint(radio_connected)) vTaskDelay(100);
    DEBUG_PRINT("Radio connected\n");

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

    const uint8_t radio_channel = 65;
    rmw_uros_set_custom_transport( 
        true, 
        (void *) &radio_channel, 
        crazyflie_serial_open, 
        crazyflie_serial_close, 
        crazyflie_serial_write, 
        crazyflie_serial_read
    ); 

    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "crazyflie_node", "", &support));

	// create publishers
    // TODO (pablogs9): these publishers must be best effort
	RCCHECK(rclc_publisher_init_default(&publisher_odometry, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/drone/odometry"));
	RCCHECK(rclc_publisher_init_default(&publisher_attitude, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/drone/attitude"));

    // // Init messages
    geometry_msgs__msg__Point32 pose;
    geometry_msgs__msg__Point32__init(&pose);
    geometry_msgs__msg__Point32 odom;
    geometry_msgs__msg__Point32__init(&odom);

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
        pose.x     = logGetFloat(pitchid);
        pose.y     = logGetFloat(rollid);
        pose.z     = logGetFloat(yawid);
        odom.x     = logGetFloat(Xid);
        odom.y     = logGetFloat(Yid);
        odom.z     = logGetFloat(Zid);

        RCSOFTCHECK(rcl_publish( &publisher_attitude, (const void *) &pose, NULL));

        RCSOFTCHECK(rcl_publish( &publisher_odometry, (const void *) &odom, NULL));

        vTaskDelay(10/portTICK_RATE_MS);
	}

	RCCHECK(rcl_publisher_fini(&publisher_attitude, &node))
	RCCHECK(rcl_publisher_fini(&publisher_odometry, &node))
	RCCHECK(rcl_node_fini(&node))

    vTaskSuspend( NULL );
}