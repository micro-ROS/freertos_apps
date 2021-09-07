/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_echo.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point32.h>

#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/ping.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include "radiolink.h"
#include <time.h>

#include "microrosapp.h"

#include "crtp.h"
#include "configblock.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void microros_primary(void * params);
void microros_secondary(void * params);

float sensor_data[2];
sensor_msgs__msg__LaserEcho sensor_topic;
static bool sensor_data_ready;

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

// Note: please set APP_STACKSIZE = 100 and CFLAGS += -DFREERTOS_HEAP_SIZE=12100 in Makefile before build

STATIC_MEM_TASK_ALLOC(microros_primary, 1000);
STATIC_MEM_TASK_ALLOC(microros_secondary, 1000);
static bool created_primary = false;

void appMain(){ 
    // TaskHandle_t task_primary, task_secondary;
    absoluteUsedMemory = 0;
    usedMemory = 0;

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = __crazyflie_allocate;
    freeRTOS_allocator.deallocate = __crazyflie_deallocate;
    freeRTOS_allocator.reallocate = __crazyflie_reallocate;
    freeRTOS_allocator.zero_allocate = __crazyflie_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        DEBUG_PRINT("Error on default allocators (line %d)\n",__LINE__); 
        vTaskSuspend( NULL );
    }

    sensor_topic.echoes.capacity = 2;
    sensor_topic.echoes.size = 2;
    sensor_topic.echoes.data = sensor_data;

    STATIC_MEM_TASK_CREATE(microros_primary, microros_primary, "microROSprimary", NULL, 3);
    STATIC_MEM_TASK_CREATE(microros_secondary, microros_secondary, "microROSsecondary", NULL, 3);
}

typedef struct rmw_init_options_impl_t rmw_init_options_impl_t;

struct  rmw_init_options_impl_t
{
  struct rmw_uxrce_transport_params_t transport_params;
};

void microros_primary(void * params){
    while(1){

        // ####################### MICROROS INIT #######################
        DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
        vTaskDelay(50);

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));
        rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

        transport_args secondary_args = { .radio_channel = 65, .radio_port = 10, .initialized = false };
        RCCHECK(rmw_uros_options_set_custom_transport(
            true,
            (void *) &secondary_args,
            crazyflie_serial_open,
            crazyflie_serial_close,
            crazyflie_serial_write,
            crazyflie_serial_read,
            rmw_options));
        
        // Wait for agent connection
        while(RMW_RET_OK != rmw_uros_ping_agent_options(50, 100, rmw_options))
        {
            vTaskDelay(250/portTICK_RATE_MS);
        }

        rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        // create node
        rcl_node_t node;
        RCCHECK(rclc_node_init_default(&node, "crazyflie_node_1", "", &support));

        created_primary = true;

        // Create publisher 1
        rcl_publisher_t pub_sensors_temp;
        RCCHECK(rclc_publisher_init_best_effort(
            &pub_sensors_temp, 
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
            "/weather_station/temperature"));

        // Create publisher 2
        rcl_publisher_t pub_sensors_hum;
        RCCHECK(rclc_publisher_init_best_effort(
            &pub_sensors_hum, 
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
            "/weather_station/humidity"));

        // Create publisher 3
        rcl_publisher_t pub_odom;
        RCCHECK(rclc_publisher_init_best_effort(
            &pub_odom, 
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/drone/odometry"));

        // Create publisher 4
        rcl_publisher_t pub_attitude;
        RCCHECK(rclc_publisher_init_best_effort(
            &pub_attitude, 
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/drone/attitude"));

        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        created_primary = true;

        // ####################### MAIN LOOP #######################

        // Init messages 
        // TODO: use micro ros utilities
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

        while(1)
        {
            if(sensor_data_ready)
            {
                std_msgs__msg__Float32 aux_msg;

                aux_msg.data = sensor_topic.echoes.data[0];
                RCSOFTCHECK(rcl_publish( &pub_sensors_temp, &aux_msg, NULL));

                aux_msg.data = sensor_topic.echoes.data[1];
                RCSOFTCHECK(rcl_publish( &pub_sensors_hum, &aux_msg, NULL));

                sensor_data_ready = 0;
            }

            pose.x = logGetFloat(pitchid);
            pose.y = logGetFloat(rollid);
            pose.z = logGetFloat(yawid);
            odom.x = logGetFloat(Xid);
            odom.y = logGetFloat(Yid);
            odom.z = logGetFloat(Zid);

            RCSOFTCHECK(rcl_publish( &pub_attitude, &pose, NULL));
            RCSOFTCHECK(rcl_publish( &pub_odom, &odom, NULL));

            vTaskDelay(100/portTICK_RATE_MS);
        }
    }

    vTaskSuspend( NULL );
}

void subscription_callback(const void * msgin)
{
	const sensor_msgs__msg__LaserEcho * msg = (const sensor_msgs__msg__LaserEcho *) msgin;
	DEBUG_PRINT("Received data from secondary agent\n");

    // TODO: add mutex to avoid multiple read/write?
    memcpy(&sensor_topic, msg, sizeof(sensor_msgs__msg__LaserEcho));
    sensor_data_ready = 1;
}

void microros_secondary(void * params){

    while(!created_primary){
        vTaskDelay(100/portTICK_RATE_MS);
    }

    while(1){
        // ####################### MICROROS INIT #######################

        DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
        vTaskDelay(50);

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));
        rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

        transport_args secondary_args = { .radio_channel = 30, .radio_port = 9, .initialized=false };
        RCCHECK(rmw_uros_options_set_custom_transport(
            true,
            (void *) &secondary_args,
            crazyflie_serial_open,
            crazyflie_serial_close,
            crazyflie_serial_write,
            crazyflie_serial_read,
            rmw_options));

        while(RMW_RET_OK != rmw_uros_ping_agent_options(50, 100, rmw_options))
        {
            vTaskDelay(250/portTICK_RATE_MS);
        }

        rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        // create node
        rcl_node_t node;
        RCCHECK(rclc_node_init_default(&node, "crazyflie_node_2", "", &support));

        // Create subscription 2
        rcl_subscription_t sub_sensors;
        sensor_msgs__msg__LaserEcho msg;
        RCCHECK(rclc_subscription_init_best_effort(
            &sub_sensors,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserEcho),
            "Float__Sequence"));

        // Create wait set
        // create executor and add subscription
        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_add_subscription(&executor, &sub_sensors, &msg, &subscription_callback, ON_NEW_DATA));

        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        // ####################### MAIN LOOP #######################


        while(1)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            vTaskDelay(100/portTICK_RATE_MS);
        }
    }

    vTaskSuspend( NULL );
}
