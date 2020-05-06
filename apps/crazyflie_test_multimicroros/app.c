/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"

#include <rcl/rcl.h>
#include <sensor_msgs/msg/laser_echo.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point32.h>

#include <rcutils/allocator.h>
#include <rmw_uros/options.h>


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

#define RCCHECK(clean) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); goto clean;}
#define RCSOFTCHECK() if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}

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
    BaseType_t rc __attribute__((unused));
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

void microros_primary(void * params){
    while(1){

        // ####################### RADIO INIT #######################

        int radio_connected = logGetVarId("radio", "isConnected");
        while(!logGetUint(radio_connected)) vTaskDelay(100);
        // DEBUG_PRINT("Radio connected\n");

        // ####################### MICROROS INIT #######################

        DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
        vTaskDelay(50);

        rcl_context_t      context;
        rcl_init_options_t init_options;
        rcl_ret_t          rc;
        rcl_ret_t          rc_aux __attribute__((unused));

        init_options = rcl_get_zero_initialized_init_options();                          
        rc = rcl_init_options_init(&init_options, rcutils_get_default_allocator());      
        RCCHECK(clean1)

        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        rc = rmw_uros_options_set_serial_device("65", rmw_options);
        rc = rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options);

        context = rcl_get_zero_initialized_context();                                    

        rc = rcl_init(0, NULL, &init_options, &context);
        rc_aux = rcl_init_options_fini(&init_options);
        RCCHECK(clean1)

        rcl_node_t node = rcl_get_zero_initialized_node();
        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, "crazyflie_node_1", "", &context, &node_ops);
        RCCHECK(clean1)

        // Create publisher 1
        rcl_publisher_t pub_sensors_temp        = rcl_get_zero_initialized_publisher();
        rcl_publisher_options_t pub_opt_sensors_temp = rcl_publisher_get_default_options();
        pub_opt_sensors_temp.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_publisher_init(
            &pub_sensors_temp,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/weather_station/temperature",
            &pub_opt_sensors_temp);

        if(rc != RCL_RET_OK){
            rc_aux = rcl_node_fini(&node);
        }
        RCCHECK(clean1)

        // Create publisher 2
        rcl_publisher_t pub_sensors_hum        = rcl_get_zero_initialized_publisher();
        rcl_publisher_options_t pub_opt_sensors_hum = rcl_publisher_get_default_options();
        pub_opt_sensors_hum.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_publisher_init(
            &pub_sensors_hum,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/weather_station/humidity",
            &pub_opt_sensors_hum);

        if(rc != RCL_RET_OK){
            rc_aux = rcl_node_fini(&node);
        }
        RCCHECK(clean1)

        // Create publisher 3
        rcl_publisher_t pub_odom        = rcl_get_zero_initialized_publisher();
        rcl_publisher_options_t pub_opt_odom = rcl_publisher_get_default_options();
        pub_opt_odom.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_publisher_init(
            &pub_odom,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/drone/odometry",
            &pub_opt_odom);
        RCCHECK(clean1)

        // Create publisher 4
        rcl_publisher_t pub_attitude        = rcl_get_zero_initialized_publisher();
        rcl_publisher_options_t pub_opt_att = rcl_publisher_get_default_options();
        pub_opt_att.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_publisher_init(
            &pub_attitude,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
            "/drone/attitude",
            &pub_opt_att);
        RCCHECK(clean1)

        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        created_primary = true;
        // ####################### MAIN LOOP #######################

        // Init messages 
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

        while(1){

            if(sensor_data_ready){
                std_msgs__msg__Float32 aux_msg;

                aux_msg.data = sensor_topic.echoes.data[0];
                rc = rcl_publish( &pub_sensors_temp, (const void *) &aux_msg, NULL);

                aux_msg.data = sensor_topic.echoes.data[1];
                rc = rcl_publish( &pub_sensors_hum, (const void *) &aux_msg, NULL);

                sensor_data_ready = 0;
            }

            pose.x     = logGetFloat(pitchid);
            pose.y     = logGetFloat(rollid);
            pose.z     = logGetFloat(yawid);
            odom.x     = logGetFloat(Xid);
            odom.y     = logGetFloat(Yid);
            odom.z     = logGetFloat(Zid);

            rc = rcl_publish( &pub_attitude, (const void *) &pose, NULL);
            RCSOFTCHECK()

            rc = rcl_publish( &pub_odom, (const void *) &odom, NULL);
            RCSOFTCHECK()

            vTaskDelay(100/portTICK_RATE_MS);
        }

        rc = rcl_node_fini(&node);
clean1:     
        rc = rcl_shutdown(&context);
        DEBUG_PRINT("Connection lost on primary, retriying\n");
    }

    vTaskSuspend( NULL );
}

void microros_secondary(void * params){
    while(!created_primary){
        vTaskDelay(100);
    }

    while(1){
        // ####################### RADIO INIT #######################

        int radio_connected = logGetVarId("radio", "isConnected");
        while(!logGetUint(radio_connected)) vTaskDelay(100);
        // DEBUG_PRINT("Radio connected\n");

        // ####################### MICROROS INIT #######################

        DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
        vTaskDelay(50);

        rcl_context_t      context;
        rcl_init_options_t init_options;
        rcl_ret_t          rc;
        rcl_ret_t          rc_aux __attribute__((unused));

        init_options = rcl_get_zero_initialized_init_options();                          
        rc = rcl_init_options_init(&init_options, rcutils_get_default_allocator());      
        RCCHECK(clean2)

        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        rc = rmw_uros_options_set_serial_device("30", rmw_options);
        rc = rmw_uros_options_set_client_key(0xDEADBEEF, rmw_options);

        context = rcl_get_zero_initialized_context();                                    

        rc = rcl_init(0, NULL, &init_options, &context); 
        rc_aux = rcl_init_options_fini(&init_options);
        RCCHECK(clean2)

        rcl_node_t node = rcl_get_zero_initialized_node();
        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, "crazyflie_node_2", "", &context, &node_ops);
        RCCHECK(clean2)

        // Create subscription 2
        const char * echo_topic_name = "Float__Sequence";

        rcl_subscription_t sub_sensors      = rcl_get_zero_initialized_subscription();
        const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserEcho);
        rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
        subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_subscription_init(
            &sub_sensors,
            &node,
            sub_type_support,
            echo_topic_name,
            &subscription_ops);
        RCCHECK(clean2)

        // Create wait set
        rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
        rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
        RCCHECK(clean2)

        if(rc != RCL_RET_OK){
            rc_aux = rcl_node_fini(&node);
        }
        RCCHECK(clean2)

        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        // ####################### MAIN LOOP #######################

        while(1){

            rc = rcl_wait_set_clear(&wait_set);
            RCSOFTCHECK()

            rc = rcl_wait_set_add_subscription(&wait_set, &sub_sensors, NULL);
            RCSOFTCHECK()

            rc = rcl_wait(&wait_set, RCL_MS_TO_NS(10));
            // RCSOFTCHECK()

            if (wait_set.subscriptions[0]){
                sensor_msgs__msg__LaserEcho rcv;
                float rcv_data[2];

                rcv.echoes.capacity = 2;
                rcv.echoes.size = 2;
                rcv.echoes.data = rcv_data;

                rc = rcl_take(&sub_sensors, &rcv, NULL, NULL);

                if (rc == RCL_RET_OK) {
                    memcpy(&sensor_topic, &rcv, sizeof(sensor_msgs__msg__LaserEcho));
                    sensor_data_ready = 1;
                }      
            }

            vTaskDelay(100/portTICK_RATE_MS);
        }

        rc = rcl_subscription_fini(&sub_sensors, &node);
        rc = rcl_node_fini(&node);
clean2:     
        rc = rcl_shutdown(&context);
        DEBUG_PRINT("Connection lost on secondary, retriying\n");
    }

    vTaskSuspend( NULL );
}