/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <geometry_msgs/msg/point32.h>
#include "example_interfaces/srv/add_two_ints.h"
#include <geometry_msgs/msg/twist.h>

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

int absoluteUsedMemory;
int usedMemory;

void microros_primary(void * params);
void microros_secondary(void * params);

void appMain(){ 
    BaseType_t rc __attribute__((unused));
    TaskHandle_t task_primary, task_secondary;

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

    rc = xTaskCreate(
                    microros_primary,       /* Function that implements the task. */
                    "microROSprimary",      /* Text name for the task. */
                    2250,                   /* Stack size in words, not bytes. */
                    NULL,                   /* Parameter passed into the task. */
                    3,                      /* Priority at which the task is created. */
                    &task_primary );        /* Used to pass out the created task's handle. */

    rc = xTaskCreate(
                microros_secondary,       /* Function that implements the task. */
                "microROSsecondary",      /* Text name for the task. */
                2250,                   /* Stack size in words, not bytes. */
                NULL,                   /* Parameter passed into the task. */
                3,                      /* Priority at which the task is created. */
                &task_secondary );        /* Used to pass out the created task's handle. */
}

void microros_primary(void * params){ 
     while(1){

        // ####################### RADIO INIT #######################

        int radio_connected = logGetVarId("radio", "isConnected");
        while(!logGetUint(radio_connected)) vTaskDelay(100);
        DEBUG_PRINT("Radio connected\n");

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
        RCCHECK(clean1)

        rc = rcl_init_options_fini(&init_options);

        rcl_node_t node = rcl_get_zero_initialized_node();
        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, "crazyflie_node_1", "", &context, &node_ops);
        RCCHECK(clean1)

        // Create publisher 1
        const char* drone_odom = "/drone/odometry";

        rcl_publisher_t pub_odom        = rcl_get_zero_initialized_publisher();
        const rosidl_message_type_support_t * pub_type_support_odom = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
        rcl_publisher_options_t pub_opt_odom = rcl_publisher_get_default_options();
        pub_opt_odom.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_publisher_init(
            &pub_odom,
            &node,
            pub_type_support_odom,
            drone_odom,
            &pub_opt_odom);

        if(rc != RCL_RET_OK){
            rc_aux = rcl_node_fini(&node);
        }
        RCCHECK(clean1)

        // Init messages 
        geometry_msgs__msg__Point32 odom;
        geometry_msgs__msg__Point32__init(&odom);

        //Get X,Y and Z value
        int Xid = logGetVarId("stateEstimate", "x");
        int Yid = logGetVarId("stateEstimate", "y");
        int Zid = logGetVarId("stateEstimate", "z");

        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        // ####################### MAIN LOOP #######################

        while(logGetUint(radio_connected)){

            odom.x     = logGetFloat(Xid);
            odom.y     = logGetFloat(Yid);
            odom.z     = logGetFloat(Zid);

            rc = rcl_publish( &pub_odom, (const void *) &odom, NULL);
            RCSOFTCHECK()
            
            vTaskDelay(500/portTICK_RATE_MS);
        }

        rc = rcl_publisher_fini(&pub_odom, &node);
        rc = rcl_node_fini(&node);
clean1:     
        rc = rcl_shutdown(&context);
        DEBUG_PRINT("Connection lost on primary, retriying\n");
    }

    vTaskSuspend( NULL );
}

void microros_secondary(void * params){
     while(1){

        // ####################### RADIO INIT #######################

        int radio_connected = logGetVarId("radio", "isConnected");
        while(!logGetUint(radio_connected)) vTaskDelay(100);
        DEBUG_PRINT("Radio connected\n");

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
        RCCHECK(clean2)

        rc = rcl_init_options_fini(&init_options);

        rcl_node_t node = rcl_get_zero_initialized_node();
        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, "crazyflie_node_1", "", &context, &node_ops);
        RCCHECK(clean2)

        // Create publisher 2
        const char* drone_attitude = "/drone/attitude";

        rcl_publisher_t pub_attitude        = rcl_get_zero_initialized_publisher();
        const rosidl_message_type_support_t * pub_type_support_att = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
        rcl_publisher_options_t pub_opt_att = rcl_publisher_get_default_options();
        pub_opt_att.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        rc = rcl_publisher_init(
            &pub_attitude,
            &node,
            pub_type_support_att,
            drone_attitude,
            &pub_opt_att);

        if(rc != RCL_RET_OK){
            rc_aux = rcl_node_fini(&node);
        }
        RCCHECK(clean2)

        // Init messages 
        geometry_msgs__msg__Point32 pose;
        geometry_msgs__msg__Point32__init(&pose);

        //Get pitch, roll and yaw value
        int pitchid = logGetVarId("stateEstimate", "pitch");
        int rollid = logGetVarId("stateEstimate", "roll");
        int yawid = logGetVarId("stateEstimate", "yaw");


        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        // ####################### MAIN LOOP #######################

        while(logGetUint(radio_connected)){

            pose.x     = logGetFloat(pitchid);
            pose.y     = logGetFloat(rollid);
            pose.z     = logGetFloat(yawid);

            rc = rcl_publish( &pub_attitude, (const void *) &pose, NULL);
            RCSOFTCHECK()
            
            vTaskDelay(500/portTICK_RATE_MS);
        }

        rc = rcl_publisher_fini(&pub_attitude, &node);
        rc = rcl_node_fini(&node);
clean2:     
        rc = rcl_shutdown(&context);
        DEBUG_PRINT("Connection lost on secondary, retriying\n");
    }

    vTaskSuspend( NULL );
}