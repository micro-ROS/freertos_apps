/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <geometry_msgs/msg/point32.h>
#include "example_interfaces/srv/add_two_ints.h"
#include <geometry_msgs/msg/twist.h>

#include <rcutils/allocator.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include "radiolink.h"
#include <time.h>

#include "microrosapp.h"

#define RCCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); goto clean;}
#define RCSOFTCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

float sign(float x){   
    return (x >= 0) ? 1.0 : -1.0;
}

void appMain(){ 
    while(1){
        absoluteUsedMemory = 0;
        usedMemory = 0;

        // ####################### RADIO INIT #######################

        int radio_connected = logGetVarId("radio", "isConnected");
        int radio_rssi = logGetVarId("radio", "rssi");
        while(!logGetUint(radio_connected)) vTaskDelay(100);
        DEBUG_PRINT("Radio connected\n");

        // ####################### MICROROS INIT #######################

        DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
        vTaskDelay(50);

        rcl_context_t      context;
        rcl_init_options_t init_options;
        rcl_ret_t          rc;

        init_options = rcl_get_zero_initialized_init_options();                          
        rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();   
        freeRTOS_allocator.allocate = __crazyflie_allocate;
        freeRTOS_allocator.deallocate = __crazyflie_deallocate;
        freeRTOS_allocator.reallocate = __crazyflie_reallocate;
        freeRTOS_allocator.zero_allocate = __crazyflie_zero_allocate;

        if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {                       
            DEBUG_PRINT("Error on default allocators (line %d)\n",__LINE__); 
            vTaskSuspend( NULL );
        }

        rc = rcl_init_options_init(&init_options, rcutils_get_default_allocator());      
        RCCHECK()

        context = rcl_get_zero_initialized_context();                                    

        rc = rcl_init(0, NULL, &init_options, &context); 
        RCCHECK()

        rc = rcl_init_options_fini(&init_options);

        rcl_node_t node = rcl_get_zero_initialized_node();
        rcl_node_options_t node_ops = rcl_node_get_default_options();

        rc = rcl_node_init(&node, "crazyflie_node", "", &context, &node_ops);
        RCCHECK()

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
            (void*) rcl_node_fini(&node);
        }
        RCCHECK()

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
            (void*) rcl_publisher_fini(&pub_odom, &node);
            (void*) rcl_node_fini(&node);
        }
        RCCHECK()

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

        DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
        DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
        DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

        // ####################### MAIN LOOP #######################

        // static P2PPacket pk;
        // pk.port = 0;
        // pk.size = 11;
        // memcpy(pk.data, "Hello World", 11);
        // radiolinkSendP2PPacketBroadcast(&pk);

        radiolinkSetPowerDbm(-50);

        while(logGetUint(radio_connected)){

            pose.x     = logGetFloat(pitchid);
            pose.y     = logGetFloat(rollid);
            pose.z     = logGetFloat(yawid);
            odom.x     = logGetFloat(Xid);
            odom.y     = logGetFloat(Yid);
            odom.z     = logGetFloat(Zid);

            // Debug
            // odom.x     = logGetFloat(radio_rssi);
            // odom.y     = xPortGetFreeHeapSize();

            rc = rcl_publish( &pub_attitude, (const void *) &pose, NULL);
            RCSOFTCHECK()

            rc = rcl_publish( &pub_odom, (const void *) &odom, NULL);
            RCSOFTCHECK()
            
            vTaskDelay(10/portTICK_RATE_MS);
        }

        rc = rcl_publisher_fini(&pub_odom, &node);
        rc = rcl_publisher_fini(&pub_attitude, &node);
        rc = rcl_node_fini(&node);
clean:     
        rc = rcl_shutdown(&context);
        DEBUG_PRINT("Connection lost, retriying\n");
    }

    vTaskSuspend( NULL );
}