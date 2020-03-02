/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <geometry_msgs/msg/point32.h>
#include "example_interfaces/srv/add_two_ints.h"

#include <rcutils/allocator.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include <time.h>

#include "microrosapp.h"

#define RCCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); vTaskSuspend( NULL );}
#define RCSOFTCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

void guardConditionTask(void * gc){
    struct timespec ts;

    while(1){
        vTaskDelay(1000/portTICK_RATE_MS);

        clock_gettime(CLOCK_REALTIME, &ts);

        DEBUG_PRINT("Triggering guard condition at %d,%d\n",(int)ts.tv_sec,(int)(ts.tv_nsec/1000000LL));
        rcl_ret_t rc = rcl_trigger_guard_condition((rcl_guard_condition_t *) gc);
        RCSOFTCHECK()
    }
}

double timespec_diff(struct timespec *start, struct timespec *stop){
    struct timespec result;
    double ret;
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result.tv_sec = stop->tv_sec - start->tv_sec - 1;
        result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result.tv_sec = stop->tv_sec - start->tv_sec;
        result.tv_nsec = stop->tv_nsec - start->tv_nsec;
    }
    ret = result.tv_sec;
    ret += result.tv_nsec/1000000000.0;
    return ret;
}

void rateCounter(void * n_packets){
    struct timespec init, end;

    uint32_t * n = (uint32_t *)n_packets;

    clock_gettime(CLOCK_REALTIME, &init);

    while(1){
        clock_gettime(CLOCK_REALTIME, &end);

        double elapsed = timespec_diff(&init,&end);
        double rate = ((double)*n)/elapsed;

        DEBUG_PRINT("Subscription rate: %0.3f (%d p in %f s)\n",rate,(int)*n,elapsed);

        *n = 0;
        clock_gettime(CLOCK_REALTIME, &init);

        vTaskDelay(1000/portTICK_RATE_MS);
    }
}

bool client_waiting = false;

void sendRequestsTask(void * client){
    struct timespec ts;
    int64_t seq; 

    example_interfaces__srv__AddTwoInts_Request req;
    example_interfaces__srv__AddTwoInts_Request__init(&req);
    req.a = 11;
    req.b = 42;

    while(1){
        // if (!client_waiting)
        // {
            clock_gettime(CLOCK_REALTIME, &ts);
            rcl_ret_t rc = rcl_send_request((rcl_client_t *)client, &req, &seq);
            RCSOFTCHECK()

            DEBUG_PRINT("Sending service request %d + %d  at %d,%d. Seq %d\n",(int)req.a, (int)req.b, (int)ts.tv_sec,(int)(ts.tv_nsec/1000000LL), (int)seq);

            client_waiting = true;
        // }

        vTaskDelay(1000/portTICK_RATE_MS);
    }
}


void appMain(){ 
    absoluteUsedMemory = 0;
    usedMemory = 0;

    vTaskDelay(2000);
    int radio_connected = logGetVarId("radio", "isConnected");
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

    rc = rcl_publisher_init(
        &pub_odom,
        &node,
        pub_type_support_odom,
        drone_odom,
        &pub_opt_odom);
    RCCHECK()

    // Create publisher 2
    const char* drone_attitude = "/drone/attitude";

    rcl_publisher_t pub_attitude        = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * pub_type_support_att = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
    rcl_publisher_options_t pub_opt_att = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &pub_attitude,
        &node,
        pub_type_support_att,
        drone_attitude,
        &pub_opt_att);
    RCCHECK()

    // Create subscriber 1
    const char * echo_topic_name = "/drone/echo";

    rcl_subscription_t sub_echo      = rcl_get_zero_initialized_subscription();
    const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
    rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

    rc = rcl_subscription_init(
        &sub_echo,
        &node,
        sub_type_support,
        echo_topic_name,
        &subscription_ops);
    RCCHECK()

    uint32_t n_packets = 0;
    xTaskCreate(rateCounter, "SUBRATE",200, (void *) &n_packets, 3, NULL); // This task will eventually trigger the guard condition


    // Create guard condition
    rcl_guard_condition_t gc1 = rcl_get_zero_initialized_guard_condition();
    rcl_guard_condition_options_t gc1_options = rcl_guard_condition_get_default_options();
    rc = rcl_guard_condition_init(&gc1,&context,gc1_options);
    RCCHECK()
    
    xTaskCreate(guardConditionTask, "TRIGGERGC",200, (void *) &gc1, 3, NULL); // This task will eventually trigger the guard condition

    // Create service
    const char * service_name = "/drone/suminput";

    rcl_service_t serv = rcl_get_zero_initialized_service();
    rcl_service_options_t service_options = rcl_service_get_default_options();
    const rosidl_service_type_support_t * service_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    rc = rcl_service_init(
        &serv,
        &node,
        service_type_support,
        service_name,
        &service_options);
    RCCHECK()

    // Create client
    const char * client_name = "/drone/sumoutput";

    rcl_client_t client = rcl_get_zero_initialized_client();
    rcl_client_options_t client_options = rcl_client_get_default_options();
    const rosidl_service_type_support_t * client_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

    rc = rcl_client_init(
        &client,
        &node,
        client_type_support,
        client_name,
        &client_options);
    RCCHECK()

    xTaskCreate(sendRequestsTask, "REQUESTAST",200, (void *) &client, 3, NULL); // This task will eventually send a service request

    // Create wait set
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    rc = rcl_wait_set_init(&wait_set, 1, 1, 0, 1, 1, 0, &context, rcl_get_default_allocator());
    RCCHECK()

    // Init messages 
    geometry_msgs__msg__Point32 pose;
    geometry_msgs__msg__Point32__init(&pose);
    geometry_msgs__msg__Point32 odom;
    geometry_msgs__msg__Point32__init(&odom);


  // ####################### MAIN LOOP #######################

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
        
        rc = rcl_wait_set_clear(&wait_set);
        RCSOFTCHECK()

        rc = rcl_wait_set_add_subscription(&wait_set, &sub_echo, NULL);
        RCSOFTCHECK()

        rc = rcl_wait_set_add_guard_condition(&wait_set, &gc1, NULL);
        RCSOFTCHECK()

        rc = rcl_wait_set_add_service(&wait_set, &serv, NULL);
        RCSOFTCHECK()

        rc = rcl_wait_set_add_client(&wait_set, &client, NULL);
        RCSOFTCHECK()

        rc = rcl_wait(&wait_set, RCL_MS_TO_NS(10));
        RCSOFTCHECK()

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        
        // Check if subscription is available
        if (wait_set.subscriptions[0])
        {
            geometry_msgs__msg__Point32 echo;
            geometry_msgs__msg__Point32__init(&echo);
            rmw_message_info_t        messageInfo;
            rc = rcl_take(&sub_echo, &echo, &messageInfo, NULL);

            if (rc == RCL_RET_OK) {
                n_packets++;
                // DEBUG_PRINT("Received command [%f %f %f]\n",(double)echo.x,(double)echo.y,(double)echo.z);
            }      
        }

        // Check if guard condition is triggered
        if (wait_set.guard_conditions[0])
        {
            // DEBUG_PRINT("Guard condition readed at %d,%d\n",(int)ts.tv_sec,(int)(ts.tv_nsec/1000000LL));

            float pitch = logGetFloat(pitchid);
            float roll  = logGetFloat(rollid);
            float yaw   = logGetFloat(yawid);
            float x     = logGetFloat(Xid);
            float y     = logGetFloat(Yid);
            float z     = logGetFloat(Zid);

            pose.x = pitch;
            pose.y = roll;
            pose.z = yaw;

            odom.x = x;
            odom.y = y;
            odom.z = z;

            // rc = rcl_publish( &pub_odom, (const void *) &odom, NULL);
            // RCSOFTCHECK()
            // DEBUG_PRINT("Publishing odom [%f, %f, %f]\n",(float)pitch,(float)roll,(float)yaw);

            rc = rcl_publish( &pub_attitude, (const void *) &pose, NULL);
            RCSOFTCHECK()
            // DEBUG_PRINT("Publishing pose [%f, %f, %f]\n",(float)x,(float)y,(float)z);
        }

        // Check if service request is available 
        if (wait_set.services[0])
        {   
            rmw_request_id_t req_id;
            example_interfaces__srv__AddTwoInts_Request req;
            example_interfaces__srv__AddTwoInts_Request__init(&req);
            rc = rcl_take_request(&serv,&req_id,&req);
            RCSOFTCHECK()

            DEBUG_PRINT("Service request at %d,%d, value: %d + %d\n",(int)ts.tv_sec,(int)(ts.tv_nsec/1000000LL),(int)req.a,(int)req.b);

            example_interfaces__srv__AddTwoInts_Response res;
            example_interfaces__srv__AddTwoInts_Response__init(&res);
            
            res.sum = req.a + req.b;

            rc = rcl_send_response(&serv,&req_id,&res);
            RCSOFTCHECK()
        }

        // Check if client response is available 
        if (wait_set.clients[0])
        {   
            rmw_request_id_t req_id;
            example_interfaces__srv__AddTwoInts_Response res;
            example_interfaces__srv__AddTwoInts_Response__init(&res);

            rc = rcl_take_response(&client,&req_id,&res);
            RCSOFTCHECK()

            DEBUG_PRINT("Received service response %d at %d,%d. Seq %d\n",(int)res.sum, (int)ts.tv_sec,(int)(ts.tv_nsec/1000000LL), (int)req_id.sequence_number);

            client_waiting = false;
        }

        vTaskDelay(10/portTICK_RATE_MS);
    }
    
    vTaskSuspend( NULL );
}