/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "semphr.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"

#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/ping.h>

#include <uxr/client/config.h>
#include <uxr/client/client.h>
#include <uxr/client/util/ping.h>

#include "config.h"
#include "log.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include "radiolink.h"
#include "param.h"
#include <time.h>

#include "microrosapp.h"

#include "crtp.h"
#include "configblock.h"

#define STREAM_HISTORY  1
#define BUFFER_SIZE UXR_CONFIG_CUSTOM_TRANSPORT_MTU * STREAM_HISTORY

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void microros_primary(void * params);
void microros_secondary(void * params);

std_msgs__msg__Int32 sensor_topic;
static bool sensor_data_ready = false;
static bool connection_established = false;

static int qxid, qyid, qzid, qwid;
static int Xid, Yid, Zid;

// Note: please set APP_STACKSIZE = 100 and CFLAGS += -DFREERTOS_HEAP_SIZE=12100 in Makefile before build

STATIC_MEM_TASK_ALLOC(microros_primary, 1000);
STATIC_MEM_TASK_ALLOC(microros_secondary, 1000);
static uint8_t crtp_buffer_primary[CRTP_BUFFER_SIZE];
static uint8_t crtp_buffer_secondary[CRTP_BUFFER_SIZE];
static bool created_primary = false;

SemaphoreHandle_t xMutex;
StaticSemaphore_t xMutexBuffer;

static paramVarId_t paramIdCommanderEnHighLevel, paramIdxyVelMax;

static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

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

    xMutex = xSemaphoreCreateBinaryStatic(&xMutexBuffer);
    xSemaphoreGive(xMutex);

    // Enable high level controller
    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    enableHighlevelCommander();

    // Set PID max speed
    paramIdxyVelMax = paramGetVarId("posCtlPid", "xyVelMax");
    paramSetFloat(paramIdxyVelMax, 0.5);

    STATIC_MEM_TASK_CREATE(microros_primary, microros_primary, "microROSprimary", NULL, 3);
    STATIC_MEM_TASK_CREATE(microros_secondary, microros_secondary, "microROSsecondary", NULL, 3);
}

void microros_primary(void * params)
{
    // ####################### MICROROS INIT #######################
    DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
    vTaskDelay(50/portTICK_RATE_MS);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    transport_args primary_args = { .radio_channel = 65, .radio_port = 9, .crtp_buffer = &crtp_buffer_primary[0] };
    RCCHECK(rmw_uros_options_set_custom_transport(
        true,
        (void *) &primary_args,
        crazyflie_serial_open,
        crazyflie_serial_close,
        crazyflie_serial_write,
        crazyflie_serial_read,
        rmw_options));

    DEBUG_PRINT("Waiting for main Agent\n");
    // Wait for agent connection
    while(RMW_RET_OK != rmw_uros_ping_agent_options(100, 1, rmw_options))
    {
        vTaskDelay(250/portTICK_RATE_MS);
    }

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    DEBUG_PRINT("Connected to Agent!\n");

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "crazyflie_node_1", "", &support));

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
    rcl_publisher_t pub_tf;
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_tf,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
        "/drone/tf"));

    DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
    DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
    DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

    created_primary = true;

    // ####################### MAIN LOOP #######################

    // Init tf message memory
    geometry_msgs__msg__TransformStamped tf;
    static micro_ros_utilities_memory_conf_t conf = {0};

    if (!micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
        &tf,
        conf))
    {
        DEBUG_PRINT("Memory allocation for /drone messages failed\n");
        return;
    }

    tf.child_frame_id = micro_ros_string_utilities_set(tf.child_frame_id, "/base_footprint_drone");
    tf.header.frame_id = micro_ros_string_utilities_set(tf.header.frame_id, "/map");

    // Get quaternion
    qxid = logGetVarId("stateEstimate", "qx");
    qyid = logGetVarId("stateEstimate", "qy");
    qzid = logGetVarId("stateEstimate", "qz");
    qwid = logGetVarId("stateEstimate", "qw");

    // Get X,Y and Z value
    Xid = logGetVarId("stateEstimate", "x");
    Yid = logGetVarId("stateEstimate", "y");
    Zid = logGetVarId("stateEstimate", "z");

    while(1)
    {
        if(sensor_data_ready)
        {
            std_msgs__msg__Float32 aux_msg;

            xSemaphoreTake(xMutex, portMAX_DELAY);
            DEBUG_PRINT("Publishing sensor data\n");
            aux_msg.data = (sensor_topic.data & 0x0000FFFF)/1000.0;
            RCSOFTCHECK(rcl_publish( &pub_sensors_temp, (const void *) &aux_msg, NULL));
            aux_msg.data = ((uint32_t) sensor_topic.data >> 16)/1000.0;
            RCSOFTCHECK(rcl_publish( &pub_sensors_hum, (const void *) &aux_msg, NULL));
            sensor_data_ready = 0;
            xSemaphoreGive(xMutex);
        }

        tf.transform.rotation.x = logGetFloat(qxid);
        tf.transform.rotation.y = logGetFloat(qyid);
        tf.transform.rotation.z = logGetFloat(qzid);
        tf.transform.rotation.w = logGetFloat(qwid);

        tf.transform.translation.x = logGetFloat(Xid);
        tf.transform.translation.y = logGetFloat(Yid);
        tf.transform.translation.z = logGetFloat(Zid);

        // Fill the message timestamp.
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
        tf.header.stamp.sec = ts.tv_sec;
        tf.header.stamp.nanosec = ts.tv_nsec;

        RCSOFTCHECK(rcl_publish( &pub_tf, &tf, NULL));

        vTaskDelay(100/portTICK_RATE_MS);
    }

    vTaskSuspend( NULL );
}


void on_topic_secondary(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) args; (void) length;

    const rosidl_message_type_support_t * type_support_xrce = get_message_typesupport_handle(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE);
    const message_type_support_callbacks_t * cdr_callbacks = (message_type_support_callbacks_t*) type_support_xrce->data;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    if (cdr_callbacks->cdr_deserialize(ub, &sensor_topic))
    {
        sensor_data_ready = 1;
        connection_established = true;
    }
    xSemaphoreGive(xMutex);
}

void microros_secondary(void * params){

    while(!created_primary){
        vTaskDelay(100/portTICK_RATE_MS);
    }

    DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
    vTaskDelay(50/portTICK_RATE_MS);

    // Transport
    uxrCustomTransport transport;
    transport_args secondary_args = { .radio_channel = 30, .radio_port = 10, .crtp_buffer = &crtp_buffer_secondary[0] };

    transport.framing = true;
    transport.open = crazyflie_serial_open;
    transport.close = crazyflie_serial_close;
    transport.write = crazyflie_serial_write;
    transport.read = crazyflie_serial_read;

    if (!uxr_init_custom_transport(&transport, (void *) &secondary_args)) {
        DEBUG_PRINT("Create secondary transport failed. Aborting task.\n");
        vTaskDelete(NULL);
    }

    // Reduce radio power for sensor demo
    radiolinkSetPowerDbm(-6);

    // Session
    uxrSession session;
    uint32_t session_key = 0x12345678;
    uxr_init_session(&session, &transport.comm, session_key);
    uxr_set_topic_callback(&session, on_topic_secondary, NULL);

    // Stream
    uint8_t output_stream_buffer[BUFFER_SIZE];
    uxrStreamId stream_out = uxr_create_output_best_effort_stream(&session, output_stream_buffer, BUFFER_SIZE);
    uxrStreamId stream_in = uxr_create_input_best_effort_stream(&session);

    // Request topics
    uxrDeliveryControl delivery_control = {0};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;

    uint16_t datareader_key = 0x01;
    uxrObjectId datareader_id = uxr_object_id(datareader_key, UXR_DATAREADER_ID);

    DEBUG_PRINT("Secondary task started!\n");
    while (1)
    {
        if (!connection_established)
        {
            uxr_buffer_request_data(&session, stream_out, datareader_id, stream_in, &delivery_control);
        }

        uxr_run_session_until_data(&session, 100);
        vTaskDelay(250/portTICK_RATE_MS);
    }

    vTaskSuspend( NULL );
}
