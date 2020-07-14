// Copyright 2020 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rmw_microxrcedds_c/config.h>
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <ucdr/microcdr.h>
#include <string.h>

#include <stdio.h> //printf
#include <string.h> //strcmp
#include <stdlib.h> //atoi

#include "FreeRTOS.h"

#include "config.h"

#ifdef UCLIENT_PROFILING_XML
#define PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE  400
#endif
#ifdef UCLIENT_PROFILING_REF
#define PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE  150
#endif

#define MINIMUM_BUFFER_SIZE 32
#define MAX_CONTROL_MSG_PAYLOAD_SIZE 5 // heartbeat
#define CONTROL_MSGS_BUFFER_SIZE MINIMUM_BUFFER_SIZE + MAX_CONTROL_MSG_PAYLOAD_SIZE
#if (TOPIC_SIZE_M + MINIMUM_BUFFER_SIZE) < PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE
    #define CALCULATE_OUTPUT_BUFFER_SIZE PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE
#else
    #define CALCULATE_OUTPUT_BUFFER_SIZE TOPIC_SIZE_M + MINIMUM_BUFFER_SIZE
#endif

#define STREAM_HISTORY (CALCULATE_OUTPUT_BUFFER_SIZE/UXR_CONFIG_SERIAL_TRANSPORT_MTU) + 2
#define INPUT_BUFFER_LEN STREAM_HISTORY*UXR_CONFIG_SERIAL_TRANSPORT_MTU


char topic_data[CALCULATE_OUTPUT_BUFFER_SIZE];

typedef struct ByteArray
{
    char * message;
} ByteArray;

bool ByteArray_serialize_topic(ucdrBuffer* writer, const ByteArray* topic)
{
    (void) ucdr_serialize_string(writer, topic->message);

    return !writer->error;
}

bool ByteArray_deserialize_topic(ucdrBuffer* reader, ByteArray* topic)
{
    (void) ucdr_deserialize_string(reader, topic->message, 10240);

    return !reader->error;
}

uint32_t ByteArray_size_of_topic(const ByteArray* topic, uint32_t size)
{
    uint32_t previousSize = size;
    size += (uint32_t)(ucdr_alignment(size, 4) + 4 + strlen(topic->message) + 1);

    return size - previousSize;
}

void on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length;

    ByteArray topic;
    topic.message = topic_data;
    memset(topic.message, 0, sizeof(topic.message));
    ByteArray_deserialize_topic(ub, &topic);

    uint32_t* count_ptr = (uint32_t*) args;
    (*count_ptr)++;

    if (*count_ptr == 30){
        UBaseType_t uxHighWaterMark;
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        printf("%d\n", (4*3000) - uxHighWaterMark*4);
    }

    // printf("Received: %d %d\n", *count_ptr, strlen(topic.message));
}

void appMain(void * arg)
{   
    uint16_t sub_number = SUB_NUM;
    uint16_t topic_size = TOPIC_SIZE_M;
    uint32_t count = 0;

    // Init transport

#if defined(MICRO_XRCEDDS_UDP)
    uxrUDPTransport transport;
    uxrUDPPlatform udp_platform;
    uxrIpProtocol ip_protocol = UXR_IPv4;
    const char* ip = "localhost";
    const char* port = "8888";
  if (!uxr_init_udp_transport(&transport, &udp_platform, ip_protocol, ip, port))
  {
    return -1;
  }
#elif defined(MICRO_XRCEDDS_CUSTOM_SERIAL)
  int pseudo_fd = 0;
  if (strlen(RMW_UXRCE_DEFAULT_SERIAL_DEVICE) > 0) {
    pseudo_fd = atoi(RMW_UXRCE_DEFAULT_SERIAL_DEVICE);
  }

  uxrSerialTransport transport;
  uxrSerialPlatform serial_platform;

  if (!uxr_init_serial_transport(&transport, &serial_platform, pseudo_fd, 0, 1))
  {
    return -1;
  }
#endif

    // Create session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAABBCD + SUB_NUM + TOPIC_SIZE_M);
    uxr_set_topic_callback(&session, on_topic, &count);
    if(!uxr_create_session(&session))
    {
        while(1){}
    }

    // Create streams
#ifdef UCLIENT_PROFILING_BEST_EFFORT

    static uint8_t output_besteffort_stream_buffer[CALCULATE_OUTPUT_BUFFER_SIZE];
    uxrStreamId stream_out = uxr_create_output_best_effort_stream(&session, output_besteffort_stream_buffer, PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE);

    static uint8_t input_reliable_stream_buffer[CALCULATE_OUTPUT_BUFFER_SIZE];
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, CONTROL_MSGS_BUFFER_SIZE, 1);

    uxrStreamId stream_in = uxr_create_input_best_effort_stream(&session);
#endif

#ifdef UCLIENT_PROFILING_RELIABLE
    static uint8_t output_reliable_stream_buffer[PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE];
    uxrStreamId stream_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, PROFILING_SUB_MIN_OUTPUT_BUFFER_SIZE, 1);

    static uint8_t input_reliable_stream_buffer[INPUT_BUFFER_LEN];
    uint32_t a = CALCULATE_OUTPUT_BUFFER_SIZE;
    uint32_t b = UXR_CONFIG_SERIAL_TRANSPORT_MTU;
    uint16_t stream_history_var = (a/b) + 2;
    uxrStreamId stream_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, INPUT_BUFFER_LEN, stream_history_var);
#endif

    static uxrObjectId datareader_ids[SUB_NUM];

#ifdef UCLIENT_PROFILING_XML
    // Create DDS entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                      "<participant>"
                                          "<rtps>"
                                              "<name>default_xrce_participant</name>"
                                          "</rtps>"
                                      "</participant>"
                                  "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, stream_out, participant_id, 0, participant_xml, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
                                "<topic>"
                                    "<name>ByteArrayTopic</name>"
                                    "<dataType>ByteArray</dataType>"
                                "</topic>"
                            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(&session, stream_out, topic_id, participant_id, topic_xml, UXR_REPLACE);

#ifdef UCLIENT_PROFILING_RELIABLE
    uint8_t status[2];
    uint16_t requests[2] = {participant_req, topic_req};
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 2))
    {
        while(1){}
    }
    uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
    uxr_flash_output_streams(&session);
    usleep(10000);
#endif

    for (size_t i = 0; i < sub_number; i++)
    {
        uxrObjectId subscriber_id = uxr_object_id(i + 1, UXR_SUBSCRIBER_ID);
        const char* subscriber_xml = "";
        uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session, stream_out, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

        uxrObjectId datareader_id = uxr_object_id(i + 1, UXR_DATAREADER_ID);
        const char* datareader_xml = "<dds>"
                                        "<data_reader>"
                                            "<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"
                                            "<topic>"
                                                "<kind>NO_KEY</kind>"
                                                "<name>ByteArrayTopic</name>"
                                                "<dataType>ByteArray</dataType>"
                                            "</topic>"
                                        "</data_reader>"
                                    "</dds>";

        datareader_ids[i] = datareader_id;
        uint16_t datareader_req = uxr_buffer_create_datareader_xml(&session, stream_out, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

#ifdef UCLIENT_PROFILING_RELIABLE
        uint8_t status[2];
        uint16_t requests[2] = {subscriber_req, datareader_req};
        if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 2))
        {
            while(1){}
        }
        uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
        uxr_flash_output_streams(&session);
        usleep(10000);
#endif
    }
#endif
#ifdef UCLIENT_PROFILING_REF
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_ref = "default_xrce_participant";
    uint16_t participant_req = uxr_buffer_create_participant_ref(&session, stream_out, participant_id, 0, participant_ref, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_ref = "profiling_topic";
    uint16_t topic_req = uxr_buffer_create_topic_ref(&session, stream_out, topic_id, participant_id, topic_ref, UXR_REPLACE);

#ifdef UCLIENT_PROFILING_RELIABLE
    uint8_t status[2];
    uint16_t requests[2] = {participant_req, topic_req};
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 2))
    {
        while(1){}
    }
    uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
    uxr_flash_output_streams(&session);
    usleep(10000);
#endif

    for (size_t i = 0; i < sub_number; i++)
    {
        uxrObjectId subscriber_id = uxr_object_id(i + 1, UXR_SUBSCRIBER_ID);
        const char* subscriber_xml = "";
        uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session, stream_out, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

        uxrObjectId datareader_id = uxr_object_id(i + 1, UXR_DATAREADER_ID);
        const char* datareader_ref = "profiling_data_reader";
        uint16_t datareader_req = uxr_buffer_create_datareader_ref(&session, stream_out, datareader_id, subscriber_id, datareader_ref, UXR_REPLACE);

        datareader_ids[i] = datareader_id;

#ifdef UCLIENT_PROFILING_RELIABLE
        uint8_t sub_dw_status[2];
        uint16_t sub_dw_requests[2] = {subscriber_req, datareader_req};
        if(!uxr_run_session_until_all_status(&session, UXR_TIMEOUT_INF, sub_dw_requests, sub_dw_status, 2))
        {
            while(1){}
        }
        uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
        uxr_flash_output_streams(&session);
        usleep(10000);
#endif
    }
#endif

    uxr_flash_output_streams(&session);

    for (size_t i = 0; i < sub_number; i++)
    {
        // Request topics
        uxrDeliveryControl delivery_control = {0};
        delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
        delivery_control.min_pace_period = 0;
        delivery_control.max_elapsed_time = UXR_MAX_ELAPSED_TIME_UNLIMITED;
        delivery_control.max_bytes_per_second = UXR_MAX_BYTES_PER_SECOND_UNLIMITED;
        uint16_t datareq = uxr_buffer_request_data(&session, stream_out, datareader_ids[i], stream_in, &delivery_control);
        uint8_t status;
        uxr_run_session_until_all_status(&session, UXR_TIMEOUT_INF, &datareq, &status, 1);
    }



    while(count < 30)
    {
        uxr_run_session_timeout(&session, 500);
        usleep(5000);
    }

    uxr_delete_session(&session);
    // uxr_close_udp_transport(&transport);

    while(1){
        sleep(10);
    }
    return 0;
}
