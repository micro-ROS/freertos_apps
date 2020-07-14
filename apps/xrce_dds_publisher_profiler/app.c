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
#define PROFILING_PUB_MIN_OUTPUT_BUFFER_SIZE  400
#endif
#ifdef UCLIENT_PROFILING_REF
#define PROFILING_PUB_MIN_OUTPUT_BUFFER_SIZE  150
#endif

#define MINIMUM_BUFFER_SIZE 32

#if (TOPIC_SIZE_M + MINIMUM_BUFFER_SIZE) < PROFILING_PUB_MIN_OUTPUT_BUFFER_SIZE
    #define CALCULATE_OUTPUT_BUFFER_SIZE(TOPIC_SIZE) PROFILING_PUB_MIN_OUTPUT_BUFFER_SIZE
#else
    #define CALCULATE_OUTPUT_BUFFER_SIZE(TOPIC_SIZE) TOPIC_SIZE + MINIMUM_BUFFER_SIZE
#endif

char topic_data[CALCULATE_OUTPUT_BUFFER_SIZE(TOPIC_SIZE_M)];

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

void appMain(void * arg)
{
    uint16_t pub_number = PUB_NUM;
    uint16_t topic_size = TOPIC_SIZE_M;

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
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    if(!uxr_create_session(&session))
    {
        printf("Error creating uXRCE session!\n");
        return 1;
    }

    // Create streams
#ifdef UCLIENT_PROFILING_BEST_EFFORT
    uint32_t buffer_length = CALCULATE_OUTPUT_BUFFER_SIZE(TOPIC_SIZE_M);

    static uint8_t output_besteffort_stream_buffer[CALCULATE_OUTPUT_BUFFER_SIZE(TOPIC_SIZE_M)];
    uxrStreamId stream_out = uxr_create_output_best_effort_stream(&session, output_besteffort_stream_buffer, buffer_length);

#endif

#ifdef UCLIENT_PROFILING_RELIABLE
    uint32_t buffer_length = CALCULATE_OUTPUT_BUFFER_SIZE(topic_size);

    static uint8_t output_reliable_stream_buffer[CALCULATE_OUTPUT_BUFFER_SIZE(TOPIC_SIZE_M)];
    uxrStreamId stream_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, buffer_length, 1);

    static uint8_t input_reliable_stream_buffer[MINIMUM_BUFFER_SIZE];
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, MINIMUM_BUFFER_SIZE, 1);
#endif

    static uxrObjectId datawriter_ids[PUB_NUM];

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
        return 1;
    }
    uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
    uxr_flash_output_streams(&session);
    usleep(10000);
#endif

    for (size_t i = 0; i < pub_number; i++)
    {
        uxrObjectId publisher_id = uxr_object_id(i + 1, UXR_PUBLISHER_ID);
        const char* publisher_xml = "";
        uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, stream_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

        uxrObjectId datawriter_id = uxr_object_id(i + 1, UXR_DATAWRITER_ID);
        const char* datawriter_xml = "<dds>"
                                        "<data_writer>"
                                            "<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"
                                            "<topic>"
                                                "<kind>NO_KEY</kind>"
                                                "<name>ByteArrayTopic</name>"
                                                "<dataType>ByteArray</dataType>"
                                            "</topic>"
                                        "</data_writer>"
                                    "</dds>";

        datawriter_ids[i] = datawriter_id;
        uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, stream_out, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE);

#ifdef UCLIENT_PROFILING_RELIABLE
        uint8_t status[2];
        uint16_t requests[2] = {publisher_req, datawriter_req};
        if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 2))
        {
            return 1;
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
        return 1;
    }
    uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
    uxr_flash_output_streams(&session);
    usleep(10000);
#endif

    for (size_t i = 0; i < pub_number; i++)
    {
        uxrObjectId publisher_id = uxr_object_id(i + 1, UXR_PUBLISHER_ID);
        const char* publisher_xml = "";
        uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, stream_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

        uxrObjectId datawriter_id = uxr_object_id(i + 1, UXR_DATAWRITER_ID);
        const char* datawriter_ref = "profiling_data_writer";
        uint16_t datawriter_req = uxr_buffer_create_datawriter_ref(&session, stream_out, datawriter_id, publisher_id, datawriter_ref, UXR_REPLACE);

        datawriter_ids[i] = datawriter_id;

#ifdef UCLIENT_PROFILING_RELIABLE
        uint8_t pub_dw_status[2];
        uint16_t pub_dw_requests[2] = {publisher_req, datawriter_req};
        if(!uxr_run_session_until_all_status(&session, 1000, pub_dw_requests, pub_dw_status, 2))
        {
            return 1;
        }
        uxr_run_session_time(&session, 5);
#endif
#ifdef UCLIENT_PROFILING_BEST_EFFORT
        uxr_flash_output_streams(&session);
        usleep(10000);
#endif
    }
#endif

    // Write topic info
    ByteArray topic;
    topic.message = topic_data;
    memset(topic.message, 'A', topic_size);
    topic.message[topic_size] = '\0';

    uint8_t count = 0;
#ifdef UCLIENT_PROFILING_INFINITE_LOOP
    while (true)
#else
    while ((30/pub_number) > count++)
#endif
    {
        for (size_t i = 0; i < pub_number; i++)
        {
            ucdrBuffer ub;
            uint32_t ba_topic_size = ByteArray_size_of_topic(&topic, 0);
            if (uxr_prepare_output_stream(&session, stream_out, datawriter_ids[i], &ub, ba_topic_size))
            {
                ByteArray_serialize_topic(&ub, &topic);
            }

#ifdef UCLIENT_PROFILING_BEST_EFFORT
            uxr_flash_output_streams(&session);
            usleep(5000);
#endif
#ifdef UCLIENT_PROFILING_RELIABLE
            uxr_run_session_time(&session, 5);
#endif      
            usleep(5000);
        }
    }

    // Delete resources
#ifdef UCLIENT_PROFILING_RELIABLE
    uxr_delete_session(&session);
#endif
    // uxr_close_udp_transport(&transport);

    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	printf("%d\n", (4*3000) - uxHighWaterMark*4);

    while(1){
        sleep(10);
    }
    return 0;
}
