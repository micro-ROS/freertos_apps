#include <uxr/client/transport.h>

#include "FreeRTOS.h"
#include "radiolink.h"
#include "task.h"

#include "stdint.h"
#include "stdbool.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <watchdog.h>
#include "configblock.h"
#include "crtp.h"

#include "crtp.h"
#include "debug.h"

#include <microros_transports.h>

// --- micro-ROS Transports ---
#define BUFFER_SIZE 10000

#define PRIMARY_CHANNEL_PORT 9
#define SECONDARY_CHANNEL_PORT 10

static size_t crpt_primary_index = 0;
static size_t crpt_secondary_index = 0;

static size_t crpt_primary_index_max = 0;
static size_t crpt_secondary_index_max = 0;

static uint8_t * crtp_buffer_primary   = (uint8_t *)0x10000000;
static uint8_t * crtp_buffer_secondary = (uint8_t *)0x10000000 + BUFFER_SIZE;

static bool init_queue_primary = false;
static bool init_queue_seconday = false;

static struct crtpLinkOperations *link = NULL;

bool crazyflie_serial_open(struct uxrCustomTransport * transport){
    const uint8_t * radio_channel = (const uint8_t*)transport->args;
    uint8_t default_radio_channel = configblockGetRadioChannel();
    bool primary_channel = *radio_channel == default_radio_channel;

    if(primary_channel && !init_queue_primary){
        crtpInitTaskQueue(PRIMARY_CHANNEL_PORT);
        init_queue_primary = true;
    }else if(!init_queue_seconday){
        crtpInitTaskQueue(SECONDARY_CHANNEL_PORT);
        init_queue_seconday = true;
    }

    link = radiolinkGetLink();
    
    return true;
}

bool crazyflie_serial_close(struct uxrCustomTransport * transport){
    return true;
}

size_t crazyflie_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    const uint8_t * radio_channel = (const uint8_t*)transport->args;
    uint8_t default_radio_channel = configblockGetRadioChannel();
    bool primary_channel = *radio_channel == default_radio_channel;

    CRTPPacket send_pkg;
    size_t index = 0;
    size_t to_write;
    
    send_pkg.header = CRTP_HEADER((primary_channel) ? PRIMARY_CHANNEL_PORT : SECONDARY_CHANNEL_PORT, 0);
    
    while(len > 0){
        to_write = (len <= CRTP_MAX_DATA_SIZE) ? len : CRTP_MAX_DATA_SIZE;
        memcpy(send_pkg.data,&buf[index],to_write);
        send_pkg.size = to_write;
        len -= to_write;
        index += to_write;

        if(!primary_channel) radiolinkSetChannel(*radio_channel);
        while (link->sendPacket(&send_pkg) == false){
            vTaskDelay(10);
        }
        if(!primary_channel){
        vTaskDelay(20);
        radiolinkSetChannel(default_radio_channel);
        }
    }

    watchdogReset();

    return index;
}

size_t crazyflie_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    const uint8_t * radio_channel = (const uint8_t*)transport->args;
    uint8_t default_radio_channel = configblockGetRadioChannel();
    bool primary_channel = *radio_channel == default_radio_channel;

    static CRTPPacket recv_packet;
    size_t written;

    *err = 0;

    uint8_t * crtp_buffer = (primary_channel) ? crtp_buffer_primary : crtp_buffer_secondary;
    size_t * crpt_index = (primary_channel) ? &crpt_primary_index : &crpt_secondary_index;
    size_t * crpt_index_max = (primary_channel) ? &crpt_primary_index_max : &crpt_secondary_index_max;

    if(!primary_channel) radiolinkSetChannel(*radio_channel);


    size_t retrieved = 0;
    while(  *crpt_index + CRTP_MAX_DATA_SIZE < BUFFER_SIZE && 
            crtpReceivePacketWait((primary_channel) ? PRIMARY_CHANNEL_PORT : SECONDARY_CHANNEL_PORT, &recv_packet, timeout))
    {
        memcpy(&crtp_buffer[*crpt_index], recv_packet.data, recv_packet.size);
        *crpt_index += recv_packet.size;
        retrieved += recv_packet.size;
        if(retrieved > len){
        break;
        }
    }

    if(!primary_channel) radiolinkSetChannel(default_radio_channel);

    if (*crpt_index + CRTP_MAX_DATA_SIZE > BUFFER_SIZE){
        DEBUG_PRINT("MicroXRCEDDS CRTP Buffer full. Clearing buffer.\n");
        *crpt_index = 0;
        *crpt_index_max = 0;
        *err = 2;
    }

    if (*crpt_index > *crpt_index_max)
    {
        *crpt_index_max = *crpt_index;
    }

    if (len < *crpt_index)
    {
        memcpy(buf, crtp_buffer, len);
        memcpy(crtp_buffer, &crtp_buffer[len], (*crpt_index)-len);
        written = len;
        *crpt_index -= len;
    }else{
        memcpy(buf, crtp_buffer, *crpt_index);
        written = *crpt_index;
        *crpt_index = 0;
    }  

    watchdogReset();
    
    // DEBUG_PRINT("Retrived %d Flushed %d Max %d Current %d\n", retrieved, written, crpt_index_max, crpt_index);
    return written;
}