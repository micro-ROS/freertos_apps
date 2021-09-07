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

static uint8_t * crtp_buffer_start   = (uint8_t *)0x10000000;
static struct crtpLinkOperations *link = NULL;
static uint8_t last_ch = 0;

bool crazyflie_serial_open(struct uxrCustomTransport * transport)
{
    transport_args * radio_args = (transport_args*)transport->args;
    static uint8_t count = 0;

    if (!radio_args->initialized)
    {
        crtpInitTaskQueue(radio_args->radio_port);
        radio_args->crtp_buffer = crtp_buffer_start + BUFFER_SIZE*count;
        count++;

        if (NULL == link)
        {
            link = radiolinkGetLink();
        }

        radio_args->initialized = true;
    }
    
    return true;
}

bool crazyflie_serial_close(struct uxrCustomTransport * transport)
{
    return true;
}

size_t crazyflie_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err)
{
    const transport_args * radio_args = (transport_args*)transport->args;

    CRTPPacket send_pkg;
    size_t index = 0;
    size_t to_write;
    
    send_pkg.header = CRTP_HEADER(radio_args->radio_port, 0);
    
    if(last_ch != radio_args->radio_channel) 
    {
        radiolinkSetChannel(radio_args->radio_channel);
        last_ch = radio_args->radio_channel;
    }
    
    while(len > 0)
    {
        to_write = (len <= CRTP_MAX_DATA_SIZE) ? len : CRTP_MAX_DATA_SIZE;
        memcpy(send_pkg.data, &buf[index], to_write);
        send_pkg.size = to_write;
        len -= to_write;
        index += to_write;

        while (!link->sendPacket(&send_pkg))
        {
            vTaskDelay(10);
        }
    }

    watchdogReset();

    return index;
}

size_t crazyflie_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    transport_args * radio_args = (transport_args*)transport->args;
    uint8_t * crtp_buffer = radio_args->crtp_buffer;
    size_t * crpt_index = &radio_args->crtp_index;

    static CRTPPacket recv_packet;
    size_t written;

    *err = 0;

    if(last_ch != radio_args->radio_channel) 
    {
        radiolinkSetChannel(radio_args->radio_channel);
        last_ch = radio_args->radio_channel;
    }

    size_t retrieved = 0;
    while(  *crpt_index + CRTP_MAX_DATA_SIZE < BUFFER_SIZE && 
            crtpReceivePacketWait(radio_args->radio_port, &recv_packet, timeout))
    {
        memcpy(&crtp_buffer[*crpt_index], recv_packet.data, recv_packet.size);
        *crpt_index += recv_packet.size;
        retrieved += recv_packet.size;
    
        if(retrieved > len)
        {
            break;
        }
    }

    if (*crpt_index + CRTP_MAX_DATA_SIZE > BUFFER_SIZE)
    {
        DEBUG_PRINT("MicroXRCEDDS CRTP Buffer full. Clearing buffer.\n");
        *crpt_index = 0;
        *err = 2;
    }

    if (len < *crpt_index)
    {
        memcpy(buf, crtp_buffer, len);
        memcpy(crtp_buffer, &crtp_buffer[len], *crpt_index - len);
        written = len;
        *crpt_index -= len;
    }
    else
    {
        memcpy(buf, crtp_buffer, *crpt_index);
        written = *crpt_index;
        *crpt_index = 0;
    }  

    watchdogReset();
    
    return written;
}