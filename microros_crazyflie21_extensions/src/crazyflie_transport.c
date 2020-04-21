#include <uxr/client/profile/transport/serial/serial_transport_external.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <watchdog.h>
#include "configblock.h"
#include "crtp.h"

#include "crtp.h"
#include "debug.h"

#define BUFFER_SIZE 10000
#define PRIMARY_CHANNEL_PORT 9
#define SECONDARY_CHANNEL_PORT 10

static size_t crpt_primary_index = 0;
static size_t crpt_secondary_index = 0;

static size_t crpt_primary_index_max = 0;
static size_t crpt_secondary_index_max = 0;

static uint8_t * crtp_buffer_primary   = (uint8_t *)0x10000000;
static uint8_t * crtp_buffer_secondary = (uint8_t *)0x10000000 + BUFFER_SIZE;

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    platform->radio_channel = fd;
    platform->default_radio_channel = configblockGetRadioChannel();
    platform->primary_channel = platform->radio_channel == platform->default_radio_channel;
    if(platform->primary_channel){
      crtpInitTaskQueue(PRIMARY_CHANNEL_PORT);
    }else{
      crtpInitTaskQueue(SECONDARY_CHANNEL_PORT);
    }
    
    return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{
    //TODO Is not necessary to close or open the platform on this fork of freeRTOS
    return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
  
  CRTPPacket send_pkg;
  size_t index = 0;
  size_t to_write;
  
  send_pkg.header = CRTP_HEADER((platform->primary_channel) ? PRIMARY_CHANNEL_PORT : SECONDARY_CHANNEL_PORT, 0);
  
  while(len > 0){
    to_write = (len <= CRTP_MAX_DATA_SIZE) ? len : CRTP_MAX_DATA_SIZE;
    memcpy(send_pkg.data,&buf[index],to_write);
    send_pkg.size = to_write;
    len -= to_write;
    index += to_write;

    if(!platform->primary_channel) radiolinkSetChannel(platform->radio_channel);
    crtpSendPacket(&send_pkg);
    if(!platform->primary_channel) radiolinkSetChannel(platform->default_radio_channel);
  }

  watchdogReset();

  return index;
}

void hexprint(uint8_t* buf, size_t len){
  for (size_t i = 0; i < len; i++) { DEBUG_PRINT("%X ",buf[i]); }
  DEBUG_PRINT("\n");  
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
  static CRTPPacket recv_packet;
  size_t written;

  // size_t retrieved = 0;

  *errcode = 0;

  uint8_t * crtp_buffer = (platform->primary_channel) ? crtp_buffer_primary : crtp_buffer_secondary;
  uint8_t * crpt_index = (platform->primary_channel) ? &crpt_primary_index : &crpt_secondary_index;
  uint8_t * crpt_index_max = (platform->primary_channel) ? &crpt_primary_index_max : &crpt_secondary_index_max;

  if(!platform->primary_channel) radiolinkSetChannel(platform->radio_channel);

  while(  *crpt_index + CRTP_MAX_DATA_SIZE < BUFFER_SIZE && 
          crtpReceivePacketWait((platform->primary_channel) ? PRIMARY_CHANNEL_PORT : SECONDARY_CHANNEL_PORT, &recv_packet, timeout))
  {
    memcpy(&crtp_buffer[*crpt_index], recv_packet.data, recv_packet.size);
    *crpt_index += recv_packet.size;
    // retrieved += recv_packet.size;
  }

  if(!platform->primary_channel) radiolinkSetChannel(platform->default_radio_channel);


  if (*crpt_index + CRTP_MAX_DATA_SIZE > BUFFER_SIZE){
    DEBUG_PRINT("MicroXRCEDDS CRTP Buffer full. Clearing buffer.\n");
    *crpt_index = 0;
    *crpt_index_max = 0;
    *errcode = 2;
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
