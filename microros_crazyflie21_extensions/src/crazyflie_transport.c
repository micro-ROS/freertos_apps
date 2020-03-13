#include <uxr/client/profile/transport/serial/serial_transport_external.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "crtp.h"
#include "debug.h"

#define BUFFER_SIZE 10000

static size_t crpt_index = 0;
static size_t crpt_index_max = 0;
// static uint8_t crtp_buffer[BUFFER_SIZE];
static uint8_t * crtp_buffer = (uint8_t *)0x10000000;

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    //TODO: For this RTOS is not necessary to initialize nothing here
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

  send_pkg.header = CRTP_HEADER(8, 0);

  while(len > 0){
    to_write = (len <= CRTP_MAX_DATA_SIZE) ? len : CRTP_MAX_DATA_SIZE;
    memcpy(send_pkg.data,&buf[index],to_write);
    send_pkg.size = to_write;
    len -= to_write;
    index += to_write;
    crtpSendPacket(&send_pkg);
  }

  return index;
}

void hexprint(uint8_t* buf, size_t len){
  for (size_t i = 0; i < len; i++) { DEBUG_PRINT("%X ",buf[i]); }
  DEBUG_PRINT("\n");  
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
  static CRTPPacket recv_packet;
  size_t index = 0;
  size_t written;

  size_t retrieved = 0;

  *errcode = 0;

  while(crpt_index + CRTP_MAX_DATA_SIZE < BUFFER_SIZE && crtpReceivePacketWait(8, &recv_packet, timeout)){
    memcpy(&crtp_buffer[crpt_index],recv_packet.data,recv_packet.size);
    crpt_index += recv_packet.size;
    retrieved += recv_packet.size;
  }

  if (crpt_index + CRTP_MAX_DATA_SIZE > BUFFER_SIZE)
  {
    DEBUG_PRINT("MicroXRCEDDS CRTP Buffer full. Clearing buffer.\n");
    crpt_index = 0;
    crpt_index_max = 0;
    *errcode = 2;
  }

  if (crpt_index > crpt_index_max)
  {
    crpt_index_max = crpt_index;
  }

  if (len < crpt_index)
  {
    memcpy(buf,crtp_buffer,len);
    memcpy(crtp_buffer,&crtp_buffer[len],crpt_index-len);
    written = len;
    crpt_index -= len;
  }else
  {
    memcpy(buf,crtp_buffer,crpt_index);
    written = crpt_index;
    crpt_index = 0;
  }  
 
  // DEBUG_PRINT("Retrived %d Flushed %d Max %d Current %d\n", retrieved, written, crpt_index_max, crpt_index);
  return written;
}
