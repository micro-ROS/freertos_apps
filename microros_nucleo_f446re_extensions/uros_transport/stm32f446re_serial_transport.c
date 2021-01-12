#include <uxr/client/profile/transport/serial/serial_transport_external.h>
#include "stm32f4xx_hal_dma.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

// static uint8_t * crtp_buffer = (uint8_t *)0x10000000;

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{
//   switch ( fd ){
//       case 3:
//         platform->uart = &huart3;
//         break;
//       default:
//         return false;
//   }
  platform->uart = &huart2;
  HAL_UART_Receive_DMA(platform->uart, dma_buffer, UART_DMA_BUFFER_SIZE);
  return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{   
  HAL_UART_DMAStop(platform->uart);
  return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
  
  HAL_StatusTypeDef ret;

  if (platform->uart->gState == HAL_UART_STATE_READY){
    ret = HAL_UART_Transmit_DMA(platform->uart, buf, len);
    while (ret == HAL_OK && platform->uart->gState != HAL_UART_STATE_READY){
      osDelay(1);
    }

    return (ret == HAL_OK) ? len : 0;
  }else{
    return 0;
  }
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{ 
  osDelay(timeout);

  __disable_irq();
  dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(platform->uart->hdmarx);
  __enable_irq();

  size_t wrote = 0;
  while ((dma_head != dma_tail) && (wrote < len)){
    buf[wrote] = dma_buffer[dma_head];
    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
    wrote++;
  }
 
  return wrote;
}
