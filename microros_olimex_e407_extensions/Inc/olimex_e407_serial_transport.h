// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _UXR_CLIENT_SERIAL_TRANSPORT_OLIMEXSTM32E407_H_
#define _UXR_CLIENT_SERIAL_TRANSPORT_OLIMEXSTM32E407_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_rx;

typedef struct uxrSerialPlatform
{
    UART_HandleTypeDef * uart;
} uxrSerialPlatform;

#ifdef __cplusplus
}
#endif

#endif //_UXR_CLIENT_SERIAL_TRANSPORT_OLIMEXSTM32E407_H_