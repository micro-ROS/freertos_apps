/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <geometry_msgs/msg/point32.h>
#include "example_interfaces/srv/add_two_ints.h"
#include <geometry_msgs/msg/twist.h>

#include <rcutils/allocator.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include "radiolink.h"
#include <time.h>

#include "microrosapp.h"

#include "crtp.h"
#include "configblock.h"

#define RCCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); goto clean;}
#define RCSOFTCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}


void appMain(){ 

    CRTPPacket pkg_ch0, pkg_ch1, recv_packet;

    pkg_ch0.header = CRTP_HEADER(9,0);
    pkg_ch1.header = CRTP_HEADER(10,0);

    crtpInitTaskQueue(9);
    crtpInitTaskQueue(10);

    pkg_ch0.size = 1;
    pkg_ch1.size = 1;
    
    pkg_ch0.data[0] = 0;
    pkg_ch1.data[0] = 255;

    uint8_t channel = configblockGetRadioChannel();
    uint8_t secondary_channel = 30;

    while(1){
        absoluteUsedMemory = 0;
        usedMemory = 0;

        // ####################### RADIO INIT #######################

        int radio_connected = logGetVarId("radio", "isConnected");
        while(!logGetUint(radio_connected)) vTaskDelay(100);
        DEBUG_PRINT("Radio connected\n");

        // ####################### LOOP INIT #######################

        while(logGetUint(radio_connected)){
            // DEBUG_PRINT("Sending: %d on p9 and %d on p10\n",pkg_ch0.data[0],pkg_ch1.data[0]);

            radiolinkSetChannel(secondary_channel);
            vTaskDelay(100/portTICK_RATE_MS);
            crtpSendPacket(&pkg_ch0);
            if(crtpReceivePacketWait(9, &recv_packet, 100)){
                DEBUG_PRINT("Received on port 9 data:%d\n",recv_packet.data[0]);
            }

            radiolinkSetChannel(channel);
            vTaskDelay(100/portTICK_RATE_MS);
            crtpSendPacket(&pkg_ch1);
            if(crtpReceivePacketWait(10, &recv_packet, 100)){
                DEBUG_PRINT("Received on port 10 data:%d\n",recv_packet.data[0]);
            }

            vTaskDelay(1000/portTICK_RATE_MS);
        }
    }

    vTaskSuspend( NULL );
}