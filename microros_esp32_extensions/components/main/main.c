
#include "app.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

void app_main(void)
{
    xTaskCreate(appMain, "uros_task", 12*2048, NULL, 5, NULL);
}
