/**
 * This example takes a picture every 5s.
   Example src: https://github.com/espressif/esp32-camera.git 
 */




// ================================ CODE ======================================

#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/image.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "boards.h"

#define STRING_CAPACITY 20

static const char *TAG = "example:take_picture";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
sensor_msgs__msg__Image* msg;
camera_fb_t *pic;


static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_YUV422, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QQVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	UNUSED(last_call_time);
	if (timer != NULL) {
		
	}
}

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void appMain(void * arg)
{
    init_camera();
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	msg = sensor_msgs__msg__Image__create();
	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "freertos_picture_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
		"freertos_picture_publisher"));

	// create timer,
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	//takes picture in yuv422 format
	pic = esp_camera_fb_get();	
	
	//set height, width, step and is_bigedian
	msg->height = pic->height;
	msg->width = pic->width;
	msg->is_bigendian = 0;
	msg->step = pic->width * 2;

	//set pixel format and frame id
	char * encoding = "yuv422";
	msg->encoding.data = (char*)heap_caps_malloc(strlen(encoding) + 1, MALLOC_CAP_8BIT); 
	msg->header.frame_id.capacity = STRING_CAPACITY;
	memset(msg->encoding.data, 0, strlen(encoding) + 1);
	memcpy(msg->encoding.data, encoding, strlen(encoding));
	msg->encoding.size = strlen(encoding) + 1;

	char* frame_id = "AI_thinker_image";
	msg->header.frame_id.data = (char*)heap_caps_malloc(strlen(frame_id) + 1, MALLOC_CAP_8BIT);
	msg->header.frame_id.capacity = STRING_CAPACITY;
	memset(msg->header.frame_id.data, 0, strlen(frame_id) + 1);
	memcpy(msg->header.frame_id.data, frame_id, strlen(frame_id));
	msg->header.frame_id.size = strlen(frame_id) + 1;


	//allocate memory for data
	msg->data.data = (uint8_t*)heap_caps_malloc(pic->len, MALLOC_CAP_8BIT);
	msg->data.capacity = pic->len;
	msg->data.size = msg->step * pic->height ;
	printf("copying data...\n");
	if(msg->data.data == NULL){
		printf("failed to allocate memory. restating in 5 sec.\n");
		vTaskDelay(5000 / portTICK_RATE_MS);
		esp_restart();
	}	
	memcpy(msg->data.data, pic->buf, pic->len);

	//msg->header.stamp ?

	printf("height: %d \n", msg->height);
	printf("width: %d \n", msg->width);
	printf("pixel format %s \n", msg->encoding.data);
	printf("is bigendian %d \n", msg->is_bigendian);
	printf("picture step %d \n", msg->step);
	printf("frame id %s \n", msg->header.frame_id.data);
	printf("buffer length %d \n", msg->data.size);
	printf("pic->len %d \n", pic->len);

	
	while(1){
		rclc_executor_spin_some(&executor, 1000);
		usleep(100000);
		//publish picture
		printf("publisher return value: %d \n", rcl_publish(&publisher, msg, NULL));
       	vTaskDelay(1000 / portTICK_RATE_MS);
	}

    // free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
	heap_caps_free(msg->encoding.data);
	heap_caps_free(msg->header.frame_id.data);
	heap_caps_free(msg->data.data);
	sensor_msgs__msg__Image__destroy(msg);
	vTaskDelete(NULL);
}
