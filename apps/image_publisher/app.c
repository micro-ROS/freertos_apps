#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/image.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define BYTES_PER_PIXEL 4

rcl_publisher_t publisher;
sensor_msgs__msg__Image* msg;

void appMain(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	msg = sensor_msgs__msg__Image__create();
	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "freertos_image_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
		"freertos_image_publisher"));
	
	//set height, width, step and is_bigedian
	msg->height = 10;
	msg->width = 10;
	msg->is_bigendian = 0;
	msg->step = msg->width * msg->height * 2;

	//set pixel format and frame id
	char * encoding = "yuv422";
	msg->encoding.capacity = strlen(encoding) + 1;
	msg->encoding.data = (char*)malloc(msg->encoding.capacity); 
	memset(msg->encoding.data, 0, strlen(encoding) + 1);
	memcpy(msg->encoding.data, encoding, strlen(encoding));
	msg->encoding.size = strlen(encoding) + 1;

	char* frame_id = "AI_thinker_image";
	msg->header.frame_id.capacity = strlen(frame_id) + 1;
	msg->header.frame_id.data = (char*)malloc(msg->header.frame_id.capacity );
	memset(msg->header.frame_id.data, 0, strlen(frame_id) + 1);
	memcpy(msg->header.frame_id.data, frame_id, strlen(frame_id));
	msg->header.frame_id.size = strlen(frame_id) + 1;


	//allocate memory for data
	msg->data.capacity = msg->height *  msg->width * BYTES_PER_PIXEL;
	printf("allocating %d\n", msg->data.capacity);
	msg->data.data = (uint8_t*)malloc(msg->data.capacity);
	msg->data.size = msg->data.capacity;//msg->step * pic->height ;
	printf("copying data...\n");
	// FAKE SETTING OF THE IMAGE DATA
	memset(msg->data.data, 0xAA, msg->data.capacity);

	printf("height: %d \n", msg->height);
	printf("width: %d \n", msg->width);
	printf("pixel format %s \n", msg->encoding.data);
	printf("is bigendian %d \n", msg->is_bigendian);
	printf("picture step %d \n", msg->step);
	printf("frame id %s \n", msg->header.frame_id.data);
	printf("buffer length %d \n", msg->data.size);
	printf("pic->len %d \n", msg->data.size);

	while(1){
		printf("publisher return value: %d \n", rcl_publish(&publisher, msg, NULL));
       	vTaskDelay(1000 / portTICK_RATE_MS);
	}

    // free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
	free(msg->encoding.data);
	free(msg->header.frame_id.data);
	free(msg->data.data);
	sensor_msgs__msg__Image__destroy(msg);
	vTaskDelete(NULL);
}