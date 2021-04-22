#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <stdio.h>

#include "FreeRTOS.h"

#define ARRAY_LEN 200
#define JOINT_DOUBLE_LEN 20

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t joint_states_subscriber;
sensor_msgs__msg__JointState joint_states_msg;

void subscription_joint_state_callback(const void *msgin){
	const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
	printf("I get joint_state topic msg.pos: %d \r\n", (int)(msg->position.data[0] * 100));
}

int appMain(void *argument)
{

  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "string_node", "", &support));

	// create joint_state subscriber
	RCCHECK(rclc_subscription_init_default(
		&joint_states_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"/joint_states"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &joint_states_subscriber, &joint_states_msg, &subscription_joint_state_callback, ON_NEW_DATA));

	char joint_states_msg_buffer[ARRAY_LEN];
	joint_states_msg.header.frame_id.data = joint_states_msg_buffer;
	joint_states_msg.header.frame_id.size = 20;
	joint_states_msg.header.frame_id.capacity = ARRAY_LEN;

	rosidl_runtime_c__String string_buffer[JOINT_DOUBLE_LEN];
	joint_states_msg.name.data = string_buffer;
	joint_states_msg.name.size = 0;
	joint_states_msg.name.capacity = JOINT_DOUBLE_LEN;

	for(int i = 0; i < JOINT_DOUBLE_LEN; i++){
		joint_states_msg.name.data[i].data = (char*) malloc(ARRAY_LEN);
		joint_states_msg.name.data[i].size = 0;
		joint_states_msg.name.data[i].capacity = ARRAY_LEN;
	}

	double joint_states_position_buffer[JOINT_DOUBLE_LEN];
	joint_states_msg.position.data = joint_states_position_buffer;
	joint_states_msg.position.size= 7;
	joint_states_msg.position.capacity = JOINT_DOUBLE_LEN;

	double joint_states_velocity_buffer[JOINT_DOUBLE_LEN];	
	joint_states_msg.velocity.data = joint_states_velocity_buffer;
	joint_states_msg.velocity.size = 7;
	joint_states_msg.velocity.capacity = JOINT_DOUBLE_LEN;
	
	double joint_states_effort_buffer[JOINT_DOUBLE_LEN];	
	joint_states_msg.effort.data = joint_states_effort_buffer;
	joint_states_msg.effort.size = 7;
	joint_states_msg.effort.capacity = JOINT_DOUBLE_LEN;
	
	rclc_executor_spin(&executor);

	RCCHECK(rcl_subscription_fini(&joint_states_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
