#include <allocators.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include "rosidl_generator_c/string_functions.h"
#include <std_msgs/msg/header.h>

#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define STRING_BUFFER_LEN 100

// FreeRTOS thread for triggering a publication guard condition
void * trigger_guard_condition(void *args){
  rcl_guard_condition_t * guard_condition = (rcl_guard_condition_t *)args;

  while(true){
    rcl_trigger_guard_condition(guard_condition);
    sleep(5);
  }
}

// App main function
void appMain(void *argument)
{
  
  //Init RCL options
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&options, rcl_get_default_allocator());

  // Set Micro-XRCE-DDS client key
  // rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
  // rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options);
  
  // Init RCL context
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init(0, NULL, &options, &context);

  // Create a node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_init(&node, "pingpong", "", &context, &node_ops);

  // Create a ping ping_publisher
  rcl_publisher_options_t ping_publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t ping_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping", &ping_publisher_ops);

  // Create a ping pong_publisher
  rcl_publisher_options_t pong_publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t pong_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong", &pong_publisher_ops);

  // Create a pong subscriber
  rcl_subscription_options_t pong_subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t pong_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&pong_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong", &pong_subscription_ops);

  // Create a ping subscriber
  rcl_subscription_options_t ping_subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t ping_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&ping_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping", &ping_subscription_ops);

  // Create a guard condition
  rcl_guard_condition_t guard_condition = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_guard_condition_init(&guard_condition, &context, guard_condition_options);
  
  // Create a thread that triggers the guard condition
  pthread_t guard_condition_thread;
  pthread_create(&guard_condition_thread, NULL, trigger_guard_condition, &guard_condition);

  // Create a wait set
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_wait_set_init(&wait_set, 2, 1, 0, 0, 0, 0, &context, rcl_get_default_allocator());

  // Create and allocate the pingpong message
  std_msgs__msg__Header msg;
  
  char msg_buffer[STRING_BUFFER_LEN];
  msg.frame_id.data = msg_buffer;
  msg.frame_id.capacity = STRING_BUFFER_LEN;

  std_msgs__msg__Header rcv_msg;

  char rcv_buffer[STRING_BUFFER_LEN];
  rcv_msg.frame_id.data = rcv_buffer;
  rcv_msg.frame_id.capacity = STRING_BUFFER_LEN;

  int device_id = rand();
  int seq_no = rand();
  sprintf(msg.frame_id.data, "%d", seq_no);
  msg.frame_id.size = strlen(msg.frame_id.data);
  
  // frame_id should contain node identity
  
  int pong_count = 0;
  struct timespec ts;
  rcl_ret_t rc;

  do {
    // Clear and set the waitset
    rcl_wait_set_clear(&wait_set);
    
    size_t index_pong_subscription;
    rcl_wait_set_add_subscription(&wait_set, &pong_subscription, &index_pong_subscription);

    size_t index_ping_subscription;
    rcl_wait_set_add_subscription(&wait_set, &ping_subscription, &index_ping_subscription);
    
    size_t index_guardcondition;
    rcl_wait_set_add_guard_condition(&wait_set, &guard_condition, &index_guardcondition);
    
    // Run session for 100 ms
    rcl_wait(&wait_set, RCL_MS_TO_NS(100));

    // Check if it is time to send a ping
    if (wait_set.guard_conditions[index_guardcondition]) {
      seq_no = rand();
      sprintf(msg.frame_id.data, "%d_%d", seq_no, device_id);
      msg.frame_id.size = strlen(msg.frame_id.data);
      
      clock_gettime(CLOCK_REALTIME, &ts);

      msg.stamp.sec = ts.tv_sec;
      msg.stamp.nanosec = ts.tv_nsec;

      pong_count = 0;
      rcl_publish(&ping_publisher, (const void*)&msg, NULL);
      // printf("Ping send seq 0x%x\n", seq_no);
    }
    
    // Check if some pong is received
    if (wait_set.subscriptions[index_pong_subscription]) {
      rc = rcl_take(wait_set.subscriptions[index_pong_subscription], &rcv_msg, NULL, NULL);

      if(rc == RCL_RET_OK && strcmp(msg.frame_id.data,rcv_msg.frame_id.data) == 0) {
          pong_count++;
          // printf("Pong for seq 0x%x (%d)\n", seq_no, pong_count);
      }
    }

    // Check if some ping is received and answer it
    if (wait_set.subscriptions[index_ping_subscription]) {
      rc = rcl_take(wait_set.subscriptions[index_ping_subscription], &rcv_msg, NULL, NULL);

      // Dont pong my own pings
      if(rc == RCL_RET_OK && strcmp(msg.frame_id.data,rcv_msg.frame_id.data) != 0){
        // printf("Ping received with seq 0x%x (%d). Answering.\n", seq_no);
        rcl_publish(&pong_publisher, (const void*)&rcv_msg, NULL);
      }
    }
    
    usleep(10000);
  } while (true);
}