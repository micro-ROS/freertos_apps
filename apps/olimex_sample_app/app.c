#include <allocators.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include "example_interfaces/srv/add_two_ints.h"

#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);  vTaskDelete(NULL);;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void * trigger_guard_condition(void *args){
  rcl_guard_condition_t * guard_condition = (rcl_guard_condition_t *)args;

  while(true){
    RCSOFTCHECK(rcl_trigger_guard_condition(guard_condition))
    sleep(1);
  }
}

void appMain(void *argument)
{
  
  printf("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());

  rcl_init_options_t options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

  // Optional RMW configuration 
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
  RCCHECK(rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options))

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(0, NULL, &options, &context))

  rcl_node_options_t node_ops = rcl_node_get_default_options();

  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "int32_publisher_rcl", "", &context, &node_ops))

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/olimex/publisher", &publisher_ops))

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/olimex/subscriber", &subscription_ops))

  rcl_guard_condition_t guard_condition = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  RCCHECK(rcl_guard_condition_init(&guard_condition, &context, guard_condition_options))

  pthread_t guard_condition_thread;
  pthread_create(&guard_condition_thread, NULL, trigger_guard_condition, &guard_condition);

  const char * service_name = "addtwoints";
  rcl_service_options_t service_op = rcl_service_get_default_options();
  rcl_service_t serv = rcl_get_zero_initialized_service();
  const rosidl_service_type_support_t * service_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  RCCHECK(rcl_service_init(&serv, &node, service_type_support, service_name, &service_op))

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 1, 1, 0, 0, 1, 0, &context, rcl_get_default_allocator()))

  std_msgs__msg__Int32 msg;
  const int num_msg = 1000;
  msg.data = 0;
  
  printf("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
  printf("uROS Used Memory %d bytes\n", usedMemory);
  printf("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

  rcl_ret_t rc;
  do {
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
    size_t index_subscription;
    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription, &index_subscription))

    size_t index_guardcondition;
    RCSOFTCHECK(rcl_wait_set_add_guard_condition(&wait_set, &guard_condition, &index_guardcondition))

    size_t index_service;
    RCSOFTCHECK(rcl_wait_set_add_service(&wait_set, &serv, &index_service))
    
    RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(100)))

    if (wait_set.subscriptions[index_subscription]) {
      std_msgs__msg__Int32 msg;

      rc = rcl_take(wait_set.subscriptions[index_subscription], &msg, NULL, NULL);
      if (RCL_RET_OK == rc) {
        printf("I received: [%i]\n", msg.data);
      }
    }

    if (wait_set.guard_conditions[index_guardcondition]) {
      // Use the guard condition trigger to publish in a slower loop

      rc = rcl_publish(&publisher, (const void*)&msg, NULL);
      if (RCL_RET_OK == rc ) {
          printf("Sent: '%i'\n", msg.data++);
      }
    }

    if (wait_set.services[index_service]) {   
      rmw_request_id_t req_id;
      example_interfaces__srv__AddTwoInts_Request req;
      example_interfaces__srv__AddTwoInts_Request__init(&req);
      RCSOFTCHECK(rcl_take_request(&serv,&req_id,&req))

      printf("Service request value: %d + %d. Seq %d\n", (int)req.a, (int)req.b, (int) req_id.sequence_number);

      example_interfaces__srv__AddTwoInts_Response res;
      example_interfaces__srv__AddTwoInts_Response__init(&res);
      
      res.sum = req.a + req.b;

      RCSOFTCHECK(rcl_send_response(&serv,&req_id,&res))
    }
    usleep(10000);
  } while (true );
  printf("TOTAL sent: %i\n", num_msg);

  RCCHECK(rcl_publisher_fini(&publisher, &node))
  RCCHECK(rcl_node_fini(&node))

  vTaskDelete(NULL);
}