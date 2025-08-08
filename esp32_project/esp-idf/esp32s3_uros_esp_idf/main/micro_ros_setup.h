#ifndef MICRO_ROS_SETUP
#define MICRO_ROS_SETUP
#include <config.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>		// if cannot find this file, please check ROS_DISTRO (humble)
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <rmw/qos_profiles.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joy.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

rmw_qos_profile_t custom_qos = rmw_qos_profile_default;

rcl_publisher_t module_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t board_connection_publisher;


rcl_subscription_t command_subscriber;

std_msgs__msg__Float32MultiArray clamp_wheel_module_state_msg;
std_msgs__msg__Float32MultiArray command_recv_msg;

std_msgs__msg__Bool connection_publisher_msg;



void command_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
	printf("Command messages are received:\n");

    #ifdef USED_CONNECTION_CHECK
        watchdog_timer_restart(&watchdogtimer);
        watchdog_timer_restart(&watchdogtimer_restartESP);
    #endif
}


void publish_clamp_wheel_module_feedback()
{
    clamp_wheel_module_state_msg.data.data[0] = (float)goal_positions[0] * UNIT_TO_RAD;
    clamp_wheel_module_state_msg.data.data[1] = (float)present_positions[0] * UNIT_TO_RAD;
    clamp_wheel_module_state_msg.data.data[2] = (float)goal_positions[1] * UNIT_TO_RAD;
    clamp_wheel_module_state_msg.data.data[3] = (float)encoder_angle;
    clamp_wheel_module_state_msg.data.data[4] = 5.0;
    clamp_wheel_module_state_msg.data.data[5] = 6.0;
    clamp_wheel_module_state_msg.data.data[6] = 7.0;
    RCSOFTCHECK(rcl_publish(&module_publisher, &clamp_wheel_module_state_msg, NULL));
}

void publisher_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) 
    {
        publish_clamp_wheel_module_feedback();
        RCSOFTCHECK(rcl_publish(&board_connection_publisher, &connection_publisher_msg, NULL));

	}
}



esp_err_t create_entities()
{
	allocator = rcl_get_default_allocator();

	// ========== Create init_options ==========
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_udp_address(MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT, rmw_options));
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));


	// ========== Create node ==========
    char node_name[64];
    snprintf(node_name, sizeof(node_name), "%s_%s_module", PROJECT_NAME, MODULE_NAME);
	RCCHECK(rclc_node_init_default(&node, node_name, "", &support));



    // ========== Create publishers ==========
    char topic_name[64];
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 5;  
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    snprintf(topic_name, sizeof(topic_name), "%s/%s/module_feedback", PROJECT_NAME, MODULE_NAME);
	RCCHECK(rclc_publisher_init(
		&module_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		topic_name,
        &custom_qos));
    ESP_LOGI("uROS", "init publisher: %s", topic_name);

    snprintf(topic_name, sizeof(topic_name), "%s/%s/imu_feedback", PROJECT_NAME, MODULE_NAME);
	RCCHECK(rclc_publisher_init(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		topic_name,
        &custom_qos));
    ESP_LOGI("uROS", "init publisher: %s", topic_name);

    snprintf(topic_name, sizeof(topic_name), "%s/%s/controller_connection", PROJECT_NAME, MODULE_NAME);
    RCCHECK(rclc_publisher_init_best_effort(
        &board_connection_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        topic_name));


    // ========== Create subscribers ==========
    snprintf(topic_name, sizeof(topic_name), "%s/%s/command", PROJECT_NAME, MODULE_NAME);
    RCCHECK(rclc_subscription_init_best_effort(
        &command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        topic_name));
    ESP_LOGI("uROS", "init subscriber: %s", topic_name);



    // ========== Create timer ==========
	timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 10;  
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		publisher_callback));

    // ========== Create executor ==========
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLE_NUMBER, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // ========== add subscription to executor ==========
	RCCHECK(rclc_executor_add_subscription(&executor, &command_subscriber, &command_recv_msg, &command_subscription_callback, ON_NEW_DATA));

    return ESP_OK;
}

void destroy_entities()
{
      rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	RCSOFTCHECK(rcl_publisher_fini(&module_publisher, &node));
	RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));

	RCSOFTCHECK(rcl_subscription_fini(&command_subscriber, &node));

    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rclc_support_fini(&support));
}

esp_err_t setup_multiarray_publisher_msg()
{
    size_t data_len;
    data_len = 7;
    clamp_wheel_module_state_msg.data.data = (float_t *)malloc(sizeof(float) * data_len);
    clamp_wheel_module_state_msg.data.size = data_len;
    clamp_wheel_module_state_msg.data.capacity = data_len;

    // data_len = 3;
    // motor_temperature_msg.data.data = (uint8_t *)malloc(sizeof(uint8_t) * data_len);
    // motor_temperature_msg.data.size = data_len;
    // motor_temperature_msg.data.capacity = data_len;

    // data_len = 3;
    // motor_connection_msg.data.data = (uint8_t *)malloc(sizeof(uint8_t) * data_len);
    // motor_connection_msg.data.size = data_len;
    // motor_connection_msg.data.capacity = data_len;

    // data_len = 3;
    // imu_msg.data.data = (float_t *)malloc(sizeof(float) * data_len);
    // imu_msg.data.size = data_len;
    // imu_msg.data.capacity = data_len;

    // data_len = 3;
    // force_sensors_msg.data.data = (float_t *)malloc(sizeof(float) * data_len);
    // force_sensors_msg.data.size = data_len;
    // force_sensors_msg.data.capacity = data_len;

    // data_len = 4;
    // motor_current_msg.data.data = (float_t *)malloc(sizeof(float) * data_len);
    // motor_current_msg.data.size = data_len;
    // motor_current_msg.data.capacity = data_len;

    // ====================================================================

    data_len = 3;
    command_recv_msg.data.capacity = data_len;  // set according to expected max size
    command_recv_msg.data.size = 0;
    command_recv_msg.data.data = (float_t *)malloc(sizeof(float) * command_recv_msg.data.capacity);
    
    return ESP_OK;
}



void reconnectMicroROS() 
{
    ESP_LOGE("uROS","Restarting WiFi and micro-ROS connection...");

	ESP_ERROR_CHECK_WITHOUT_ABORT(uros_network_interface_initialize());
    ESP_ERROR_CHECK_WITHOUT_ABORT(create_entities());
	ESP_ERROR_CHECK_WITHOUT_ABORT(setup_multiarray_publisher_msg());
    
    #ifdef USED_CONNECTION_CHECK
        watchdog_timer_restart(&watchdogtimer);
        watchdog_timer_restart(&watchdogtimer_restartESP);
    #endif
}

void connectionCheck()
{
  if (watchdog_timer_check_timeout(&watchdogtimer))
  {
    // ESP_LOGW("uROS","connection timeout : Robot will stop ");
    // Robot stop and show some status
  }

  if (watchdog_timer_check_timeout(&watchdogtimer_restartESP))
  {
    ESP_LOGE("uROS","connection timeout : ESP will be restart");
    destroy_entities();
    reconnectMicroROS();
  }
  else
  {
    // Show some status
  }
}


#endif // MICRO_ROS_SETUP