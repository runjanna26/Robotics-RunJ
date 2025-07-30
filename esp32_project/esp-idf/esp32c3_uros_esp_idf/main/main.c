/**
 * Task List:
 * [/] Micro-ROS pub/sub
 * [/] Reconnect when its lost
 * [ ] CAN BUS with AK motors
 * [ ] IMU
 * [ ] Camera read
 * [ ] Encoder
 * [ ] Force Sensors
 * [ ] Current Sensors
 */

#include <config.h>
#include <micro_ros_setup.h>

void setup()
{
	ESP_ERROR_CHECK_WITHOUT_ABORT(uros_network_interface_initialize());
    ESP_ERROR_CHECK_WITHOUT_ABORT(create_entities());
	ESP_ERROR_CHECK_WITHOUT_ABORT(setup_multiarray_publisher_msg());

	watchdog_timer_init(&watchdogtimer, 200); 				// Timeout after 200 ticks
    watchdog_timer_init(&watchdogtimer_restartESP, 1000); 	// Timeout after 1000 ticks


}

void loop()
{

//     #ifdef USED_CONNECTION_CHECK
// 		EXECUTE_EVERY_N_MS(1, connectionCheck()); 
// 		connection_publisher_msg.data = true;
// 	#endif

// 	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); 
// printf("Micro-ROS is running...\n");
}





void app_main(void)
{
	setup();
	while(1)
	{
		loop();
	}
}