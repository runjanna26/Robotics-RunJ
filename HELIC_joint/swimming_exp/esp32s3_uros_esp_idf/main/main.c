/**
 * Task List:
 * [/] Micro-ROS pub/sub
 * [/] Reconnect when its lost
 * [/] RS485 with Dynamixel motors 
 * [ ] CAN BUS with AK motors
 * [ ] IMU
 * [ ] Camera read
 * [/] Encoder
 * [ ] Force Sensors
 * [ ] Current Sensors
 */

// #define USED_UROS 
// #define USED_CONNECTION_CHECK 
#define USED_DYNAMIXEL 
// #define USED_ENCODER 
// #define USED_LIMIT_SWITCHES

#include <config.h>
#include <micro_ros_setup.h>
                                // Uses Dynamixel SDK library

void setup()
{
    #ifdef USED_UROS
		ESP_ERROR_CHECK_WITHOUT_ABORT(uros_network_interface_initialize());
		ESP_ERROR_CHECK_WITHOUT_ABORT(create_entities());
		ESP_ERROR_CHECK_WITHOUT_ABORT(setup_multiarray_publisher_msg());

		watchdog_timer_init(&watchdogtimer, 200); 				// Timeout after 200 ticks
		watchdog_timer_init(&watchdogtimer_restartESP, 1000); 	// Timeout after 1000 ticks
	#endif

	ESP_ERROR_CHECK_WITHOUT_ABORT(init_gpio_inputs());

#ifdef USED_ENCODER
	ESP_ERROR_CHECK_WITHOUT_ABORT(AS5X47_init(&enc, SPI_HOST, 
													PIN_MOSI, 
													PIN_MISO, 
													PIN_SCLK, 
													PIN_CS, 
													SPI_CLOCK_SPEED_HZ));
#endif

#ifdef USED_LIMIT_SWITCHES
	ESP_ERROR_CHECK_WITHOUT_ABORT(init_gpio_inputs());
#endif

#ifdef USED_DYNAMIXEL
	ESP_ERROR_CHECK_WITHOUT_ABORT(dynamixel_init(motor_ids, sizeof(motor_ids) / sizeof(motor_ids[0])));
#endif
	usleep(100000);

	// generate_symmetric_trajectory(0.0f, 11.78, 5000, &traj);  // radians 7.85
	num_points = generate_sine_trajectory(PI/4, PI/4, 150, &traj);
}


void loop()
{

#ifdef USED_LIMIT_SWITCHES
	green_sw_state 	= gpio_get_level(GREEN_SW);
    blue_sw_state 	= gpio_get_level(BLUE_SW);

	if (green_sw_state == 0 && prev_green_sw_state == 1) 
	{
	}

	if (blue_sw_state == 0 && prev_blue_sw_state == 1) 
	{
	}

	prev_green_sw_state = green_sw_state;
	prev_blue_sw_state = blue_sw_state;
#endif

#ifdef USED_DYNAMIXEL
	goal_positions[0] = 1*(int32_t)(traj[i]/UNIT_TO_RAD);  	// Set goal position  (+) Pulling , (-) Pushing
	// set_goal_position(motor_ids, goal_positions, sizeof(motor_ids) / sizeof(motor_ids[0]));

	EXECUTE_EVERY_N_MS(1, update_i());


	EXECUTE_EVERY_N_MS(10, get_present_positions(port_num, motor_ids, sizeof(motor_ids) / sizeof(motor_ids[0]), present_positions));
	
	EXECUTE_EVERY_N_MS(10, get_present_currents(port_num, motor_ids, sizeof(motor_ids) / sizeof(motor_ids[0]), present_currents));


	ESP_LOGI("DYNAMIXEL", "Goal position 0 %.3f rad", (float)goal_positions[0] * UNIT_TO_RAD);
	ESP_LOGI("DYNAMIXEL", "Present position 0 %.3f rad", (float)present_positions[0] * UNIT_TO_RAD);
	ESP_LOGI("DYNAMIXEL", "Present Torque 0 %f Nm", (float)(present_currents[0])*(2.69e-3)/(4094.0f));


	sleep_usecs(5000);
#endif



#ifdef USED_ENCODER
    EXECUTE_EVERY_N_MS(10, encoder_read());
#endif

	// micro-ros loop
#ifdef USED_CONNECTION_CHECK
	EXECUTE_EVERY_N_MS(1, connectionCheck()); 
	connection_publisher_msg.data = true;
#endif

#ifdef USED_UROS
	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); 
#endif

}





void app_main(void)
{
	setup();
	while(true)
	{
		loop();
	}
#ifdef USED_DYNAMIXEL
	closePort(port_num);
#endif

}