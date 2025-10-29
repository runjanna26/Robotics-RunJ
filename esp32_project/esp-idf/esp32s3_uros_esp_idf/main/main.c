/**
 * Task List:
 * [/] Micro-ROS pub/sub
 * [/] Reconnect when its lost
 * [/] RS485 with Dynamixel motors 
 * [/] CAN BUS with motors
 * [/] Encoder
 * [/] Muscle Model
 * [/] Receive Center Command
 * [ ] CPG-RBF
 * [ ] IMU
 * [ ] Camera read
 * [ ] Force Sensors
 */

#define USED_UROS 
// #define USED_CONNECTION_CHECK 
// #define USED_DYNAMIXEL 
// #define USED_ENCODER 
// #define USED_LIMIT_SWITCHES
#define USED_RMD_MOTOR
#define USED_MUSCLE_MODEL


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


#ifdef USED_RMD_MOTOR
    ESP_ERROR_CHECK_WITHOUT_ABORT(CAN_Init());                	// CAN Initialization
	motor_reboot(&hip_motor_st);  										// Reboot motor with ID 1
#endif

#ifdef USED_MUSCLE_MODEL
	MuscleModel_init(&Muscle_1, 0.5, 10.0, 0.05);
#endif

	// num_points = generate_sine_trajectory(_PI/6, 0.0, 1000, &traj);  // radians 7.85
}

int i = 0;
bool started = false;
bool runned = false;


void run_motor()
{


	MuscleModel_calculate(&Muscle_1, pos_des, vel_des, hip_motor_st.position, hip_motor_st.velocity);
	send_mit_force_command(&hip_motor_st, RMD_X4_10, pos_des, vel_des, Muscle_1.K, Muscle_1.D, -Muscle_1.F);
	
	motor_update(&hip_motor_st);
    while (twai_receive(&msg_rx, pdMS_TO_TICKS(5)) == ESP_OK)  
    {  
        unpack_reply(msg_rx, &hip_motor_st, RMD_X4_10);
    }
}

void loop()
{
	
	EXECUTE_EVERY_N_MS(1, run_motor()); 




#ifdef USED_LIMIT_SWITCHES
	green_sw_state 	= gpio_get_level(GREEN_SW);
    blue_sw_state 	= gpio_get_level(BLUE_SW);

	if (green_sw_state == 0 && prev_green_sw_state == 1) 
	{
		started = true;
		i=0;
	}

	if (blue_sw_state == 0 && prev_blue_sw_state == 1) 
	{
	}

	prev_green_sw_state = green_sw_state;
	prev_blue_sw_state = blue_sw_state;
#endif

#ifdef USED_DYNAMIXEL

	// if (i  >= num_points)
	// 	i=0;

	// pos_des = traj[i];  // Set goal position 
	// i++;


	if (started)
		i++;
	if (i  >= num_points)
		started = false;
	ESP_LOGI("DYNAMIXEL", "%d", i);
	goal_positions[0] = -1*(int32_t)(traj[i]/UNIT_TO_RAD);  // Set goal position 
	// goal_positions[1] += (int32_t)(0.174532925/UNIT_TO_RAD);  // Set goal position
	set_goal_position(motor_ids, goal_positions, sizeof(motor_ids) / sizeof(motor_ids[0]));

	EXECUTE_EVERY_N_MS(10, get_present_positions(port_num, motor_ids, sizeof(motor_ids) / sizeof(motor_ids[0]), present_positions));
	// ESP_LOGI("DYNAMIXEL", "Present position 0 %.3f rad", (float)present_positions[0] * UNIT_TO_RAD);
	// ESP_LOGI("DYNAMIXEL", "Present position 1 %.3f rad", (float)present_positions[1] * UNIT_TO_RAD);
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
		// vTaskDelay(1);
	}
#ifdef USED_DYNAMIXEL
	closePort(port_num);
#endif

}