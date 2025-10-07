/**    HOW TO SETUP FOR ESP32S3
 * 1. clone to components directory:
 * 		git clone -b jazzy git@github.com:micro-ROS/micro_ros_espidf_component.git
 * 2. edit these files:
 * 		2.1 go to @file "colcon.meta" in micro_ros_espidf_component
 * 			line 14:
 * 				"rcutils": {
        		    "cmake-args": [
        		        "-DBUILD_SHARED=OFF",
        		        "-DENABLE_TESTING=OFF",
        		        "-DRCUTILS_NO_FILESYSTEM=ON",
        		        "-DRCUTILS_NO_THREAD_SUPPORT=ON",
        		        "-DRCUTILS_NO_64_ATOMIC=OFF",
        		        "-DRCUTILS_AVOID_DYNAMIC_ALLOCATION=ON"
        		    ]
        		},
 * 			line 24: 
 * 				"microxrcedds_client": {
   				  	"cmake-args": [
   				  	    "-DUCLIENT_PIC=OFF",
   				  	    "-DUCLIENT_PROFILE_DISCOVERY=ON",
   				  	    "-DUCLIENT_PROFILE_UDP=ON",
   				  	    "-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=ON",
   				  	    "-DUCLIENT_PROFILE_SERIAL=OFF",
   				  	    "-DUCLIENT_PROFILE_TCP=OFF",
   				  	    "-DUCLIENT_MIN_HEARTBEAT_TIME_INTERVAL=1"
   				  	]},
		2.2 go to @file "libmicroros.mk" in micro_ros_espidf_component and edit "esp32s2" to "esp32s3"
			line 114: 
				ifeq ($(IDF_TARGET),$(filter $(IDF_TARGET),esp32s3 esp32c3 esp32c6))
 * 5. Add "libmicroros.a" 
 * 4. Replace /components/micro_ros_espidf_component/include with old include folder 
  
 */



#ifndef CONFIG_H
#define CONFIG_H
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"


/**
 * ====================================
 *        	    GPIO Setup
 * ====================================
 */
#define GREEN_SW    38
#define BLUE_SW     39
#define ESP_INTR_FLAG_DEFAULT 0

int green_sw_state;
int blue_sw_state;
int prev_green_sw_state;
int prev_blue_sw_state;

int pressing_state = 0;

esp_err_t init_gpio_inputs()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GREEN_SW) | (1ULL << BLUE_SW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    return ESP_OK;
}

/**
 * ====================================
 *        	  UROS Setup
 * ====================================
 */


#define PROJECT_NAME ""
#define MODULE_NAME ""

#define MICRO_ROS_AGENT_IP "10.10.0.167"
#define MICRO_ROS_AGENT_PORT "8888"
#define EXECUTOR_HANDLE_NUMBER 10

#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)


/**
 * ====================================
 *        Watchdog Timer Setup
 * ====================================
 */


#include "watchdog_timer.h"
WatchdogTimer watchdogtimer;
WatchdogTimer watchdogtimer_restartESP;


/**
 * ====================================
 *        	  Encoder Setup
 * ====================================
 */
#define SPI_HOST    SPI2_HOST  // Use SPI2_HOST for ESP32-C6
#define PIN_MOSI    11
#define PIN_MISO    13
#define PIN_SCLK    14
#define PIN_CS      10        // Change this to your actual CS pin
#define SPI_CLOCK_SPEED_HZ 1000000 

#include "AS5X47.h"
spi_t enc;
float encoder_angle;
float encoder_angle_offset = 5.19;  // radian

void encoder_read()
{
	encoder_angle = AS5X47_readAngle(&enc) * 0.0174532925 - encoder_angle_offset;    // radian
	// ESP_LOGI("ENCODER", "angle: %f", encoder_angle);
}


/**
 * ====================================
 *          Dynamixel Setup
 * ====================================
 */
#include <dynamixel_sdk.h>  

int motor_ids[] = {5, 41};
int32_t goal_positions[] = {0, 0};
uint32_t present_positions[sizeof(motor_ids) / sizeof(motor_ids[0])] = {};
uint32_t present_currents[sizeof(motor_ids) / sizeof(motor_ids[0])] = {};


#define PROTOCOL_VERSION                2                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        1000000
#define DEVICENAME                      "UART1"      		  // Check which port is being used on your controller// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


// XM540-W270
// https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
#define UNIT_TO_RAD  (0.088f * M_PI / 180.0f)
#define CUR_UNIT_TO_TOR_NM (0.00269) * (5.0/2.1)

#define ADDR_OPERATING_MODE       11
#define OPERATING_MODE_EXTENDED_POSITION  4
#define ADDR_MOVING                 (122)                
#define ADDR_TORQUE_ENABLE          (64)                 // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION          (116)
#define ADDR_PRESENT_POSITION       (132)
#define ADDR_PRESENT_CURRENT        (126)

#define LEN_GOAL_POSITION           (4)
#define LEN_PRESENT_POSITION        (4)
#define LEN_PRESENT_CURRENT         (2)

#define ADDR_PING                   (1)
#define ADDR_REBOOT                 (8)
#define ADDR_RETURN_DELAY_TIME      (9)

// These also need to be looked up
#define DXL_MINIMUM_POSITION_VALUE      (0)             	// Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      (4096)          	// and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

#define TORQUE_ENABLE                   1                 // Value for enabling the torque
#define TORQUE_DISABLE                  0                 // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     50                // Dynamixel moving status threshold
#define ESC_ASCII_VALUE                 0x1b
#define COMMAND_RETRY                   (400)     			  // number of times to retry


// wrapper for a sleep function
void sleep_usecs(int usec) { esp_rom_delay_us(usec); }

int port_num;

#define ADDR_MIN_POSITION_LIMIT   48
#define ADDR_MAX_POSITION_LIMIT   52

void remove_position_limits(int port_num, int id)
{
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MIN_POSITION_LIMIT, -0x80000000);  // Very low min
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MAX_POSITION_LIMIT,  0x7FFFFFFF);  // Very high max
}

int set_torque_mode(int port_num, int id, bool enable) 
{
  uint8_t torque_result;
  int attempts = 0;
  int dxl_comm_result;  
  int dxl_error;

  do {
    sleep_usecs(5000);

    attempts++;
    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, enable ? TORQUE_ENABLE : TORQUE_DISABLE);

    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      // printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      // printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
    else
    {
      ESP_LOGI("DYNAMIXEL", "ID %d Torque %s", id, enable? "ENABLED" : "DISABLED"); 
    }

    sleep_usecs(5000);

    // check that torque is in the correct state
    torque_result = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      // printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      // printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
    else
    {
      // printf("read back torque, is: %d needs %d\n",torque_result,(int)enable);
    }

    if (enable && torque_result == TORQUE_ENABLE) 
    {
      ESP_LOGI("DYNAMIXEL", "ID %d Enabled torque, took %d attempts", id, attempts); 
      return(0);
    }
    if (!enable && torque_result == TORQUE_DISABLE) 
    {
      ESP_LOGI("DYNAMIXEL", "ID %d Disabled torque, took %d attempts", id, attempts); 
      return(0);
    }
 
  } while(attempts < COMMAND_RETRY);

  ESP_LOGE("DYNAMIXEL", "ID %d Attempted to set torque more times than retry standard, failing", id); 
  return(-1);
}

int motor_init(int port_num, int id) 
{
  uint8_t dxl_error = 0;    
  int dxl_comm_result = COMM_TX_FAIL;    

  ESP_LOGI("DYNAMIXEL", "ID %d Reboot ", id);
  sleep_usecs(10000);

  bool reboot_success = false;

  for (int i=0; i<3; i++) 
  {
    reboot(port_num, PROTOCOL_VERSION, id);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) 
	  {
      ESP_LOGI("DYNAMIXEL", "ID %d Communication failed: %s", id, getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      ESP_LOGI("DYNAMIXEL", "ID %d Reboot Error: %s", id, getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else 
	  {
      ESP_LOGI("DYNAMIXEL", "ID %d Reboot completed", id);
      reboot_success = true;
      sleep_usecs(150000);
      break;
    }
    sleep_usecs(150000);
  }

  if (reboot_success == false) { return(-1); }

  
  if (0 != set_torque_mode(port_num,id, true)) 
  {ESP_LOGI("DYNAMIXEL", "ID %d Enable torque failure: %s", id, getRxPacketError(PROTOCOL_VERSION, dxl_error));}


  return(0);
}

void set_operating_mode_extended(int port_num, int id)
{
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 0);  // Disable torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_OPERATING_MODE, OPERATING_MODE_EXTENDED_POSITION);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 1);  // Enable torque
}

esp_err_t dynamixel_init(const int* ids, int num_ids)
{
  // Initialize Dynamixel SDK
	port_num = portHandler(DEVICENAME);

	if (setBaudRate(port_num, BAUDRATE)) { ESP_LOGI("DYNAMIXEL", "Succeeded to change the baudrate! "); }
	else { ESP_LOGE("DYNAMIXEL", "Failed to change the baudrate! "); }

	if (openPort(port_num)) { ESP_LOGI("DYNAMIXEL", "Succeeded to open the port! ");}
	else{ ESP_LOGE("DYNAMIXEL", "Failed to open the port! ");}
  
	packetHandler();

  // Loop through all IDs and initialize each
  for (int i = 0; i < num_ids; i++) 
  {
    int id = ids[i];
    ESP_LOGI("DYNAMIXEL", "Initializing servo with ID %d", id);
    set_operating_mode_extended(port_num, id);      // <<-------------------------- Change mode HERE!!!
    remove_position_limits(port_num, id);          
    if (0 != motor_init(port_num, id)) 
    {
      ESP_LOGE("DYNAMIXEL", "Failed to initialize servo with ID %d", id);
    }
  }
  return ESP_OK;
}

int set_goal_position(const int* ids, const int32_t* goal_positions, int num_ids) 
{
  int dxl_comm_result;  
  int dxl_error;
  uint8_t dxl_addparam_result = false;

  // initalizae GroupSyncWrite Structs
  int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);


  for (int i = 0; i < num_ids; i++) 
  {
      int id = ids[i];
      uint32_t pos = goal_positions[i];
      dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, id, pos, LEN_GOAL_POSITION);
      if (!dxl_addparam_result) 
      {
          ESP_LOGE("DYNAMIXEL", "[ID:%d] groupSyncWrite addparam failed", id);
          groupSyncWriteClearParam(groupwrite_num);
          return -1;
      }
  }

  /*=================================================================================*/

  groupSyncWriteTxPacket(groupwrite_num);		// Send the goal position to the motors

  /*=================================================================================*/

  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) 
  {
    // printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    groupSyncWriteClearParam(groupwrite_num);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    // printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    groupSyncWriteClearParam(groupwrite_num);
  }
  groupSyncWriteClearParam(groupwrite_num);

  return 0;
}

int32_t get_position(int port_num, int id, uint32_t* present_position) 
{
  uint32_t pos    = read4ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_PRESENT_POSITION);
  int comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
  int err         = getLastRxPacketError(port_num, PROTOCOL_VERSION);

  if (comm_result != COMM_SUCCESS) 
  {
      // printf("ID %d: Comm failed: %s\n", 5, getTxRxResult(PROTOCOL_VERSION, comm_result));
  } 
  else if (err != 0) 
  {
      // printf("ID %d: Packet error: %s\n", 5, getRxPacketError(PROTOCOL_VERSION, err));
  } 
  else 
  {
      // printf("ID %d: Position %lu\n", 5, pos);
      *present_position = pos;
  }
  return 0;
}

int get_present_positions(int port_num, int* dxl_ids, int num_motors, uint32_t* positions) 
{
    for (int i = 0; i < num_motors; ++i) 
    {
        if (get_position(port_num, dxl_ids[i], &positions[i]) != 0) 
        {
            // Optional: handle failure here (e.g., retry or set to 0)
            // printf("Failed to read ID %d\n", dxl_ids[i]);
        }
    }
    return 0;
}


int32_t get_current(int port_num, int id, uint32_t* present_torque)
{
  uint32_t cur    = read4ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_PRESENT_CURRENT);
  int comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
  int err         = getLastRxPacketError(port_num, PROTOCOL_VERSION);

  if (comm_result != COMM_SUCCESS) 
  {
      // printf("ID %d: Comm failed: %s\n", 5, getTxRxResult(PROTOCOL_VERSION, comm_result));
  } 
  else if (err != 0) 
  {
      // printf("ID %d: Packet error: %s\n", 5, getRxPacketError(PROTOCOL_VERSION, err));
  } 
  else 
  {
      // printf("ID %d: current %lu\n", 5, cur);
      *present_torque = cur;
  }
  return 0;
}

int get_present_currents(int port_num, int* dxl_ids, int num_motors, uint32_t* torques)
{
    for (int i = 0; i < num_motors; ++i) 
    {
        if (get_current(port_num, dxl_ids[i], &torques[i]) != 0) 
        {
            // Optional: handle failure here (e.g., retry or set to 0)
            // printf("Failed to read ID %d\n", dxl_ids[i]);
        }
    }
    return 0;
}



float* traj;
int num_points;
#define PI 3.14159265358979323846

int generate_symmetric_trajectory(float start, float end, int steps, float** trajectory_out)
{
    if (steps <= 0) return -1;

    int total_steps = steps * 2;  // Forward + Backward
    float* trajectory = (float*)malloc(sizeof(float) * total_steps);
    if (!trajectory) return -1;

    // Forward trajectory: 0 to pi
    for (int i = 0; i < steps; ++i) {
        float t = (float)i / (steps - 1);
        trajectory[i] = start + (end - start) * t;
    }

    // Backward trajectory: pi to 0
    for (int i = 0; i < steps; ++i) {
        float t = (float)i / (steps - 1);
        trajectory[i + steps] = end - (end - start) * t;
    }

    *trajectory_out = trajectory;
    return total_steps;
}


int generate_sine_trajectory(float amplitude, float offset, int steps, float** trajectory_out)
{
    if (steps <= 1) return -1; // need at least 2 points

    float* trajectory = (float*)malloc(sizeof(float) * steps);
    if (!trajectory) return -1;

    for (int i = 0; i < steps; ++i) {
        float theta = (2.0f * PI * i) / (steps - 1);  // angle in [0, 2π]
        trajectory[i] = amplitude * sinf(theta) + offset;
    }

    *trajectory_out = trajectory;
    return steps;
}

#endif // CONFIG_H