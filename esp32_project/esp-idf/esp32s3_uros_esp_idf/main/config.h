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
   				  	]}
		2.2 go to @file "libmicroros.mk" in micro_ros_espidf_component and edit "esp32s2" to "esp32s3"
			line 114: 
				ifeq ($(IDF_TARGET),$(filter $(IDF_TARGET),esp32s3 esp32c3 esp32c6))
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


/**
 * ====================================
 *        	  UROS Setup
 * ====================================
 */

// #define USED_UROS 
#define PROJECT_NAME "REFINE"
#define MODULE_NAME "front"

#define MICRO_ROS_AGENT_IP "10.10.0.167"
#define MICRO_ROS_AGENT_PORT "8888"
#define EXECUTOR_HANDLE_NUMBER 10

#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)


/**
 * ====================================
 *        Watchdog Timer Setup
 * ====================================
 */

// #define USED_CONNECTION_CHECK 1
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
	ESP_LOGI("ENCODER", "angle: %f", encoder_angle);
}


/**
 * ====================================
 *          Dynamixel Setup
 * ====================================
 */
#include <dynamixel_sdk.h>  

// Control table address -- YOU MUST LOOK UP YOUR MODEL
//#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
//#define ADDR_PRO_GOAL_POSITION          596
//#define ADDR_PRO_PRESENT_POSITION       611

// XM540-W270
// https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
#define ADDR_PRO_TORQUE_ENABLE          (64)                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          (116)
#define ADDR_PRO_PRESENT_POSITION       (132)
#define ADDR_PRO_PRESENT_CURRENT        (126)

#define LEN_PRO_GOAL_POSITION           (4)
#define LEN_PRO_PRESENT_POSITION        (4)
#define LEN_PRO_PRESENT_CURRENT         (2)

#define ADDR_PRO_PING                   (1)
#define ADDR_PRO_REBOOT                 (8)
#define ADDR_PRO_RETURN_DELAY_TIME      (9)

// These also need to be looked up
#define DXL_MINIMUM_POSITION_VALUE      (0)             	// Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      (4096)          	// and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting                  
#define BAUDRATE                        4000000
#define DEVICENAME                      "UART1"      		// Check which port is being used on your controller// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     50                  // Dynamixel moving status threshold
#define ESC_ASCII_VALUE                 0x1b
#define COMMAND_RETRY                   (400)     			// number of times to retry

// for some reason this isn't in the header
void action2(int port_num, uint8_t id);


// wrapper for a sleep function

void sleep_usecs(int usec) { esp_rom_delay_us(usec); }

#define DXL_ID_1                        (1)
#define DXL_ID_2                        (2) 



char *fill_time_buf(char *buf, int buf_len) 
{
  uint64_t now = esp_timer_get_time();
  uint32_t secs = now / 1000000;
  uint32_t usecs = now % 1000000;
  snprintf(buf, buf_len, "%lu.%06lu",secs,usecs);
  return(buf);
}


int set_goal_position(int port_num, uint32_t goal_pos) 
{
  int dxl_comm_result;  
  int dxl_error;
  uint8_t dxl_addparam_result = false;

  uint64_t t1 = esp_timer_get_time();

  // initalizae GroupSyncWrite Structs
  int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  uint64_t t2 = esp_timer_get_time();


  // Add Dynamixel#1 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID_1, goal_pos, LEN_PRO_GOAL_POSITION);
  if (dxl_addparam_result != true) 
  {
    printf("[ID:%03d] groupSyncWrite addparam failed", DXL_ID_1);
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }

  // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID_2, goal_pos, LEN_PRO_GOAL_POSITION);
  if (dxl_addparam_result != true) 
  {
    printf("[ID:%03d] groupSyncWrite addparam failed", DXL_ID_2);
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }

  uint64_t t3 = esp_timer_get_time();

  groupSyncWriteTxPacket(groupwrite_num);		// Send the goal position to the motors


  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) 
  {
    printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    groupSyncWriteClearParam(groupwrite_num);
    return(-1);
  }

  uint64_t t4 = esp_timer_get_time();

  groupSyncWriteClearParam(groupwrite_num);

  uint64_t t5 = esp_timer_get_time();

  printf("setGoal: goal_pos %lu t1~t2: %llu t2~t3 %llu t3~t4 %llu t4~t5 %llu t1~t5 %llu\n",goal_pos,t2-t1,t3-t2,t4-t3,t5-t4,t5-t1);
}


int get_goal_position(int port_num, uint32_t *goal_pos_1, uint32_t *goal_pos_2) 
{

  int dxl_comm_result;  
  int dxl_error;
  uint8_t dxl_addparam_result = False;

  uint64_t t1 = esp_timer_get_time();

  // initalizae GroupSyncWrite Structs
  int groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  uint64_t t2 = esp_timer_get_time();

  dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID_1);
  if (dxl_addparam_result != true) {
    printf("[ID:%03d] groupSyncRead addparam failed",DXL_ID_1);
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

  dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID_2);
  if (dxl_addparam_result != true) {
    printf("[ID:%03d] groupSyncRead addparam failed",DXL_ID_2);
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

  uint64_t t3 = esp_timer_get_time();

  // do the deed
  groupSyncReadTxRxPacket(groupread_num);

  uint64_t t4 = esp_timer_get_time();

  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
    printf("SyncRead: comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("SyncRead: Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

    // Check if groupsyncread data of Dynamixel#1 is available
  uint8_t dataIsAvailable = groupSyncReadIsAvailable(groupread_num, DXL_ID_1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dataIsAvailable == False)
  {
    printf("[ID:%03d] groupSyncRead getdata not available\n", DXL_ID_1);
    groupSyncReadClearParam(groupread_num);
    printf("getGoalPosition: failing read is not available\n");
    return(-1);
  }

  // Check if groupsyncread data of Dynamixel#2 is available
  if (False == groupSyncReadIsAvailable(groupread_num, DXL_ID_2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION))
  {
    printf("[ID:%03d] groupSyncRead getdata failed\n", DXL_ID_2);
    groupSyncReadClearParam(groupread_num);
    return(-1);
  }

  uint64_t t5 = esp_timer_get_time();

  // Get Dynamixel#1 present position value
  *goal_pos_1 = groupSyncReadGetData(groupread_num, DXL_ID_1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  // Get Dynamixel#2 present position value
  *goal_pos_2 = groupSyncReadGetData(groupread_num, DXL_ID_2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  uint64_t t6 = esp_timer_get_time();

  groupSyncReadClearParam(groupread_num);

  uint64_t t7 = esp_timer_get_time();

  printf("  getGoal: pos1 %lu pos2 %lu t1~t2: %llu t2~t3 %llu t3~t4 %llu t4~t5 %llu t5~t6 %llu t6~t7 %llu t1~t7 %llu\n",
    *goal_pos_1, *goal_pos_2, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5, t7-t6, t7-t1);


  return(0);

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
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, 
        enable ? TORQUE_ENABLE : TORQUE_DISABLE);

    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
	else
	{
		printf("set torque to %s\n",enable? "ENABLED" : "DISABLED");
	}

    sleep_usecs(5000);

    // check that torque is in the correct state
    torque_result = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("comm failed: %s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      continue;
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("Error: %s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      continue;
    }
   else
	{
		printf("read back torque, is: %d needs %d\n",torque_result,(int)enable);
	}

    if (enable && torque_result == TORQUE_ENABLE) {
      printf("Enabled torque, took %d attempts\n",attempts);
      return(0);
    }
    if (!enable && torque_result == TORQUE_DISABLE) {
      printf("Disabled torque, took %d attempts\n",attempts);
      return(0);
    }
 
  } while(attempts < COMMAND_RETRY);

  printf("Attempted to set torque more times than retry standard, failing\n");
  return(-1);
}

int servo_init(int port_num, int id) 
{
  uint8_t dxl_error = 0;    
  int dxl_comm_result = COMM_TX_FAIL;    

  printf("servo init: reboot: ID %d\n",id);

  sleep_usecs(10000);

  bool reboot_success = false;

  for (int i=0; i<3; i++) {

    reboot(port_num, PROTOCOL_VERSION,id);

    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) 
	{
        printf("reboot comm id %d failed: %s\n",id, getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        //return(-1);
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("reboot Error: id %d %s\n", id,getRxPacketError(PROTOCOL_VERSION, dxl_error));
        //return(-1);
    }
    else 
	{
      reboot_success = true;
      sleep_usecs(150000);
      break;
    }

    sleep_usecs(150000);

  }

  if (reboot_success == false) {
    return(-1);
  }

  printf("Enable torque\n");
  if (0 != set_torque_mode(port_num,id, true)) {
    printf("Enable Torque Failure: Error: id %d %s\n", id,getRxPacketError(PROTOCOL_VERSION, dxl_error));
    // coulnd't enable... disabel??
    return(-1);
  }

  return(0);
}


#endif // CONFIG_H