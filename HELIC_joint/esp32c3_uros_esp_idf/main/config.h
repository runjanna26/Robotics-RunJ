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

#include "watchdog_timer.h"

#define PROJECT_NAME "REFINE"
#define MODULE_NAME "front"

#define MICRO_ROS_AGENT_IP "10.10.0.167"
#define MICRO_ROS_AGENT_PORT "8888"
#define EXECUTOR_HANDLE_NUMBER 10

#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while (0)


// #define USED_CONNECTION_CHECK 1
WatchdogTimer watchdogtimer;
WatchdogTimer watchdogtimer_restartESP;


#endif // CONFIG_H