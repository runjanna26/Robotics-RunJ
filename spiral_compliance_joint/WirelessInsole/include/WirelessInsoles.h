
/**
 * $    pip3 install paho-mqtt python-etcd
 * 
 */

#include <Arduino.h>
#include <WiFi.h>
#include <cstdint>
#include <time.h>
#include <string>
#include <stdlib.h>
#include <cstring>
// External lib.
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"

// LED lib.
#include <Adafruit_NeoPixel.h>
#define PIN_WS2812B 16
#define NUM_PIXELS 1
Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

// Micro-ROS lib.
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>  // Access connection status

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joy.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

rcl_publisher_t IMUpublisher;
sensor_msgs__msg__Imu IMUmsg;

rcl_publisher_t FootForcepublisher;
std_msgs__msg__Float32MultiArray FootForcemsg;

geometry_msgs__msg__Twist RPYmsg;

// IMU lib.
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define OUTPUT_READABLE_YAWPITCHROLL
#include "ICM_20948.h"
#define AD0_VAL 0
ICM_20948_I2C myICM;


//MUX Pin setup
const int ADC_PIN = 32;
const int S0 = 18;
const int S1 = 5;
const int S2 = 19;
const int S3 = 2;

int FootForce[16];


// ================================================================
//                   General Function & Macro                
// ================================================================
unsigned long previousMillis = 0;
bool LED_mode_trigger = false;
int batteryValue;
bool CheckConnectionFlag = false;
int ConnectionStatus;


char *create_name(const std::string &str1, const std::string &str2)
{
  // Combine the two strings into a single string
  std::string combined = str1 + str2;

  // Allocate memory for the combined string
  char *name = new char[combined.length() + 1];

  // Copy the combined string into the allocated memory
  std::strcpy(name, combined.c_str());

  return name;
}


#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
