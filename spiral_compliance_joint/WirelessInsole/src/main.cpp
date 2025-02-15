#include <WirelessInsoles.h>
#define Module_name "Left"  // Right, Left

// NANO-PC
// #define IP "10.10.0.2"
// #define SSID "WearableRobot-wifi-2.4G"

// NUC
#define IP "192.168.1.3"
#define SSID "WearableRobotics_2.4G"
#define PWD "exvis123"




//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------       Timer Callback for publish data to ros2       -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL)
  {


  }
  return;
}

//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------     Create Entity for node,publisher,subscriber     -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------


bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  char *name = create_name("WirelessInsoleNode_", String(Module_name).c_str());
  RCCHECK(rclc_node_init_default(&node, name, "", &support));
  delete[] name; // don't forget to free the memory allocated by create_name

  // create publisher
  name = create_name("WearableRobot/WirelessInsoles/", String(Module_name).c_str());
  name = create_name(name, "/IMU");
  RCCHECK(rclc_publisher_init_best_effort(
      &IMUpublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      name));
  delete[] name;

  name = create_name("WearableRobot/WirelessInsoles/", String(Module_name).c_str());
  name = create_name(name, "/FootForces");
  RCCHECK(rclc_publisher_init_best_effort(
      &FootForcepublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      name));
  delete[] name;


  // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // RCCHECK(rclc_executor_add_subscription(&executor, &CMDVELsubscriber, &CMDmsg, &subscription_callback_cmdvel, ON_NEW_DATA));
  return true;
}


//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------     Destroy Entity when disconnect                  -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&IMUpublisher, &node);
  rcl_publisher_fini(&FootForcepublisher, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


void create_messages()
{
  // Create message
  FootForcemsg.data.capacity = 16;
  FootForcemsg.data.size = 0;
  FootForcemsg.data.data = (float *)malloc(FootForcemsg.data.capacity * sizeof(float));
}

void setup() 
{
  Serial.begin(115200);     //Initializes serial communication
  WS2812B.begin();

  set_microros_wifi_transports(SSID, PWD, IP, 8888);
  if (WiFi.status() != WL_CONNECTED)
  {
    WS2812B.setPixelColor(0, WS2812B.Color(0, 0, 100));
    WS2812B.show();
    delay(5000);
  }

  pinMode(ADC_PIN,INPUT);   //Set this pin as input
  pinMode(S0,OUTPUT);       //Set this pin as output
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);


  //----------------------------------------------------------------------------------------
  //------------------                                                     -----------------
  //------------------                   IMU Initial setup                 -----------------
  //------------------                                                     -----------------
  //----------------------------------------------------------------------------------------
  Wire.begin();
  Wire.setClock(400000);

  create_messages();
  // Create Micro-ROS entities
  create_entities();


}

void loop() 
{
  

}