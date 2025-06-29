// swerve
#include "HERO_WheelModule.h"
#define robot_type 3       // omni =  1 //diff = 2 //swerve = 3
#define Module_name "Back" // front = 1 //back = 2

bool using_connectioncheck = true;

#include "ICM_20948.h"
#define WIRE_PORT Wire
#define AD0_VAL 0

// for PWM
const int freq = 10000;
const int ledChannel = 0;
const int resolution = 8;
float speed_delay;
uint8_t i, recv_cnt, recv_cnt_torque_en;
rcl_subscription_t MotorCMDsubscriber;
rcl_subscription_t MotorENsubscriber;
rcl_subscription_t BoardCMDsubscriber;
rcl_subscription_t CMDVELsubscriber;
rcl_subscription_t LEDsubscriber;
rcl_subscription_t Connectionchecksubscriber;

rcl_publisher_t publisher;
rcl_publisher_t DXLpublisher;
rcl_publisher_t IMUpublisher;
rcl_publisher_t RPYpublisher;
rcl_publisher_t Connectioncheckpublisher;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

bool micro_ros_init_successful;

std_msgs__msg__Int16MultiArray DXLmsg;
std_msgs__msg__Int8MultiArray MotorENmsg;
std_msgs__msg__Int8MultiArray BoardCMDmsg;

sensor_msgs__msg__Imu IMUmsg;
geometry_msgs__msg__Twist RPYmsg;
sensor_msgs__msg__Joy Joymsg;
std_msgs__msg__Float32MultiArray MotorCMDmsg;
std_msgs__msg__Int8 LEDmsg;
std_msgs__msg__Bool ConnectionSubmsg;
std_msgs__msg__Bool ConnectionPubmsg;

int imu_ini_count = 0;

float goal_robot_speed[4] = {0.0, 0.0, 0.0, 0.0};
int goal_robot_angle[4] = {0, 0, 0, 0};
float anti_goal_robot_angle[4] = {0.0, 0.0, 0.0, 0.0};

float wheelrad[4] = {0.0, 0.0, 0.0, 0.0};
int wheelsign[4] = {1, 1, 1, 1};
float anti_wheelrad[4] = {0.0, 0.0, 0.0, 0.0};
float wheeldeg[4] = {0.0, 0.0, 0.0, 0.0};
float anti_wheeldeg[4] = {0.0, 0.0, 0.0, 0.0};
float anti_deg360[4] = {0.0, 0.0, 0.0, 0.0};
float VelocityWheel1 = 0.0;
float VelocityWheel2 = 0.0;
float VelocityWheel3 = 0.0;
float VelocityWheel4 = 0.0;

float PositionWheel1 = 0.0;
float PositionWheel2 = 0.0;
float PositionWheel3 = 0.0;
float PositionWheel4 = 0.0;

//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------     Create Watchdog Timer for reconnection          -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------

int32_t watchdog_time = 0;
int32_t last_time = 0;

WatchdogTimer watchdogtime(200); // depend on hz of joystick//200
WatchdogTimer watchdogtime_restartESP(1000);
//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------     Create millis Timer for reding dxl value        -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------
unsigned long previousMillis = 0;
const long interval = 50;

Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
ICM_20948_I2C myICM;

//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------     Create Entity for node,publisher,subscriber     -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node

  char *name = create_name("HEROWheelModuleNode_", String(Module_name).c_str());
  RCCHECK(rclc_node_init_default(&node, name, "", &support));
  delete[] name; // don't forget to free the memory allocated by create_name

  // create publisher

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/SwerveFeedback");
  RCCHECK(rclc_publisher_init_best_effort(
      &DXLpublisher,
      &node,
      // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/IMU");

  RCCHECK(rclc_publisher_init_best_effort(
      &IMUpublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/RPY");

  RCCHECK(rclc_publisher_init_best_effort(
      &RPYpublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/ConnectionWheelStation");

  RCCHECK(rclc_publisher_init_best_effort(
      &Connectioncheckpublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/Command");
  RCCHECK(rclc_subscription_init_best_effort(
      &MotorCMDsubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/MotorEnable");
  RCCHECK(rclc_subscription_init_default(
      &MotorENsubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/BoardCommand");
  RCCHECK(rclc_subscription_init_default(
      &BoardCMDsubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
      name));
  delete[] name;

  name = create_name("HERO/WheelModule/", String(Module_name).c_str());
  name = create_name(name, "/LED");
  RCCHECK(rclc_subscription_init_default(
      &LEDsubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      name));
  delete[] name;

  if (using_connectioncheck)
  {
    name = create_name("HERO/WheelModule/", String(Module_name).c_str());
    name = create_name(name, "/ConnectionStationWheel");
    RCCHECK(rclc_subscription_init_best_effort(
        &Connectionchecksubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        name));
    delete[] name;
  }

  // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executorr
  // RCCHECK(rclc_executor_init(&executor, &support.context,1, &allocator));
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // RCCHECK(rclc_executor_add_subscription(&executor, &CMDVELsubscriber, &CMDmsg, &subscription_callback_cmdvel, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &MotorCMDsubscriber, &MotorCMDmsg, &subscription_callback_motorcmd, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &MotorENsubscriber, &MotorENmsg, &subscription_callback_motorenablecmd, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &BoardCMDsubscriber, &BoardCMDmsg, &subscription_callback_boardcmd, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &LEDsubscriber, &LEDmsg, &subscription_callback_ledcmd, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &Connectionchecksubscriber, &ConnectionSubmsg, &subscription_callback_connectioncheck, ON_NEW_DATA));

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

  rcl_publisher_fini(&DXLpublisher, &node);
  rcl_publisher_fini(&IMUpublisher, &node);
  rcl_publisher_fini(&RPYpublisher, &node);

  rcl_subscription_fini(&MotorCMDsubscriber, &node);
  rcl_subscription_fini(&MotorENsubscriber, &node);
  rcl_subscription_fini(&BoardCMDsubscriber, &node);
  rcl_subscription_fini(&LEDsubscriber, &node);
  rcl_subscription_fini(&Connectionchecksubscriber, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------         CMDvel Callback for command as vector       -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------

void subscription_callback_connectioncheck(const void *msgin)
{
  const std_msgs__msg__Bool *ConnectionSubmsg = (const std_msgs__msg__Bool *)msgin;

  if (using_connectioncheck == true)
  {
    watchdogtime.restart_counter();
    watchdogtime_restartESP.restart_counter();
  }
}

void subscription_callback_motorenablecmd(const void *msgin)
{
  const std_msgs__msg__Int8MultiArray *MotorENmsg = (const std_msgs__msg__Int8MultiArray *)msgin;

  for (i = 0; i < 8; i++)
  {
    if (MotorENmsg->data.data[i] == 0)
    {
      dxl.torqueOff(DXL_ALL_ID_LIST[i]);
      Serial.print(F("disable torque motor : "));
      Serial.println(i);
    }
    else if (MotorENmsg->data.data[i] == 1)
    {
      dxl.torqueOn(DXL_ALL_ID_LIST[i]);
      Serial.print(F("Enable torque motor : "));
      Serial.println(i);
    }
  }
}

void subscription_callback_boardcmd(const void *msgin)
{
  const std_msgs__msg__Int8MultiArray *BoardCMDmsg = (const std_msgs__msg__Int8MultiArray *)msgin;

  // motor command
  if (BoardCMDmsg->data.data[0] == 0)
  {
    // off motor power
    digitalWrite(pin_MOTOR_EN, LOW);
    delay(500);
  }
  else if (BoardCMDmsg->data.data[0] == 1)
  {
    // restart esp
    ESP.restart();
  }
}

void subscription_callback_ledcmd(const void *msgin)
{
  const std_msgs__msg__Int8 *LEDmsg = (const std_msgs__msg__Int8 *)msgin;

  Serial.println(F("LEDtopic"));

  if (LEDmsg->data == 0)
  {
    analogWrite(pin_WhiteLight_EN, 0);
  }
  else if (LEDmsg->data == 1)
  {
    analogWrite(pin_WhiteLight_EN, 150);
  }

  return;
}

void subscription_callback_motorcmd(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *MotorCMDmsg = (const std_msgs__msg__Float32MultiArray *)msgin;

  Struct_wheelvector command;

  command.goalrad[0] = constrain((MotorCMDmsg->data.data[1]), -M_PI, M_PI); // Dir FL   //swap because difference btw motor config of real robot and sim
  command.goalrad[1] = constrain((MotorCMDmsg->data.data[3]), -M_PI, M_PI); // Dir FR
  command.goalrad[2] = constrain((MotorCMDmsg->data.data[5]), -M_PI, M_PI); // Dir BL
  command.goalrad[3] = constrain((MotorCMDmsg->data.data[7]), -M_PI, M_PI); // Dir BR

  command.goalmagnitude[0] = MotorCMDmsg->data.data[0]; // Drv FL   -
  command.goalmagnitude[1] = MotorCMDmsg->data.data[2]; // Drv FR   +
  command.goalmagnitude[2] = MotorCMDmsg->data.data[4]; // Drv BL  +
  command.goalmagnitude[3] = MotorCMDmsg->data.data[6]; // Drv BR  -

  Struct_wheelvector closest = closestAngle(command);

  for (int i = 0; i < 4; i++)
  {
    wheelrad[i] = closest.closestAngle[i];
    wheelsign[i] = closest.magnitudeSign[i];
    goal_robot_angle[i] = int((wheelrad[i] * 2048) / M_PI) + 2048;
  }

  for (int i = 0; i < 4; i++)
  {
    sw_position_data[i].goal_position = constrain(goal_robot_angle[i], 0, 4095);
  }

  sw_position_infos.is_info_changed = true;
  if (dxl.syncWrite(&sw_position_infos) == true)
  {
    // Serial.println("Write position success");curr
  }

  for (int i = 0; i < 4; i++)
  {
    goal_robot_speed[i] = command.goalmagnitude[i] * wheelsign[i];
  }

  speed_delay = ((sw_position_data[0].goal_position - DXLmsg.data.data[0]) / 2048.00);
  if (speed_delay < 0)
    speed_delay = -speed_delay;
  if (speed_delay > 0.5) // 0.3
    speed_delay = 0;
  else
    speed_delay = constrain((1 - (1 * speed_delay)), 0, 1);

  sw_velocity_data[0].goal_velocity = goal_robot_speed[0] * 200 * speed_delay; // FL
  sw_velocity_data[1].goal_velocity = goal_robot_speed[1] * 200 * speed_delay; // FR
  sw_velocity_data[2].goal_velocity = goal_robot_speed[2] * 200 * speed_delay; // BL
  sw_velocity_data[3].goal_velocity = goal_robot_speed[3] * 200 * speed_delay; // BR

  sw_velocity_infos.is_info_changed = true;

  if (dxl.syncWrite(&sw_velocity_infos) == true)
  {
    // DEBUG_SERIAL.println(F("[SyncWrite   VELO ] Success"));
  }

  WS2812B.setPixelColor(0, WS2812B.Color(0, 100, 100));
  WS2812B.show();
  return;
}

//----------------------------------------------------------------------------------------
//------------------                                                     -----------------
//------------------                Command Robot Subroutine             -----------------
//------------------                                                     -----------------
//----------------------------------------------------------------------------------------

Struct_wheelvector closestAngle(Struct_wheelvector wheelcommand)
{

  Struct_wheelvector closestpoint;
  float anti_goalRad = 0.0;
  for (int i = 0; i < 4; i++)
  {
    if (wheelcommand.goalrad[i] > 0)
      anti_goalRad = wheelcommand.goalrad[i] - M_PI;
    else if (wheelcommand.goalrad[i] < 0)
      anti_goalRad = wheelcommand.goalrad[i] + M_PI;

    float presentPositionRad = constrain((((DXLmsg.data.data[i] - 2048) * M_PI) / 2048), -M_PI, M_PI);

    if ((presentPositionRad - wheelcommand.goalrad[i]) > (presentPositionRad - anti_goalRad))
    {
      closestpoint.magnitudeSign[i] = -1;
      closestpoint.closestAngle[i] = anti_goalRad;
    }
    else if ((presentPositionRad - wheelcommand.goalrad[i]) <= (presentPositionRad - anti_goalRad))
    {
      closestpoint.magnitudeSign[i] = 1;
      closestpoint.closestAngle[i] = wheelcommand.goalrad[i];
    }
  }
  return closestpoint;
}

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

    RCSOFTCHECK(rcl_publish(&DXLpublisher, &DXLmsg, NULL));
    RCSOFTCHECK(rcl_publish(&IMUpublisher, &IMUmsg, NULL));
    RCSOFTCHECK(rcl_publish(&RPYpublisher, &RPYmsg, NULL));

    RCSOFTCHECK(rcl_publish(&Connectioncheckpublisher, &ConnectionPubmsg, NULL));
    WS2812B.setPixelColor(0, WS2812B.Color(100, 100, 0));
    WS2812B.show();
  }
  return;
}

//****************************************************************************************
//****************************************************************************************
//------------------                                                     -----------------
//------------------                     Void Setup                      -----------------
//------------------                                                     -----------------
//****************************************************************************************
//****************************************************************************************

void setup()
{
  WS2812B.begin();
  WS2812B.setPixelColor(0, WS2812B.Color(0, 0, 100));
  WS2812B.show();

  DEBUG_SERIAL.begin(115200);

  set_microros_wifi_transports("hero-wifi-router", "heroplusplus", "10.99.0.99", 8888);
  // set_microros_wifi_transports("hero-wifi-router", "heroplusplus", "10.99.0.99", 8888);
  // set_microros_wifi_transports("VISTEC-Gait", "exvis123", "192.168.1.157", 8888);
  // set_microros_wifi_transports("WAI-wifi", "asdfasdf", "192.168.0.106", 8888);
  // set_microros_transports();

  state = WAITING_AGENT;

  pinMode(pin_MAG1, OUTPUT);
  digitalWrite(pin_MAG1, HIGH);

  pinMode(LED_PIN, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(pin_WhiteLight_EN, OUTPUT);
  analogWrite(pin_WhiteLight_EN, 150);

  delay(500);
  pinMode(pin_MOTOR_EN, OUTPUT);
  digitalWrite(pin_MOTOR_EN, LOW);
  delay(1500);
  digitalWrite(pin_MOTOR_EN, HIGH);
  delay(500);

  // WS2812B.setPixelColor(0, WS2812B.Color(100, 100, 100));
  // WS2812B.show();
  //----------------------------------------------------------------------------------------
  //------------------                                                     -----------------
  //------------------         Handling Memory for subscribe massage       -----------------
  //------------------                                                     -----------------
  //----------------------------------------------------------------------------------------

  MotorCMDmsg.data.capacity = 20;
  MotorCMDmsg.data.size = 0;
  MotorCMDmsg.data.data = (float_t *)malloc(MotorCMDmsg.data.capacity * sizeof(float_t));

  MotorENmsg.data.capacity = 8;
  MotorENmsg.data.size = 0;
  MotorENmsg.data.data = (int8_t *)malloc(MotorENmsg.data.capacity * sizeof(int8_t));

  BoardCMDmsg.data.capacity = 8;
  BoardCMDmsg.data.size = 0;
  BoardCMDmsg.data.data = (int8_t *)malloc(MotorENmsg.data.capacity * sizeof(int8_t));

  //----------------------------------------------------------------------------------------
  //------------------                                                     -----------------
  //------------------         Handling Memory for public massage          -----------------
  //------------------                                                     -----------------
  //----------------------------------------------------------------------------------------

  DXLmsg.data.capacity = 40;
  DXLmsg.data.size = 0;
  DXLmsg.data.data = (int16_t *)malloc(DXLmsg.data.capacity * sizeof(int16_t));

  //----------------------------------------------------------------------------------------
  //------------------                                                     -----------------
  //------------------         Dynamixel First Setup                       -----------------
  //------------------                                                     -----------------
  //----------------------------------------------------------------------------------------
  dxl.begin(1000000);                               // 57600
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Get DYNAMIXEL information

  for (i = 0; i < 4; i++) // set mode position motor
  {
    dxl.torqueOff(DXL_POSITION_ID_LIST[i]);
    dxl.setOperatingMode(DXL_POSITION_ID_LIST[i], OP_POSITION);
    dxl.writeControlTableItem(DRIVE_MODE, DXL_POSITION_ID_LIST[i], 4);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_POSITION_ID_LIST[i], 300);
    dxl.torqueOn(DXL_POSITION_ID_LIST[i]);
  }
  for (i = 0; i < 4; i++) // set mode velocity motor
  {
    dxl.torqueOff(DXL_VELOCITY_ID_LIST[i]);
    dxl.setOperatingMode(DXL_VELOCITY_ID_LIST[i], OP_VELOCITY);
    dxl.torqueOn(DXL_VELOCITY_ID_LIST[i]);
  }
  Serial.println(F("Enable torque"));

  fillSyncReadStructures();
  fillSyncWriteStructures();

  //----------------------------------------------------------------------------------------
  //------------------                                                     -----------------
  //------------------                   IMU Initial setup                 -----------------
  //------------------                                                     -----------------
  //----------------------------------------------------------------------------------------

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  initializeIMU();
  //----------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------

  create_entities();
}

//****************************************************************************************
//****************************************************************************************
//------------------                                                     -----------------
//------------------                     Void Loop                     -----------------
//------------------                                                     -----------------
//****************************************************************************************
//****************************************************************************************

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Reset the timer

    recv_cnt = dxl.syncRead(&sr_infos);
    if (recv_cnt > 0)
    {

      for (i = 0; i < 4; i++)
        if (sr_data[i].present_position >= 0 && sr_data[i].present_position <= 4096)
        {
          DXLmsg.data.data[i] = (int16_t)sr_data[i].present_position;
        }
      for (i = 4; i < 8; i++)
      {
        DXLmsg.data.data[i] = (int16_t)sr_data[i].present_velocity;
      }
      for (i = 8; i < 16; i++)
      {
        DXLmsg.data.data[i] = (int16_t)sr_data[i - 8].present_current;
      }
    }

    recv_cnt_torque_en = dxl.syncRead(&sr_torque_en_infos);
    if (recv_cnt_torque_en > 0)
    {

      for (i = 16; i < 24; i++)
      {
        DXLmsg.data.data[i] = (int16_t)sr_data_torque_en[i - 16].torque_en;
      }
    }
    DXLmsg.data.size = 24;
  }

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {

      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      if (!isnan(q0) && !isnan(q1) && !isnan(q2) && !isnan(q3))
      {
        IMUmsg.orientation.w = q0;
        IMUmsg.orientation.x = q1;
        IMUmsg.orientation.y = q2;
        IMUmsg.orientation.z = q3;

        RPYmsg.angular.x = roll;
        RPYmsg.angular.y = pitch;
        RPYmsg.angular.z = yaw;
      }
    }
    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      float acc_y = (float)data.Raw_Accel.Data.Y;
      float acc_z = (float)data.Raw_Accel.Data.Z;

      IMUmsg.linear_acceleration.x = acc_x;
      IMUmsg.linear_acceleration.y = acc_y;
      IMUmsg.linear_acceleration.z = acc_z;
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      float gy_x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      float gy_y = (float)data.Raw_Gyro.Data.Y;
      float gy_z = (float)data.Raw_Gyro.Data.Z;

      IMUmsg.angular_velocity.x = gy_x;
      IMUmsg.angular_velocity.y = gy_y;
      IMUmsg.angular_velocity.z = gy_z;
    }

    if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
    {
      float mag_x = (float)data.Compass.Data.X; // Extract the compass data
      float mag_y = (float)data.Compass.Data.Y;
      float mag_z = (float)data.Compass.Data.Z;

      IMUmsg.orientation_covariance[0] = mag_x;
      IMUmsg.orientation_covariance[1] = mag_y;
      IMUmsg.orientation_covariance[2] = mag_z;
    }
  }

  if (using_connectioncheck)
  {

    EXECUTE_EVERY_N_MS(1, connectionCheck());
    ConnectionPubmsg.data = true;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}

void connectionCheck()
{
  if (watchdogtime.checktimeout())
  {
    DEBUG_SERIAL.println("connection timeout : robot will stop ");
    sw_position_data[0].goal_position = 2048;
    sw_position_data[1].goal_position = 2048;
    sw_position_data[2].goal_position = 2048;
    sw_position_data[3].goal_position = 2048;

    sw_position_infos.is_info_changed = true;
    if (dxl.syncWrite(&sw_position_infos) == true)
    {
      // Serial.println("Write position success");
    }

    sw_velocity_data[0].goal_velocity = 0; // FL
    sw_velocity_data[1].goal_velocity = 0; // FR
    sw_velocity_data[2].goal_velocity = 0; // BL
    sw_velocity_data[3].goal_velocity = 0; // BR

    sw_velocity_infos.is_info_changed = true;

    if (dxl.syncWrite(&sw_velocity_infos) == true)
    {
      // DEBUG_SERIAL.println(F("[SyncWrite   VELO ] Success"));
    }

    // destroy_entities();
    digitalWrite(LED_PIN, 0);
    WS2812B.setPixelColor(0, WS2812B.Color(100, 0, 0));
    WS2812B.show();
    // DEBUG_SERIAL.println("reconnecting...");
    // create_entities();
  }

  if (watchdogtime_restartESP.checktimeout())
  {
    DEBUG_SERIAL.println("connection timeout : ESP will be restart");
    destroy_entities();
    delay(1000);
    ESP.restart();
  }
  else
  {
    digitalWrite(LED_PIN, 1);
  }
}

void initializeIMU()
{

  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    DEBUG_SERIAL.print(F("Initialization of the sensor returned: "));
    DEBUG_SERIAL.println(myICM.statusString());
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
      imu_ini_count = imu_ini_count + 1;
      if (imu_ini_count > 4)
      {
        imu_ini_count = 0;
        break;
      }
    }
    else
    {
      initialized = true;
    }
  }
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  DEBUG_SERIAL.println(success);

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  DEBUG_SERIAL.println(success);
  // Enable any additional sensors / features
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
  DEBUG_SERIAL.println(success);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);        // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);        // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);         // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok);        // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  DEBUG_SERIAL.println(success);
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  DEBUG_SERIAL.println(success);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  DEBUG_SERIAL.println(success);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  DEBUG_SERIAL.println(success);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  DEBUG_SERIAL.println(success);
}

//****************************************************************************************
//****************************************************************************************
//---------------------                                         --------------------------
//---------------------                                         --------------------------
//---------------------                END                      --------------------------
//---------------------                                         --------------------------
//---------------------                                         --------------------------
//****************************************************************************************
//****************************************************************************************
