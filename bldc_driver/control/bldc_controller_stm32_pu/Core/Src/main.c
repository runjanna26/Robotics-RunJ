/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   // printf function
#include <stdlib.h>
#include <stdbool.h>
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
#define rad2deg 57.29577951
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _sqrt(a) (_sqrtApprox(a))
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _isset(a) ( (a) != (NOT_SET) )
#define NOT_SET -12345.0

#define DEF_ANGLE_REGISTER 0x3FFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CORDIC_HandleTypeDef hcordic;

CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;

FMAC_HandleTypeDef hfmac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
//===Magnetic Sensor===//
struct MagneticSensorSPIConfig_s {
	int bit_resolution;
	int angle_registers;
	int data_start_bit;
	int command_rw_bit;
	int command_parity_bit;
};
struct MagneticSensorSPIConfig_s AS5048A_SPI = { .bit_resolution = 14,
		.angle_registers = 0x3FFF, .data_start_bit = 13, .command_rw_bit = 14,
		.command_parity_bit = 15 };
float cpr; // Maximum range of the magnetic sensor
int bit_resolution; 	//!< the number of bites of angle data
int command_parity_bit; //!< the bit where parity flag is stored in command
int command_rw_bit; 	//!< the bit where read/write flag is stored in command
int data_start_bit; 	//!< the the position of first bit
int angle_register; 	//!< SPI angle register to read

float angle_prev = 0; // result of last call to getSensorAngle(), used for full rotations and velocity
float vel_angle_prev = 0; // angle at last call to getVelocity, used for velocity
float acc = 0;
float acc_prev = 0 ;
float acc_vel_prev = 0;
float acc_vel = 0;
float zeta_1 = 0 ;
float zeta_2 = 0 ;
float zeta_3 = 0 ;
float getVelocity;
int32_t full_rotations = 0; 	// full rotation tracking
int32_t vel_full_rotations = 0; // previous full rotation value for velocity calculation
long vel_angle_prev_ts = 0; 	// last velocity calculation timestamp
long angle_prev_ts = 0; // timestamp of last call to getAngle, used for velocity
long vel_angle_ts = 0;
long acc_ts = 0;
long acc_prev_ts = 0;
//===Current Sensor===
struct DQCurrent_s // dq current structure
{
	float d;
	float q;
} debug_current_dq;
struct PhaseCurrent_s // phase current structure
{
	float a;
	float b;
	float c;
	float rms;
} debug_current_abc, debug_current_rms;
struct DQVoltage_s // d-q voltage struct
{
	float d;
	float q;
};
double offset_ia; //!< zero current A voltage value (center of the adc reading)
double offset_ib; //!< zero current B voltage value (center of the adc reading)
double offset_ic; //!< zero current C voltage value (center of the adc reading)
float offset_iq;
float offset_id;
float gain_a;     //!< phase A gain
float gain_b;     //!< phase B gain
float gain_c;     //!< phase C gain
float R_sense;
float CurrentSense_resistance;
float CurrentSense_gain;
//ADC DMA variable
uint32_t adcResultDMA_a[1], adcResultDMA_b[1];  // to store the ADC value
//const int adcChannelCount = sizeof(adcResultDMA_a)/sizeof(adcResultDMA_a[0]);
volatile int adcConversionComplete = 0; // set by callback

//SVPWM
float Ua, Ub, Uc;	// Current phase voltages Ua,Ub,Uc set to motor

//Control Variable
float target;             //!< current target value - depends of the controller
float shaft_angle;        //!< current motor angle
float electrical_angle;   //!< current electrical angle
float shaft_velocity;     //!< current motor velocity
float current_sp;         //!< target current ( q current )
float shaft_velocity_sp;  //!< current target velocity
float shaft_angle_sp;     //!< current target angle
float prev_shaft_angle_sp;
float switch_threshold ;
bool dir_setpoint ;
struct DQVoltage_s voltage;      //!< current d and q voltage set to the motor
struct DQCurrent_s current;      //!< current d and q current measure

//Low pass filter
struct LPF // LPF struct
{
	float x;							//!< (INPUT)
	unsigned long timestamp_prev;  	//!< Last execution timestamp
	float y_prev; 		//!< filtered value in previous execution step (OUTPUT)
	float Tf;
	float debug_dt;
} LPF_angle, LPF_velocity,LPF_acceleration, LPF_current_Q, LPF_current_D, LPF_Command_setpoint, LPF_w_est, LPF_r_est,
		LPF_a_est ,LPF_SMC_out , LPF_current_a, LPF_current_b, LPF_current_c;

//PID
struct PID {
	float error;					//!< INPUT
	float integral_prev; 			//!< last integral component value
	float error_prev; 				//!< last tracking error value
	unsigned long long timestamp_prev; 	//!< Last execution timestamp
	float output_prev;  			//!< last pid output value (OUTPUT)
	float output_ramp; 			//!< Maximum speed of change of the output value
	float limit; 					//!< Maximum output value
	float anti_windup;
	float dtx;
	float P; 						//!< Proportional gain
	float I; 						//!< Integral gain
	float D; 						//!< Derivative gain
} current_D,current_Q, velocity, angle, haptic;

struct PID_current_D {
	float error;					//!< INPUT
	float integral_prev; 			//!< last integral component value
	float error_prev; 				//!< last tracking error value
	unsigned long long timestamp_prev; 	//!< Last execution timestamp
	float output_prev;  			//!< last pid output value (OUTPUT)
	float output_ramp; 			//!< Maximum speed of change of the output value
	float limit; 					//!< Maximum output value
	float anti_windup;
	float dtx;
	float P; 						//!< Proportional gain
	float I; 						//!< Integral gain
	float D; 						//!< Derivative gain
} current_D_2;
float passivity_gain;

struct SMC {
	//SMC+ESDMO state variable
	float error;
	float error_prev;
	float error_est;
	float error_est_prev;
	float integral_error_prev;
	float integral_error_est_prev;
	float sign_temp_prev;
	float integral_sign_prev ;
	float a_est_prev ;
	float r_est_prev;
	float w_est_prev;
	float da_est_prev;
	float dr_est_prev;
	float dw_est_prev;
	float eq_s_prev;
	float output_prev;
	float current_q_prev;
	float integral_iq_prev;
	//gain
	float k;
	float f;
	float ep;
	float delta;
	float g;
	float neta;
	float kp;
	float ki;
	//other variable
	float debug_output;
	float debug_eq;
	float limit;
	float eq_s_ramp;
	float dtx;
	unsigned long timestamp_prev;

} SMC_vel,SMC_ang;
struct fusion {
	//SMC+ESDMO state variable
	float error;
	float error_prev;
	float error_est;
	float error_est_prev;
	float a_est_prev ;
	float r_est_prev;
	float w_est_prev;
	float da_est_prev;
	float dr_est_prev;
	float dw_est_prev;
	float eq_s_prev;
	float output_prev;
	float current_q_prev;
	float integral_prev;
	//gain
	float k;
	float ep;
	float delta;
	float g;
	float neta;
	float P; 						//!< Proportional gain
	float I; 						//!< Integral gain
	float D; 						//!< Derivative gain
	//other variable
	float debug_output;
	float debug_eq;
	float anti_windup;
	float limit;
	float eq_s_ramp;
	float dtx;
	unsigned long timestamp_prev;

} fusion_vel;
// ======motor physical parameters (SET HERE !!!)======
// motor configuration parameters
float voltage_sensor_align;		//!< sensor and motor align voltage parameter
float velocity_index_search;	//!< target velocity for index search
float voltage_power_supply;
float phase_resistance; 	//!< motor phase resistance
float phase_inductance;
float inertia;
float flux_linkage;
float damping;
int pole_pairs;			//!< motor pole pairs number

// limiting variables
float voltage_limit; //!< Voltage limitting variable - global limit
float current_limit; //!< Current limitting variable - global limit
float velocity_limit; //!< Velocity limitting variable - global limit

//Position sensor variable
float sensor_offset = 0; //!< user defined sensor zero offset
float zero_electric_angle = NOT_SET; //!< absolute zero electric angle - if available
float zero_electric_angle_avg = NOT_SET;
int sensor_direction = NOT_SET; //!< if sensor_direction == Direction::CCW then direction will be flipped to CW
enum Direction {
	CW = 1,  //clockwise
	CCW = -1, // counter clockwise
	UNKNOWN = 0   //not yet known or invalid state
};
struct align {
	float shaft_angle_pp[14];
	float zero_angle_pp[14];
} align;

long open_loop_timestamp;

union Float2Bytes_4 {
	float asFloat;
	uint8_t asUint8_t[4];
} f2b, b2f;

union int2int_2 {
	int16_t asInt16_t;
	uint8_t asUint8_t[2];
} i2b_current, i2b_velocity, i2b_angle, b2i_limit_vel;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_CRC_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE BEGIN PFP */
__STATIC_INLINE void DWT_Init(void); 						//checked
__STATIC_INLINE void delay_us(uint32_t us);				//dont use it
__STATIC_INLINE uint32_t micros(void);						//checked

void float2Bytes(float val, uint8_t *bytes_array);  		//checked
float Bytes2float(uint8_t *bytes_array);					//checked
void velocity_command();
void position_command();
void feedback_command();
//===Current Sensor function===//
void initCurrentsense(float _shunt_resistor, float _gain);  	//
void calibrateOffsets();									//
struct PhaseCurrent_s getPhaseCurrents();					//checked
struct DQCurrent_s getFOCCurrents(float angle_el);			//checked

//Math Calculation
const int sine_array[200] = { 0, 79, 158, 237, 316, 395, 473, 552, 631, 710,
		789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494, 1572, 1650,
		1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269, 2346, 2423, 2499, 2575,
		2652, 2728, 2804, 2879, 2955, 3030, 3105, 3180, 3255, 3329, 3404, 3478,
		3552, 3625, 3699, 3772, 3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349,
		4420, 4491, 4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181,
		5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839, 5903, 5967,
		6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461, 6521, 6581, 6640, 6699,
		6758, 6815, 6873, 6930, 6987, 7043, 7099, 7154, 7209, 7264, 7318, 7371,
		7424, 7477, 7529, 7581, 7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977,
		8025, 8072, 8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512,
		8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899, 8935, 8970,
		9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231, 9261, 9291, 9320, 9348,
		9376, 9403, 9429, 9455, 9481, 9506, 9530, 9554, 9577, 9599, 9621, 9642,
		9663, 9683, 9702, 9721, 9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850,
		9863, 9876, 9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969,
		9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000 };
float _sin(float a);							//checked
float _cos(float a);							//checked
float _normalizeAngle(float angle);				//checked
float _sqrtApprox(float number);				//checked

//Megnetic Sensor SPI function
void MagneticSensorSPI_init(struct MagneticSensorSPIConfig_s config); //checked
uint8_t spiCalcEvenParity(uint16_t value);							//checked
uint16_t read(uint16_t angle_register);								//checked
int getRawCount();													//checked
float getAngle();													//checked
float getvelocity();  					 							//checked
float getacceleration();

void Sensor_init();
void updateSensor();												//checked
float getMechanicalAngle();											//checked
float getAngle();													//checked
int32_t getFullRotations();												//

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs);
float electricalAngle();
void adaptiveZero();
int _normalizeIndex(int ind);
float fast_fmaxf(float x, float y);
float fast_fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
// shaft angle calculation
float shaftAngle();
// shaft velocity calculation
float shaftVelocity();
// shaft acceleration calculation
float shaftAcceleration();
//PWM function
void writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c);  //checked
void setPhaseVoltage(float Uq, float Ud, float angle_el);   //checked

//Motion control function
int needsSearch();											//checked
int absoluteZeroSearch();									//don't use
int alignSensor();											//checked
void initGain();
int initFOC(float zero_electric_offset, enum Direction _sensor_direction); //checked
float calSwitchThreshold(float setpoint, float feedback);
void loopFOC();

struct LPF LowPassFilter(struct LPF LPF); 					//checked
struct PID PID(struct PID PID);								//checked
struct PID_current_D PID_current_D(struct PID_current_D PID) ;

//Closed Loop
void SMC_velocity(float new_target);
void SMC_angle(float new_target);
void fusion_angle(float new_target);
void move_velocity(float new_target);		//closed loop			//checked
void move_angle(float new_target);	 //closed loop			//checked
void move_haptic(float new_target, float passivity_gain);
//Open Loop
float velocityOpenloop(float target_velocity);				//checked
void move_velocity_openloop(float target);					//checked
float angleOpenloop(float target_angle);					//
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Debug Global variable
uint16_t pwmSin[] = { 1440, 1627, 1812, 1991, 2160, 2316, 2458, 2582, 2687,
		2770, 2830, 2867, 2880, 2867, 2830, 2770, 2687, 2582, 2458, 2316, 2160,
		1991, 1812, 1627, 1440, 1252, 1067, 888, 720, 563, 421, 297, 192, 109,
		49, 12, 0, 12, 49, 109, 192, 297, 421, 563, 719, 888, 1067, 1252, 1440 };
int currentStepA, currentStepB, currentStepC;

struct PhaseCurrent_s sensortest;
float Sx, dtx = 0;
uint16_t temp;
uint8_t tempA[2];
float a, b, c;
uint16_t value[3];
uint32_t CAN_error;

int count1, count2;
float pp_check;

unsigned long t1 = 0, ts, t2 = 0, t3, t4 = 0, t5 = 0, period = 0;
uint8_t MSG[35] = { '\0' };
float Command_setpoint, LP_Command_setpoint, prev_Command_setpoint, offset_Command_setpoint;
float _acceleration, _velocity, _angle, debug_float, debug_zero, _setpoint, _1a, _2a, _3a;
uint16_t debug_uint16_t;
int debug_state = 1;
int motor_nbr = 0;

uint8_t setpoint[8];
uint8_t feedback[8];
float can_setpoint, prev_can_setpoint;

float motor_param[15][5] = { //   0  /   1   /   2   /   3   /   4    //   // motor_nbr
		{ 4.24989367f, 4.27136946f, 4.62f, 0.52f, 0.53f },   // 0  zero_electric_angle_avg
				{ 1.0f, 1.0f, 10.0f, 10.0f, 10.0f },   // 1  current_D.P
				{ 400.0f, 400.0f, 100.0f, 100.0f, 100.0f },   // 2  current_D.I
				{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },   // 3  current_D.D
				{ 1.0f, 0.8f, 1.0f, 1.0f, 1.0f },   // 4  current_Q.P
				{ 10.0f, 9.0f, 30.0f, 30.0f, 30.0f },   // 5  current_Q.I
				{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },   // 6  current_Q.D
				{ 0.5f,  0.5f, 1.0f, 1.0f, 1.0f },   // 7  velocity.P
				{ 3.0f, 3.0f, 20.0f, 20.0f, 20.0f },   // 8  velocity.I
				{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },   // 9  velocity.D
				{ 10.0f, 12.0f, 10.0f, 13.0f, 10.0f },   // 10 angle.P
				{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },   // 11 angle.I
				{ 0.05f, 0.05f, 0.05f, 0.1f, 0.05f },   // 12 angle.D
				{ 50.0f, 50.0f, 30.0f, 20.0f, 0.05f },   // 13 velocity limit
		};
float motor_param_SMC[14][5] = { //   0  /   1   /   2   /   3   /   4    //   // motor_nbr
				{ 2600.0    , 2400.0   , 2400.0f   , 0.0f, 0.0f },   // 0  k_vel
				{ 0.01      , 0.01     , 0.01f    , 0.0f, 0.0f },   // 1  ep_vel
				{ 10.0      , 1.0     , 8.0f      , 0.0f, 0.0f },   // 2  delta_vel
				{ 6000.0    , 7000.0   , 7000.0f   , 0.0f, 0.0f },   // 3  g_vel
				{ -30000.0  , -70000.0 , -70000.0f , 0.0f, 0.0f },   // 4  neta_vel
				{ 24.0      , 24.0    , 24.0f     , 0.0f, 0.0f },   // 5  current_limit
				{ 2600.0    , 90000.0    , 90000.0f   , 0.0f, 0.0f },   // 6  kp
				{ 2600.0    , 1000000.0    ,1000000.0f   , 0.0f, 0.0f },   // 7  ki
				{ 2600.0    , 10.0    , 2800.0f   , 0.0f, 0.0f },   // 8  k_ang
				{ 0.01      , 0.1      , 0.008f    , 0.0f, 0.0f },   // 9  ep_ang
				{ 10.0      , 10.0     , 8.0f      , 0.0f, 0.0f },   // 10  delta_ang
				{ 6000.0    , 1000.0   , 6000.0f   , 0.0f, 0.0f },   // 11  g_ang
				{ -30000.0  , -1000.0 , -70000.0f , 0.0f, 0.0f },   // 12  neta_ang
				{ 24.0      ,  1.0    , 24.0f     , 0.0f, 0.0f }    // 13  velocity_limit
};
float shaft_angle_pp[14][5] =
				{//     0     /      1      /      2      /      3       /        4      //  // motor_nbr
				{ 2.39224315, 2.39301014, 4.83165598f, 3.25625777f, 2.26223826f },	// 0  round 0
				{ 1.93741775, 1.93550026 , 5.27075815f, 3.6754179f, 2.70210719f },// 1  round 1
				{ 1.48757792, 1.48566043, 5.72328234f, 4.07578707f, 3.19413161f },	// 2  round 2
				{ 1.04540789, 1.04310691, 6.16468525f, 4.49264622f, 3.67081594f },	// 3  round 3
				{ 0.594034076, 0.592116594, 0.311781585f, 4.9290638f, 4.10838413f },	// 4  round 4
				{ 0.138825268, 0.133839831, 0.761237979f, 5.40459776f, 4.54671907f },	// 5  round 5
				{ 5.97869015, 5.97485495, 1.21376228f, 5.89355421f, 4.99962711f },	// 6  round 6
				{ 5.53000069, 5.52693272, 1.67203903f, 0.0839854479f, 5.44026279f },	// 7  round 7
				{ 5.07517529, 5.07287455, 2.13031578f, 0.50161171f, 5.84830189f },	// 8  round 8
				{ 4.63223839, 4.62801981, 2.58437419f, 0.903514683f, 0.0391165093f },	// 9  round 9
				{ 4.19428682, 4.18930149, 3.02155876f, 1.32574284f, 0.487422407f },	// 10 round 10
				{ 3.74483061, 3.74137902, 3.45222378f, 1.7748158f, 0.913869083f },	// 11 round 11
				{ 3.29307318, 3.29153919, 3.90091324f, 2.26223826f, 1.35220408f },	// 12 round 12
				{ 2.84706831, 2.84438396, 3.90091324f, 2.26223826f, 1.35220408f },	// 13 round 13
		};
float zero_angle_pp[14][5] =
				{ //      0   /      1     /      2     /       3      /        4      //      // motor_nbr
				{ 4.1593895, 4.19696808, 4.81132793f, 1.60531116f, 0.255408674f },// 0  round 0
				{ 4.2920785, 4.31892252, 4.67557287f, 1.19036627f,0.130390257f },	// 1  round 1
				{ 4.29591322, 4.33349562, 4.7277298f, 0.512352109f,0.73554498f },	// 2  round 2
				{ 4.20310783, 4.24605942, 4.62418652f, 0.0651958808f,1.12593985f },	// 3  round 3
				{ 4.24452448, 4.27673769, 4.36494207f, 0.0651958808f,0.968711972f },	// 4  round 4 6.17504406f
				{ 4.33426285, 4.40942764, 4.37414598f, 0.266143978f,0.822214901f },	// 5  round 5
				{ 4.2736764, 4.31662226, 4.42630148f, 0.828348994f,0.879743099f },	// 6  round 6
				{ 4.26676655, 4.30434895, 4.75899048f, 1.17579627f,0.765457332f },	// 7  round 7
				{ 4.372612, 4.37798309, 4.69167948f, 0.739378631f,0.194819346f },	// 8  round 8
				{ 4.30128717, 4.32276392, 4.76531363f, 0.082834594f,0.547631145f },	// 9  round 9
				{ 4.40331318, 4.18163013, 4.60271072f, 0.0651958808f,0.540728271f },	// 10 round 10 5.99402952f
				{ 3.95177872, 4.16936111, 4.34883404f, 0.0651958808f,0.227796897f },	// 11 round 11 5.99786568f
				{ 4.40561106, 4.18393183, 4.34730291f, 0.255408674f,0.0813007802f },	// 12 round 12
				{ 4.50408041, 4.16092062, 4.34730291f, 0.255408674f,0.0813007802f },	// 13 round 13
		};
int sensor_dir[5] = { CCW, CCW, CW, CW, CW };
//uint32_t can_id[4] = { 16, 128, 32, 64 };
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;
uint32_t count = 0;

int datacheck = 0;
float debug_float = 0;

bool state_speed_loop = 0;
bool state_position_loop = 0;
bool state_enable = 0;
bool state_limit = 0;
int state_control = 0;
float angle_limit = 0;

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
////	if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, RxData)
////			!= HAL_OK) {
////		Error_Handler();
////	}
//	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
////	datacheck++;
//
//	if (RxData[0] == 0xa2) {
//		state_speed_loop = 1;
//		state_position_loop = 0;
//		velocity_command();
//
//	} else if (RxData[0] == 0xa3) {
//		state_speed_loop = 0;
//		state_position_loop = 1;
//		position_command();
//	} else if (RxData[0] == 0x9c) {
//		feedback_command();
//	}
//	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
//			!= HAL_OK) {
//		/* Notification Error */
//		Error_Handler();
//	}
//}

//uint8_t countTest = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	motor_nbr = 1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_CRC_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */
	//Delay SETUP
	DWT_Init();
	//SPI SETUP
	MagneticSensorSPI_init(AS5048A_SPI);
	//POSITION SENSOR SETUP
	Sensor_init();
	//
	//Driver Setup
	state_enable = 1;

	voltage_sensor_align = 2; // aligning voltage [V]
	velocity_index_search = 2; // index search velocity [rad/s]

	voltage_power_supply = 24.0;

	voltage_limit = 24.0;
	current_limit = 24.0;
	velocity_limit = 30.0;

	Command_setpoint = 0.0;
	offset_Command_setpoint = -2.0;
	LPF_Command_setpoint.Tf = 0.03;
	initGain();
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);  // Enable

	//CURRENT SENSE SETUP
	HAL_ADC_Start_DMA(&hadc1, adcResultDMA_a, 1);
	HAL_ADC_Start_DMA(&hadc2, adcResultDMA_b, 1);
	initCurrentsense(CurrentSense_resistance, CurrentSense_gain);
	calibrateOffsets();
	//
	//PWM SETUP
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  //pinMode
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	//pinMode
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	//pinMode

//
//	//Control system configuration

//	//====Motor====

	zero_electric_angle_avg = motor_param[0][motor_nbr];
	initFOC(motor_param[0][motor_nbr], sensor_dir[motor_nbr]);
//	initFOC(zero_electric_angle, UNKNOWN); //Not yet calibrate find the best init value
//	while (1);
//
//	for (int i = 0; i < 100; i++) {
//		updateSensor();
//		Command_setpoint = shaftAngle();
//	}
//	//CAN Transmit
//	HAL_FDCAN_Start(&hfdcan1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

//	TxHeader.Identifier = 0x99	;
//	TxHeader.IdType = FDCAN_STANDARD_ID;
//	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//	TxHeader.MessageMarker = 0;

	t1 = 0;
	t2 = micros();
	_1a = 0;
	_2a = 0;
	_3a = 0;

//	TxData[0] = 0x00;
//	TxData[1] = 0x00;
//	TxData[2] = 0x00;
//	TxData[3] = 0x00;
//	TxData[4] = 0x00;
//	TxData[5] = 0x00;
//	TxData[6] = 0x00;
//	TxData[7] = 0x00;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		//=====================================================//
		//													   //
		//												       //
		//                   Don't comment                     //
		//													   //
		//													   //
		//=====================================================//
		updateSensor();
		adaptiveZero();

		debug_current_abc = getPhaseCurrents();
		debug_current_dq = current;
		_velocity = shaftVelocity();
		_angle = shaftAngle() - offset_Command_setpoint;
		_acceleration = shaftAcceleration() ;
		_setpoint = Command_setpoint ;

		period = micros() - t1;
		t1 = micros();

//		state_limit = HAL_GPIO_ReadPin(LIMIT_GPIO_Port, LIMIT_Pin); // limit = 1
//		if (Command_setpoint != prev_Command_setpoint)
//			state_enable = 1;
		if ((current_D.output_prev >= 23.9) || (current_D.output_prev < -23.9))
			state_enable = 0;
//		if (state_enable == 0)
//			Command_setpoint = shaftAngle();
		if (Command_setpoint != prev_Command_setpoint)
			if((Command_setpoint <= 0.1)&&(Command_setpoint >= -0.1))
				current_D.integral_prev = 0.0f ;
		HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, state_enable);
		prev_Command_setpoint = Command_setpoint;
//		int16_t debug_int_current_q = (int16_t) (debug_current_dq.q * 1000);
//		i2b_current.asInt16_t = debug_int_current_q;
//		int16_t debug_int_velocity = (int16_t) (debug_float_velocity * 100);
//		i2b_velocity.asInt16_t = debug_int_velocity;
//		int16_t debug_int_angle = (int16_t) (debug_float_angle * 100);
//		i2b_angle.asInt16_t = debug_int_angle;

		//=====================================================//
		//												   	   //
		//													   //
		//                  Hardware Testing                   //
		//													   //
		//													   //
		//====================PWM testing======================//
//		  	  if(micros() - t1 >= 10000)
//		  	  {
//		  		  t1 = micros();
//		  	 	  debug_float = debug_float + 0.00201;
//		  	  	  if(debug_float > 0.94)
//		  	  	  	  debug_float = 0.0;
//		  	  }
//		LPF_Command_setpoint.x = Command_setpoint;
//		LPF_Command_setpoint = LowPassFilter(LPF_Command_setpoint);
//		LP_Command_setpoint = LPF_Command_setpoint.y_prev;  // filter values
//		  	  writeDutyCycle3PWM(LP_Command_setpoint , 0.0 , 0.0);

		//===============Current Sense testing=================//
//	  	  	  writeDutyCycle3PWM(0.2 , 0.5, 0.8);
		//  	  debug_current_abc = getPhaseCurrents() ;

		//==============Position Sensor testing================//
		//	  	  updateSensor();
		//	  	  shaft_angle = shaftAngle();
		//	  	  shaft_velocity = shaftVelocity();

		//=================Open-loop testing===================//
		//  	  Command_setpoint = 10.0 ;
//	  		move_velocity_openloop(Command_setpoint);

		//=====================================================//
		//													   //
		//													   //
		//                      Tuning                         //
		//													   //
		//													   //
		//===========Tune Current in STM32CubeMonitor==========//
		//	  	  current_sp = Command_setpoint ;
		//	  	  loopFOC();

//		//============Tune Speed in STM32CubeMonitor===========//
//		LPF_Command_setpoint.x = Command_setpoint;
//		LPF_Command_setpoint = LowPassFilter(LPF_Command_setpoint);
//		LP_Command_setpoint = LPF_Command_setpoint.y_prev;	// filter values
//		move_velocity(LP_Command_setpoint);
//		loopFOC();

		//===========Tune Position in STM32CubeMonitor=========//
//		LPF_Command_setpoint.x = Command_setpoint;
//		LPF_Command_setpoint = LowPassFilter(LPF_Command_setpoint);
//		LP_Command_setpoint = LPF_Command_setpoint.y_prev;   // filter values
//		move_angle(LP_Command_setpoint);
//		loopFOC();
//		//=====================================================//
//		//													   //
//		//													   //
//		//                      Testing                        //
//		//													   //
//		//													   //
//		//===============Test forward reverse drive============//
//		//	  	  updateSensor();
//			  	  if(micros() - t1 >= 3000000)
//			  	  {
//			  	    t1 = micros();
//		if (LP_Command_setpoint > 0.0)
//			writeDutyCycle3PWM(LP_Command_setpoint , 0.0 , 0.0);
//		else if (LP_Command_setpoint <= 0.0)
//			writeDutyCycle3PWM((-1.0)*(LP_Command_setpoint), 0.0 , 0.0);
//				writeDutyCycle3PWM(0.0, 0.0 , 0.0);

//			  		count += 1 ;
//			  	  }
		//=====================================================//
		//													   //
		//													   //
		//                      CAN bus                        //
		//													   //
		//													   //
		//=====================================================//

//		if (state_speed_loop == 1) {
//			if (state_enable == 0) {
//				Command_setpoint = shaftAngle();
//				if (can_setpoint != prev_can_setpoint)
//				state_enable = 1;
//			}
//			else if (state_enable == 1){
//				Command_setpoint = can_setpoint;
//			}
//			LPF_Command_setpoint.x = Command_setpoint;
//			LPF_Command_setpoint = LowPassFilter(LPF_Command_setpoint);
//			LP_Command_setpoint = LPF_Command_setpoint.y_prev;  // filter values
//			move_velocity(LP_Command_setpoint);
//			loopFOC();
//			prev_can_setpoint = can_setpoint;
//		}
//
//		else if (state_position_loop == 1) {
//			if (state_enable == 0) {
//				Command_setpoint = shaftAngle();
//				if (can_setpoint != prev_can_setpoint)
//				state_enable = 1;
//			}
//			else if (state_enable == 1){
//				Command_setpoint = can_setpoint;
//			}
//			LPF_Command_setpoint.x = Command_setpoint;
//			LPF_Command_setpoint = LowPassFilter(LPF_Command_setpoint);
//			LP_Command_setpoint = LPF_Command_setpoint.y_prev;  // filter values
//			move_angle(LP_Command_setpoint);
//			loopFOC();
//			prev_can_setpoint = can_setpoint;
//		}
//		feedback_command();

//		if (micros() - t1 >= 20000) {
//			t1 = micros();
//			count = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
//		}
		//=====================================================//
		//													   //
		//													   //
		//                        SMC                          //
		//													   //
		//													   //
		//=====================================================//

		LPF_Command_setpoint.x = Command_setpoint + offset_Command_setpoint;
		LPF_Command_setpoint = LowPassFilter(LPF_Command_setpoint);
		LP_Command_setpoint = LPF_Command_setpoint.y_prev;  // filter values
		if (state_control == 1)
			move_angle(LP_Command_setpoint);
		else if (state_control == 0)
			SMC_angle(LP_Command_setpoint);
//			SMC_velocity(LP_Command_setpoint);
		else if (state_control == 2)
			fusion_angle(LP_Command_setpoint);
		loopFOC();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = NomPS;
  hfdcan1.Init.NominalSyncJumpWidth = 11;
  hfdcan1.Init.NominalTimeSeg1 = 73;
  hfdcan1.Init.NominalTimeSeg2 = 11;
  hfdcan1.Init.DataPrescaler = DataPS;
  hfdcan1.Init.DataSyncJumpWidth = 15;
  hfdcan1.Init.DataTimeSeg1 = 18;
  hfdcan1.Init.DataTimeSeg2 = 15;
  hfdcan1.Init.StdFiltersNbr = 10;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	FDCAN_FilterTypeDef CAN1Filter;

	CAN1Filter.IdType = FDCAN_STANDARD_ID;
	CAN1Filter.FilterIndex = 0;
	CAN1Filter.FilterType = FDCAN_FILTER_MASK;
	CAN1Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//	can_id[4] = { 16, 128, 32, 64 };
//	CAN1Filter.FilterID1 = can_id[motor_nbr];
//	CAN1Filter.FilterID2 = can_id[motor_nbr];

	if (motor_nbr == 0) {
		CAN1Filter.FilterID1 = 0x80;
		CAN1Filter.FilterID2 = 0x80;
//		datacheck = 1 ;
	}
	if (motor_nbr == 1) {
		CAN1Filter.FilterID1 = 0x16;
		CAN1Filter.FilterID2 = 0x16;
//		datacheck = 2 ;
	}
	if (motor_nbr == 2) {
		CAN1Filter.FilterID1 = 0x24;
		CAN1Filter.FilterID2 = 0x24;
//		datacheck = 3 ;
	}
	if (motor_nbr == 3) {
		CAN1Filter.FilterID1 = 0x32;
		CAN1Filter.FilterID2 = 0x32;
//		datacheck = 4 ;
	}
	if (motor_nbr == 4) {
		CAN1Filter.FilterID1 = 0x69;
		CAN1Filter.FilterID2 = 0x69;
//		datacheck = 4 ;
	}
	HAL_FDCAN_ConfigFilter(&hfdcan1, &CAN1Filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 3600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_CON_Pin|ENABLEA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_CON_Pin ENABLEA15_Pin */
  GPIO_InitStruct.Pin = CS_CON_Pin|ENABLEA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
__STATIC_INLINE void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Data watchpoint trigger(DWT)
}

__STATIC_INLINE void delay_us(uint32_t us) {
	uint32_t us_count_tic = us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while (DWT->CYCCNT < us_count_tic)
		;
}

__STATIC_INLINE uint32_t micros(void) {
	return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void float2Bytes(float val, uint8_t *bytes_array) {
// Create union of shared memory space
	union {
		float float_variable;
		uint8_t temp_array[4];
	} u;
// Overite bytes of union with float variable
	u.float_variable = val;
// Assign bytes to input array
	memcpy(bytes_array, u.temp_array, 4);
}

float Bytes2float(uint8_t *bytes_array) {
	uint8_t temp_array[4] = { bytes_array[0], bytes_array[1], bytes_array[2],
			bytes_array[3] };
	float f = *(float*) &temp_array;
	return f;

}

void velocity_command() {
//	setpoint[0] = RxData[0];  //command byte
//	setpoint[1] = RxData[1];  //null
//	setpoint[2] = RxData[2];  //vel_limit
//	setpoint[3] = RxData[3];  //vel_limit
//	setpoint[4] = RxData[4];  //float velocity[0]
//	setpoint[5] = RxData[5];  //float velocity[1]
//	setpoint[6] = RxData[6];  //float velocity[2]
//	setpoint[7] = RxData[7];  //float velocity[3]
	b2i_limit_vel.asUint8_t[0] = RxData[2];
	b2i_limit_vel.asUint8_t[1] = RxData[3];
	b2f.asUint8_t[0] = RxData[4];
	b2f.asUint8_t[1] = RxData[5];
	b2f.asUint8_t[2] = RxData[6];
	b2f.asUint8_t[3] = RxData[7];

	voltage_limit = 24.0;
	current_limit = 24.0;
	velocity_limit = (float) (b2i_limit_vel.asInt16_t);
	can_setpoint = b2f.asFloat;
}

void position_command() {
//	setpoint[0] = RxData[0];  //command byte
//	setpoint[1] = RxData[1];  //null
//	setpoint[2] = RxData[2];  //vel_limit
//	setpoint[3] = RxData[3];  //vel_limit
//	setpoint[4] = RxData[4];  //angle[0]m
//	setpoint[5] = RxData[5];  //angle[1]
//	setpoint[6] = RxData[6];  //angle[2]
//	setpoint[7] = RxData[7];  //angle[3]
	b2i_limit_vel.asUint8_t[0] = RxData[2];
	b2i_limit_vel.asUint8_t[1] = RxData[3];
	b2f.asUint8_t[0] = RxData[4];
	b2f.asUint8_t[1] = RxData[5];
	b2f.asUint8_t[2] = RxData[6];
	b2f.asUint8_t[3] = RxData[7];

	voltage_limit = 24.0;
	current_limit = 24.0;
	velocity_limit = (float) (b2i_limit_vel.asInt16_t);
	can_setpoint = b2f.asFloat;
}

void feedback_command() {
	TxData[0] = 0x00;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;
	if (motor_nbr == 0) {
		TxData[0] = 0x80;
//		datacheck = 1 ;
	} else if (motor_nbr == 1) {
		TxData[0] = 0x16;
//		datacheck = 2 ;
	} else if (motor_nbr == 2) {
		TxData[0] = 0x24;
//		datacheck = 3 ;
	} else if (motor_nbr == 3) {
		TxData[0] = 0x32;
//		datacheck = 4 ;
	}

//	TxData[0] = 0x9c;  //command byte
	TxData[1] = state_limit;  //null
	TxData[2] = i2b_current.asUint8_t[0];  //current_Q[0]
	TxData[3] = i2b_current.asUint8_t[1];  //current_Q[1]
	TxData[4] = i2b_velocity.asUint8_t[0];  //velocity[0]
	TxData[5] = i2b_velocity.asUint8_t[1];  //velocity[1]
	TxData[6] = i2b_angle.asUint8_t[0];  //angle[0]
	TxData[7] = i2b_angle.asUint8_t[1];  //angle[1]

//	TxData[1] = state_limit;  //null
//	TxData[2] = 0x00;  //current_Q[0]
//	TxData[3] = 0x00;  //current_Q[1]
//	TxData[4] = 0x00;  //velocity[0]
//	TxData[5] = 0x00;  //velocity[1]
//	TxData[6] = 0x00;  //angle[0]
//	TxData[7] = 0x00;  //angle[1]
//	TxData[6] = i2b_angle.asUint8_t[0];  //angle[0]
//	TxData[7] = i2b_angle.asUint8_t[1];  //angle[1]

//	int16_t tmp_pos = (int16_t)(debug_float_angle*100.0f);

//	TxData[6] = (uint8_t)(tmp_pos << 8) & 0xFF;
//	TxData[7] = (uint8_t)(tmp_pos & 0xFF);

//	TxData[6] = i2b_angle.asUint8_t[0];  //angle[0]
//	TxData[7] = i2b_angle.asUint8_t[1];  //angle[1]

//	debug_float_angle
//	datacheck++ ;
//	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

}

//Initialize Current Sensor
void initCurrentsense(float _shunt_resistor, float _gain) {
	R_sense = _shunt_resistor;
	gain_a = _gain;
	gain_b = _gain;
	gain_c = _gain;
}

//Calibrate Offset current sensor
void calibrateOffsets() {
	const int calibration_rounds = 1000;
// find adc offset = zero current voltage
	offset_ia = 0;
	offset_ib = 0;
	offset_ic = 0;
// read the adc voltage 1000 times ( arbitrary number )
	for (int i = 0; i < calibration_rounds; i++) {
		offset_ia += adcResultDMA_a[0];
		offset_ib += adcResultDMA_b[0];
//		offset_ic += adcResultDMA_c[0];
		HAL_Delay(1);
	}
// calculate the mean offsets
	offset_ia = offset_ia / calibration_rounds;
	offset_ib = offset_ib / calibration_rounds;
//	offset_ic = offset_ic / calibration_rounds;
}

//// read all three phase currents (if possible 2 or 3)
struct PhaseCurrent_s getPhaseCurrents() {
	struct PhaseCurrent_s current;
	current.a = ((offset_ia - adcResultDMA_a[0]) * (3.3 / 4096.0))
			/ (R_sense * gain_a);
	current.b = ((offset_ib - adcResultDMA_b[0]) * (3.3 / 4096.0))
			/ (R_sense * gain_b);
//	current.c = ((offset_ic - adcResultDMA_c[0]) * (3.3 / 4096.0))
//			/ (R_sense * gain_c);
//    current.c = -current.a-current.b;
//    current.rms = _sqrtApprox(
//			powf(debug_current_abc.a, 2) + powf(debug_current_abc.b, 2)
//					+ powf(debug_current_abc.c, 2));
	return current;
}

// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents
//   - using getPhaseCurrents internally
struct DQCurrent_s getFOCCurrents(float angle_el) {
// read current phase currents
	struct PhaseCurrent_s current = getPhaseCurrents(); //Ia,Ib,Ic

// calculate clarke transform
	float i_alpha, i_beta;
//    if(!current.c)
//    {
// 3 phase measured
//        i_alpha = (0.6666667f) * (current.a + (-(0.5) * current.b) + (-(0.5) * current.c));
//        i_beta =  (0.6666667f) * ((_SQRT3_2) * current.b) + (-(_SQRT3_2) * current.c);
// if only two measured currents
// phase a - c
//	i_alpha = current.a;
//	i_beta = (-(_1_SQRT3) * current.a) + (-(_2_SQRT3) * current.c);
// phase a - b
	i_alpha = current.a;
	i_beta = (_1_SQRT3) * current.a + (_2_SQRT3) * current.b;
//    }
//    else
//    {
//        // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
//        float mid = (1.f/3) * (current.a + current.b + current.c);
//        float a = current.a - mid;
//        float b = current.b - mid;
//        i_alpha = a;
//        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
//    }

// calculate park transform
	float _ct = _cos(angle_el);
	float _st = _sin(angle_el);

	struct DQCurrent_s return_current;
	return_current.d = i_alpha * _ct + i_beta * _st;
	return_current.q = i_beta * _ct - i_alpha * _st;
	return return_current;
}

// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin(float a) {
	if (a < _PI_2) {
		//return sine_array[(int)(199.0*( a / (_PI/2.0)))];
		//return sine_array[(int)(126.6873* a)];           // float array optimized
		return 0.0001f * sine_array[_round(126.6873f * a)]; // int array optimized
	} else if (a < _PI) {
		// return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
		//return sine_array[398 - (int)(126.6873*a)];          // float array optimized
		return 0.0001f * sine_array[398 - _round(126.6873f * a)]; // int array optimized
	} else if (a < _3PI_2) {
		// return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
		//return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
		return -0.0001f * sine_array[-398 + _round(126.6873f * a)]; // int array optimized
	} else {
		// return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
		//return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
		return -0.0001f * sine_array[796 - _round(126.6873f * a)]; // int array optimized
	}
}
// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a) {
	float a_sin = a + _PI_2;
	a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
	return _sin(a_sin);
}

//normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle) {
	float a = fmod(angle, _2PI);
	return a >= 0 ? a : (a + _2PI);      //always project from 0 degree
}

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs) {
	return (shaft_angle * pole_pairs);
}

// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox(float number) {
//low in fat
	long i;
	float y;
// float x;
// const float f = 1.5F; // better precision

// x = number * 0.5F;
	y = number;
	i = *(long*) &y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float*) &i;
// y = y * ( f - ( x * y * y ) ); // better precision
	return number * y;
}

// initialize SPI for Magnetic Sensor
void MagneticSensorSPI_init(struct MagneticSensorSPIConfig_s config) {
// angle read register of the magnetic sensor
	angle_register = config.angle_registers ? config.angle_registers : DEF_ANGLE_REGISTER;
// register maximum value (counts per revolution)
	cpr = pow(2, config.bit_resolution);
	bit_resolution = config.bit_resolution;

	command_parity_bit = config.command_parity_bit; // for backwards compatibility
	command_rw_bit = config.command_rw_bit; // for backwards compatibility
	data_start_bit = config.data_start_bit; // for backwards compatibility

	HAL_GPIO_WritePin(CS_CON_GPIO_Port, CS_CON_Pin, GPIO_PIN_SET);
}

/**
 * Utility function used to calculate even parity of word
 */
uint8_t spiCalcEvenParity(uint16_t value) {
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++) {
		if (value & 0x1)
			cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
uint16_t read(uint16_t angle_register) {
	uint16_t register_value;
	uint16_t command = angle_register;

	if (command_rw_bit > 0) {
		command = angle_register | (1 << command_rw_bit);
	}
	if (command_parity_bit > 0) {
		//Add a parity bit on the the MSB
		command |= ((uint16_t) spiCalcEvenParity(command) << command_parity_bit);
	}

	command = 0xFFFF;
//Now read the response (NO_OPERATION_COMMAND = 0x0000)
//  uint16_t register_value = spi->transfer16(0x00);
	HAL_GPIO_WritePin(CS_CON_GPIO_Port, CS_CON_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &command,
			(uint8_t*) &register_value,
			sizeof(register_value) / sizeof(uint16_t), 100);
	HAL_GPIO_WritePin(CS_CON_GPIO_Port, CS_CON_Pin, GPIO_PIN_SET);

//SPI - end transaction

	register_value = register_value >> (1 + data_start_bit - bit_resolution); //this should shift data to the rightmost bits of the word
	uint16_t data_mask = 0xFFFF >> (16 - bit_resolution);
	temp = register_value & data_mask;
	return register_value & data_mask; // Return the data, stripping the non data (e.g parity) bits
}

// function reading the raw counter of the magnetic sensor
int getRawCount() {
	return (int) read(angle_register);
}

//  Shaft angle calculation
//  angle is in radians [rad]
float getSensorAngle() {
	return (getRawCount() / (float) cpr) * _2PI;
}

void Sensor_init() {
// initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
	getSensorAngle(); // call once

	vel_angle_prev = getSensorAngle(); // call again
	vel_angle_prev_ts = micros();
	HAL_Delay(1);
	getSensorAngle(); // call once

	angle_prev = getSensorAngle(); // call again
	angle_prev_ts = micros();
}

void updateSensor() {
	float val = getSensorAngle();
	angle_prev_ts = micros();
	float d_angle = val - angle_prev;
// if overflow happened track it as full rotation
	if (abs(d_angle) > (0.8f * _2PI))
		full_rotations += (d_angle > 0) ? -1 : 1;
	angle_prev = val;
}

float getMechanicalAngle() {
	return angle_prev;
}

float getAngle() {
	return (float) full_rotations * _2PI + angle_prev;
}

int32_t getFullRotations() {
	return full_rotations;
}

float getvelocity() {
// calculate sample time
	vel_angle_ts = micros();
	float Ts = (vel_angle_ts - vel_angle_prev_ts) * 1e-6;
// quick fix for strange cases (micros overflow)
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;
// velocity calculation
	float vel = ((float) (full_rotations - vel_full_rotations) * _2PI
			+ (angle_prev - vel_angle_prev)) / Ts;
// save variables for future pass
	vel_angle_prev = angle_prev;
	vel_full_rotations = full_rotations;
	vel_angle_prev_ts = vel_angle_ts;
	return vel;
}

float getacceleration() {
// calculate sample time
	acc_ts = micros() ;
	float Ts = (acc_ts - acc_prev_ts) * 1e-6;
// quick fix for strange cases (micros overflow)
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;
	float temp = (int)(shaftVelocity() * 100 +0.5);
	acc_vel = (float)temp/100 ;
	float acc = (acc_vel_prev - acc_vel) / Ts ;

	acc_vel_prev = acc_vel ;
	acc_prev_ts = acc_ts ;
	return acc ;
//	// calculate sample time
//		acc_ts = micros() ;
//
//		float Ts = (acc_ts - acc_prev_ts) * 1e-6;
//	// quick fix for strange cases (micros overflow)
//		if (Ts <= 0 || Ts > 0.0008f)
//			Ts = 0.0008f;
//		float temp = (int)(shaftAngle() * 10 +0.5);
//		zeta_1 = (float)temp/10 ;
//		float acc = (zeta_3 - (2 * zeta_2) + zeta_1) / (Ts * Ts) ;
//
//		zeta_3 = zeta_2;
//		zeta_2 = zeta_1 ;
//		acc_prev_ts = acc_ts ;
//		return acc ;
}
// shaft angle calculation
float shaftAngle() {
	LPF_angle.x = getAngle();
	LPF_angle = LowPassFilter(LPF_angle);
	return sensor_direction * LPF_angle.y_prev - sensor_offset;
}
// shaft velocity calculation
float shaftVelocity() {
	LPF_velocity.x = getvelocity();
	LPF_velocity = LowPassFilter(LPF_velocity);
	return sensor_direction * LPF_velocity.y_prev;
}

float shaftAcceleration() {
	LPF_acceleration.x = getacceleration();
	LPF_acceleration = LowPassFilter(LPF_acceleration);
	return sensor_direction * LPF_acceleration.y_prev;
}
//Conversion shaft angle to elec angle
float electricalAngle() {
	return _normalizeAngle(
			(float) (sensor_direction * pole_pairs) * getMechanicalAngle()
					- zero_electric_angle);
}
void adaptiveZero() {
	float min_value = 99.0;
	int min_index = 0;
	float value;
	for (int i = 0; i < pole_pairs; i++) {
		value = fabs(angle_prev - shaft_angle_pp[i][motor_nbr]);
		if (value < min_value) {
			min_value = value;
			min_index = i;
		}
	}
	zero_electric_angle = zero_angle_pp[min_index][motor_nbr];
//	return _normalizeAngle(zero_electric_angle);
}
//normalizing index pole pairs to [0,pole pairs]
int _normalizeIndex(int ind) {
	return (ind + pole_pairs) % pole_pairs;    //always project from 0 degree
}
float fast_fmaxf(float x, float y) {
/// Returns maximum of x, y ///
	return (((x) > (y)) ? (x) : (y));
}

float fast_fminf(float x, float y) {
/// Returns minimum of x, y ///
	return (((x) < (y)) ? (x) : (y));
}

float fmaxf3(float x, float y, float z) {
/// Returns maximum of x, y, z ///
	return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z) {
/// Returns minimum of x, y, z ///
	return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

//Write PWM fsw = 25kHzfloat Ts
void writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c) {
//	dc_a = 1.0f - dc_a ;
//	dc_b = 1.0f - dc_b ;
//	dc_c = 1.0f - dc_c ;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, fsw*dc_b);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fsw*dc_c);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fsw*dc_a);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
void setPhaseVoltage(float Uq, float Ud, float angle_el) {
// Sinusoidal PWM modulation ---------------------------------------------------------
// Inverse Park + Clarke transformation

// angle normalization in between 0 and 2pi
// only necessary if using _sin and _cos - approximation functions
	angle_el = _normalizeAngle(angle_el);
// Inverse park transform
	float _ca = _cos(angle_el);
	float _sa = _sin(angle_el);
	float iv_alpha, iv_beta;

	iv_alpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
	iv_beta = _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

// center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
	float center = voltage_limit / 2;
// Inverse Clarke transform
	Ua = iv_alpha + center;
	Ub = -0.5f * iv_alpha + _SQRT3_2 * iv_beta + center;
	Uc = -0.5f * iv_alpha - _SQRT3_2 * iv_beta + center;

//  Space Vector PWM modulation ------------------------------------------------------
//  float Uout;
//  // a bit of optitmisation
//  if(Ud)
//  {
//	// only if Ud and Uq set
//    // _sqrt is an approx of sqrt (3-4% error)
//    Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_limit;
//    // angle normalisation in between 0 and 2pi
//    // only necessary if using _sin and _cos - approximation functions
//    angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
//  }
//  else
//  {
//	// only Uq available - no need for atan2 and sqrt
//    Uout = Uq / voltage_limit;
//    // angle normalisation in between 0 and 2pi
//    // only necessary if using _sin and _cos - approximation functions
//    angle_el = _normalizeAngle(angle_el + _PI_2);
//  }
//  // find the sector we are in currently
//  int sector = floor(angle_el / _PI_3) + 1;
//  // calculate the duty cycles
//  float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
//  float T2 = _SQRT3*_sin(angle_el - (sector-1.0f)*_PI_3) * Uout;
////  float T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2
//  float T0 = 0; // pulled to 0 - better for low power supply voltage
//
//
//  // calculate the duty cycles(times)
//  float Ta, Tb, Tc;
//  switch (sector)
//  {
//    case 1:
//      Ta = T1 + T2 + T0 / 2;
//      Tb = T2 + T0 / 2;
//      Tc = T0 / 2;
//      break;
//    case 2:
//      Ta = T1 +  T0 / 2;
//      Tb = T1 + T2 + T0 / 2;
//      Tc = T0 / 2;
//      break;
//    case 3:
//      Ta = T0 / 2;
//      Tb = T1 + T2 + T0 / 2;
//      Tc = T2 + T0 / 2;
//      break;
//    case 4:
//      Ta = T0 / 2;
//      Tb = T1 + T0 / 2;
//      Tc = T1 + T2 + T0 / 2;
//      break;
//    case 5:
//      Ta = T2 + T0 / 2;
//      Tb = T0 / 2;
//      Tc = T1 + T2 + T0 / 2;
//      break;
//    case 6:
//      Ta = T1 + T2 + T0 / 2;
//      Tb = T0 / 2;
//      Tc = T1 + T0 / 2;
//      break;
//    default:
//      // possible error state
//      Ta = 0;
//      Tb = 0;
//      Tc = 0;
//  }
//
//  // calculate the phase voltages
//  Ua = Ta * voltage_limit;
//  Ub = Tb * voltage_limit;
//  Uc = Tc * voltage_limit;
//--------------------------------------------------------------

// set the voltages in hardware
// limit the voltage in driver
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
// calculate duty cycle
	float dc_a;  //duty cycle phase A [0, 0.94]
	float dc_b;  //duty cycle phase B [0, 0.94]
	float dc_c;  //duty cycle phase C [0, 0.94]
// limited in [0,1]
	dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 0.94f);
	dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 0.94f);
	dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 0.94f);
	writeDutyCycle3PWM(dc_a, dc_b, dc_c);
}

// returns 0 if it does need search for absolute zero
// 0 - magnetic sensor (& encoder with index which is found)
// 1 - encoder with index (with index not found yet)
int needsSearch() {
	return 0;
}
// Encoder alignment the absolute zero angle
// - to the index
int absoluteZeroSearch() {
// search the absolute zero with small velocity
	float limit_vel = velocity_limit;
	float limit_volt = voltage_limit;
	velocity_limit = velocity_index_search;
	voltage_limit = voltage_sensor_align;
	shaft_angle = 0;
	while (needsSearch() && shaft_angle < _2PI) {
		angleOpenloop(1.5 * _2PI);
		// call important for some sensors not to loose count
		// not needed for the search
		getAngle();
	}
	setPhaseVoltage(0, 0, 0);

// re-init the limits
	velocity_limit = limit_vel;
	voltage_limit = limit_volt;
	return !needsSearch();
}
// Encoder alignment to electrical 0 angle
int alignSensor() {
	int exit_flag = 1; //success
// if unknown natural direction
	if (!_isset(sensor_direction)) {
		// check if sensor needs zero search
		if (needsSearch()) //needSearch == 0 because use Magnetic sensor
			exit_flag = absoluteZeroSearch(); // o
		if (!exit_flag)
			return exit_flag;

		// find natural direction
		// move one electrical revolution forward
		for (int i = 0; i <= 500; i++) {
			float angle = _3PI_2 + _2PI * i / 500.0f;
			setPhaseVoltage(voltage_sensor_align, 0, angle);
			updateSensor();
			HAL_Delay(1);
		}
		updateSensor();
		// take and angle in the middle
		float mid_angle = getAngle();
		// move one electrical revolution backwards
		for (int i = 500; i >= 0; i--) {
			float angle = _3PI_2 + _2PI * i / 500.0f;
			setPhaseVoltage(voltage_sensor_align, 0, angle);
			updateSensor();
			HAL_Delay(1);
		}
		updateSensor();
		float end_angle = getAngle();
		setPhaseVoltage(0, 0, 0);
		HAL_Delay(100);
		// determine the direction the sensor moved
		if (mid_angle == end_angle) {
			return 0; // failed calibration
		} else if (mid_angle < end_angle) {
			sensor_direction = CCW;
		} else {
			sensor_direction = CW;
		}
	}

// zero electric angle not known
	if (!_isset(zero_electric_angle)) {
		// align the electrical phases of the motor and sensor
		// set angle -90(270 = 3PI/2) degrees
		for (int round = 0; round < pole_pairs; round++) {
			zero_electric_angle = 0.0;
			for (int k = 100; k >= 0; k--) {
				setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);
				updateSensor();
				HAL_Delay(1);
			}
//			setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);
//			HAL_Delay(500);
			updateSensor();
			align.zero_angle_pp[round] = electricalAngle();	//radian
//		if ((align.zero_angle_pp[round] < _2PI)
//				&& (align.zero_angle_pp[round] > 5.5)) {
//			if ((align.zero_angle_pp[round] < 1.0)
//					&& (align.zero_angle_pp[round] > 0.0)) {
//			align.zero_angle_pp[round] = align.zero_angle_pp[round] + _2PI;
//		}
//			align.shaft_angle_pp[round] = getAngle() * (360.0 / _2PI);	//degree
			align.shaft_angle_pp[round] = getMechanicalAngle();	//rad
//    zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
			HAL_Delay(20);
			// stop everything
			setPhaseVoltage(0, 0, 0);
			HAL_Delay(100);
			for (int j = 0; j <= 100; j++) {
				float angle = _3PI_2 + _2PI * j / 100.0f;
				setPhaseVoltage(voltage_sensor_align, 0, angle);
				updateSensor();
				HAL_Delay(1);
			}
		}
		setPhaseVoltage(0, 0, 0);
		HAL_Delay(100);
		for (int round = 0; round < pole_pairs; round++) {
			zero_electric_angle = zero_electric_angle
					+ align.zero_angle_pp[round];
		}
		zero_electric_angle = zero_electric_angle / pole_pairs;
	}
	return exit_flag;
}
void initGain() {

	CurrentSense_resistance = 0.22;
	CurrentSense_gain = 5.0;
	phase_resistance = 9.526; 	// ht4315 = 9.526     , mg4005 = 1.4
	phase_inductance = 0.00245; // ht4315 = 0.00245   , mg4005 = 0.0005
	voltage_power_supply = 24.0;
	pole_pairs = 14;			// ht4315 = 14        , mg4005 =
	// flux_linkage = 60/(sqrt(3)*poles*pi*kv)   ref : https://vesc-project.com/node/52
	flux_linkage = 0.01968;     // ht4315 = 0.01968   , mg4005 = 0.0075
	//gear_ratio                // ht4315 = 1:1       , mg4005 = 10:1
	//inertia_load = (gear_ratio^2) * inertia_motor
	inertia = 0.0000268;        // ht4315 = 0.0000268 , mg4005 = 0.0014
	damping = 0.0;              // ht4315 = 0.0       , mg4005 =


	//The detection box can still calculate the kp and ki parameters.   ref : https://vesc-project.com/node/52
	//The observer gain can be calculates roughly as 1000 / (lambda^2) where lambda is in mWb.

	LPF_SMC_out.y_prev = 0.0;
	LPF_SMC_out.Tf = 0.0001;
	SMC_ang.limit = motor_param_SMC[11][motor_nbr] ;
	SMC_ang.k = motor_param_SMC[8][motor_nbr] ;
	SMC_ang.ep = motor_param_SMC[9][motor_nbr] ;
	SMC_ang.delta = motor_param_SMC[10][motor_nbr] ;
	SMC_ang.g = motor_param_SMC[11][motor_nbr] ; // convergence rate of disturbance estimation error
	SMC_ang.neta = motor_param_SMC[12][motor_nbr] ;

	SMC_vel.limit = motor_param_SMC[5][motor_nbr];
	SMC_vel.k = motor_param_SMC[0][motor_nbr] ;
	SMC_vel.ep = motor_param_SMC[1][motor_nbr] ;
	SMC_vel.delta = motor_param_SMC[2][motor_nbr] ;
	SMC_vel.g = motor_param_SMC[3][motor_nbr] ; // convergence rate of disturbance estimation error
	SMC_vel.neta = motor_param_SMC[4][motor_nbr] ;
	SMC_vel.kp = motor_param_SMC[6][motor_nbr] ;
	SMC_vel.ki = motor_param_SMC[7][motor_nbr] ;

	fusion_vel.limit = motor_param_SMC[5][motor_nbr];
	fusion_vel.k = motor_param_SMC[0][motor_nbr] ;
	fusion_vel.ep = motor_param_SMC[1][motor_nbr] ;
	fusion_vel.delta = motor_param_SMC[2][motor_nbr] ;
	fusion_vel.g = motor_param_SMC[3][motor_nbr] ; // convergence rate of disturbance estimation error
	fusion_vel.neta = motor_param_SMC[4][motor_nbr] ;
	fusion_vel.P = motor_param[7][motor_nbr];
	fusion_vel.I = motor_param[8][motor_nbr];
	fusion_vel.D = motor_param[9][motor_nbr];
	fusion_vel.anti_windup = motor_param_SMC[5][motor_nbr];

//	LPF_a_est.y_prev = 0.0;
//	LPF_a_est.Tf = 0.001;
	LPF_w_est.y_prev = 0.0;
	LPF_w_est.Tf = 0.02;
	LPF_r_est.y_prev = 0.0;
	LPF_r_est.Tf = 0.02;

//	LPF_current_a.y_prev = 0.0;
//	LPF_current_a.Tf = 0.01;
//	LPF_current_c.y_prev = 0.0;
//	LPF_current_c.Tf = 0.01;
	LPF_current_D.y_prev = 0.0;
	LPF_current_D.Tf = 0.001;
	current_D.P = motor_param[1][motor_nbr];
	current_D.I = motor_param[2][motor_nbr];
	current_D.D = motor_param[3][motor_nbr];
	current_D.output_ramp = 100000.0;
	current_D.anti_windup = 3.0;
	current_D.limit = voltage_limit;
////
	LPF_current_Q.y_prev = 0.0;
	LPF_current_Q.Tf = 0.001;
	current_Q.P = motor_param[4][motor_nbr];
	current_Q.I = motor_param[5][motor_nbr];
	current_Q.D = motor_param[6][motor_nbr];
	current_Q.output_ramp = 100000.0;
	current_Q.anti_windup = voltage_limit;
	current_Q.limit = voltage_limit;
////
	LPF_velocity.y_prev = 0.0;
	LPF_velocity.Tf = 0.02;
	velocity.P = motor_param[7][motor_nbr];
	velocity.I = motor_param[8][motor_nbr];
	velocity.D = motor_param[9][motor_nbr];
	velocity.output_ramp = 10000;
	velocity.anti_windup = current_limit;
	velocity.limit = current_limit;
////
	LPF_angle.y_prev = 0.0;
	LPF_angle.Tf = 0.001;
	angle.P = motor_param[10][motor_nbr];
	angle.I = motor_param[11][motor_nbr];
	angle.D = motor_param[12][motor_nbr];
	angle.output_ramp = 10000;
	angle.anti_windup = motor_param[13][motor_nbr];
	angle.limit = motor_param[13][motor_nbr];

	LPF_acceleration.y_prev = 0.0 ;
	LPF_acceleration.Tf = 0.005 ;
////
////	haptic.P = 1.0;
////	haptic.I = 0.0;
////	haptic.D = 0.0;
////	haptic.output_ramp = 0;
////	haptic.limit = velocity_limit_output;
////	passivity_gain = 0.0f;
////
}
// zero_electric_offset , _sensor_direction : from Run code "find_sensor_offset_and_direction"
// sensor : Encoder , Hall sensor , Magnetic encoder
int initFOC(float zero_electric_offset, enum Direction _sensor_direction) {
	int exit_flag = 1;
// align motor if necessary
// alignment necessary for encoders.
	if (_isset(zero_electric_offset))

	{
		// absolute zero offset provided - no need to align
		zero_electric_angle = zero_electric_offset;
		// set the sensor direction - default CW
		sensor_direction = _sensor_direction;
	}
// sensor and motor alignment - can be skipped
// by setting motor.sensor_direction and motor.zero_electric_angle
	exit_flag *= alignSensor();
// added the shaft_angle update
	shaft_angle = getAngle();
	HAL_Delay(500);

	return exit_flag;
}

float calSwitchThreshold(float setpoint, float feedback)
{
	if((setpoint > -0.1)&&(setpoint <= 0.1))
		return 0.3 * fabs(feedback) ;
	else
		return 0.7 * fabs(setpoint) ;
}
void loopFOC() {
	updateSensor();
// shaft angle/velocity need the update() to be called first
// get shaft angle
	shaft_angle = shaftAngle();
// electrical angle - need shaftAngle to be called first
	electrical_angle = electricalAngle();

// Chosen TorqueControlType::foc_current
// read dq currents
	current = getFOCCurrents(electrical_angle);
	LPF_current_Q.x = current.q;
	LPF_current_Q = LowPassFilter(LPF_current_Q);
	current.q = LPF_current_Q.y_prev;   // filter values

	LPF_current_D.x = current.d;
	LPF_current_D = LowPassFilter(LPF_current_D);
	current.d = LPF_current_D.y_prev;   // filter values

// calculate the phase voltages
	current_Q.error = current_sp - current.q;
	current_Q = PID(current_Q);
	voltage.q = current_Q.output_prev;

	current_D.error = 0.0 - current.d;
	current_D = PID(current_D);
	voltage.d = current_D.output_prev;
	electrical_angle = electricalAngle();
// set the phase voltage - FOC heart function :)
	setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
//	setPhaseVoltage(current_sp, 0.0, electrical_angle);

}

//Low-Pass Filter
struct LPF LowPassFilter(struct LPF LPF) {
	unsigned long long timestamp = micros();

	float dt = (timestamp - LPF.timestamp_prev) * 1e-6f;

	if (dt <= 0 || dt > 0.0008f)
		dt = 0.0008f;
//    if (dt < 0.0f )
//        dt = 0.00016f;
//    else if(dt > 0.00016f)
//    {
//    	LPF.y_prev = LPF.x;
//    	LPF.timestamp_prev = timestamp;
//        return LPF;
//    }
	float alpha = LPF.Tf / (LPF.Tf + dt);
	float y = alpha * LPF.y_prev + (1.0f - alpha) * LPF.x;

	LPF.y_prev = y;
	LPF.timestamp_prev = timestamp;
	LPF.debug_dt = alpha;

	return LPF;
}

//float PID(float error,float P, float I, float D, float output_ramp, float limit, unsigned long timestamp_prev, float integral_prev, float error_prev , float output_prev)
struct PID PID(struct PID PID) {

// calculate the time from the last call
	unsigned long long timestamp_now = micros();
	float Ts = (timestamp_now - PID.timestamp_prev) * 1e-6;
//fix for micros overflow
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;
	dtx = Ts;
// u(s) = (P + I/s + Ds)e(s)
// Discrete implementations
// proportional part
// u_p  = P *e(k)
	float proportional = PID.P * PID.error;
// Tustin transform of the integral part
// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float integral = PID.integral_prev + PID.I * Ts * 0.5f * (PID.error + PID.error_prev);
	if ((state_enable == 0) || (integral > 23.5) || (integral < -23.5)) {
		integral = 0.0f;
	}
// antiwindup - limit the output
	integral = _constrain(integral, -PID.anti_windup, PID.anti_windup);
// Discrete derivation
// u_dk = D(ek - ek_1)/Ts
	float derivative = PID.D * (PID.error - PID.error_prev) / Ts;

// sum all the components
	float output = proportional + integral + derivative;
// antiwindup - limit the output variable
	output = _constrain(output, -PID.limit, PID.limit);

// saving for the next pass
	PID.integral_prev = integral;
	PID.output_prev = output;
	PID.error_prev = PID.error;
	PID.timestamp_prev = timestamp_now;
	PID.dtx = dtx;
	return PID;
}

struct PID_current_D PID_current_D(struct PID_current_D PID_current_D) {

// calculate the time from the last call
	unsigned long long timestamp_now = micros();
	float Ts = (timestamp_now - PID_current_D.timestamp_prev) * 1e-6;
//fix for micros overflow
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;
	dtx = Ts;
// u(s) = (P + I/s + Ds)e(s)
// Discrete implementations
// proportional part
// u_p  = P *e(k)
	float proportional = PID_current_D.P * PID_current_D.error;
// Tustin transform of the integral part
// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float integral = PID_current_D.I * Ts * 0.5f * (PID_current_D.error + PID_current_D.error_prev);
	if ((state_enable == 0) || (integral > 23.5) || (integral < -23.5)) {
		integral = 0.0f;
	}
// antiwindup - limit the output
	integral = _constrain(integral, -PID_current_D.anti_windup, PID_current_D.anti_windup);
// Discrete derivation
// u_dk = D(ek - ek_1)/Ts
	float derivative = PID_current_D.D * (PID_current_D.error - PID_current_D.error_prev) / Ts;

// sum all the components
	float output = proportional + integral + derivative;
// antiwindup - limit the output variable
	output = _constrain(output, -PID_current_D.limit, PID_current_D.limit);

// saving for the next pass
	PID_current_D.integral_prev = integral;
	PID_current_D.output_prev = output;
	PID_current_D.error_prev = PID_current_D.error;
	PID_current_D.timestamp_prev = timestamp_now;
	PID_current_D.dtx = dtx;
	return PID_current_D;
}

struct SMC SMC_V(struct SMC SMC) {
	unsigned long timestamp_now = micros();
	float Ts = (timestamp_now - SMC.timestamp_prev) * 1e-6;
	//fix for micros overflow
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;
	dtx = Ts;


	//Dynamic equations
	float an = (3 * (pole_pairs * pole_pairs) * flux_linkage) / (2 * inertia);
//	float bn = pole_pairs / inertia ;
	float cn = damping / inertia;

	// Select sliding surface
	// S = e_dot + e
//	float sliding_surface = ((SMC.error - SMC.error_prev)* Ts) + SMC.error ;
//	float sliding_surface_est = ((SMC.error_est - SMC.error_est_prev)* Ts) + SMC.error_est ;
	// S = e + e_integral
//	float integral_error = SMC.integral_error_prev + Ts * 0.5f * (SMC.error + SMC.error_prev);
//	float integral_error_est = SMC.integral_error_est_prev + Ts * 0.5f * (SMC.error_est + SMC.error_est_prev);
//	float sliding_surface = SMC.error + integral_error;
//	float sliding_surface_est = SMC.error_est + integral_error_est;
	// S = e
	float sliding_surface = SMC.error ;
	float sliding_surface_est = SMC.error_est ;

	//ESDMO
	//input_SM_observer = neta * sign(error_estimated)
	float u_smo = SMC.neta * _sign(sliding_surface_est);
	// Dif velocity estimated
	float dw_est = (an * current.q) - (cn * SMC.w_est_prev) + (SMC.r_est_prev) + u_smo;
	// Dif error estimated
	float dr_est = SMC.g * u_smo;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float w_est = SMC.w_est_prev + Ts * 0.5f * (dw_est + SMC.dw_est_prev);
	float r_est = SMC.r_est_prev + Ts * 0.5f * (dr_est + SMC.dr_est_prev);
	LPF_w_est.x = w_est;
	LPF_w_est = LowPassFilter(LPF_w_est);
	w_est = LPF_w_est.y_prev;  // filter values
	LPF_r_est.x = r_est;
	LPF_r_est = LowPassFilter(LPF_r_est);
	r_est = LPF_r_est.y_prev;  // filter values

//	if(shaft_angle_sp != prev_shaft_angle_sp)
//	{
//		switch_threshold = calSwitchThreshold(shaft_angle_sp, shaft_angle)  ;
//		if(fabs(shaft_angle_sp) >= fabs(prev_shaft_angle_sp))
//			dir_setpoint = 0 ;
//		else if(fabs(shaft_angle_sp) < fabs(prev_shaft_angle_sp))
//			dir_setpoint = 1 ;
//		prev_shaft_angle_sp = shaft_angle_sp ;
//	}
//
//	float sign_temp = _sign(sliding_surface) ;
//	float integral_sign = SMC.integral_sign_prev + Ts * 0.5f * (sign_temp + SMC.sign_temp_prev) ;
//
//	if(((fabs(shaft_angle) <= switch_threshold)&&(dir_setpoint == 1))||(((fabs(shaft_angle) >= switch_threshold)&&(dir_setpoint == 0))))
//	{
//		//Super twisted method
//		// Function to perform the integration of signum(s) over time
//		float eq_st = (SMC.kp * _sqrtApprox(fabs(sliding_surface)) * _sign(sliding_surface)) + (SMC.ki * integral_sign);
//		SMC.debug_eq = eq_st ;
//		datacheck = 1 ;
//	}
//	else
//	{
//		//Novel Reaching Law method
//		float eq_nr = (SMC.k / (SMC.ep + (1 + 1 / (fabs(SMC.error)) - SMC.ep) * expf(-SMC.delta * fabs(sliding_surface)))) * _sign(sliding_surface);
//		integral_sign = 0.0f  ;
//		SMC.debug_eq = eq_nr ;
//		datacheck = 0 ;
//	}

////fusion method
	float sign_temp = _sign(sliding_surface) ;
	float integral_sign = SMC.integral_sign_prev + Ts * 0.5f * (sign_temp + SMC.sign_temp_prev) ;
	SMC.debug_eq = (SMC.kp * _sqrtApprox(fabs(sliding_surface)) * _sign(sliding_surface)) + (SMC.ki * integral_sign* expf(-SMC.delta * fabs(sliding_surface)));
	float output = (1 / an) * ((cn * shaft_velocity) - r_est + SMC.debug_eq);
	output = _constrain(output, -SMC.limit, SMC.limit);
	SMC.debug_output = output ;
	SMC.error_prev = SMC.error ;
	SMC.error_est_prev = SMC.error_est ;
//	SMC.integral_error_prev = integral_error;
//	SMC.integral_error_est_prev = integral_error_est;
	SMC.sign_temp_prev = sign_temp ;
	SMC.integral_sign_prev = integral_sign ;
	SMC.output_prev = output ;
	SMC.r_est_prev = r_est ;
	SMC.dr_est_prev = dr_est ;
	SMC.w_est_prev = w_est ;
	SMC.dw_est_prev = dw_est ;
	SMC.timestamp_prev = timestamp_now;
	SMC.dtx = dtx ;
	return SMC;
}

void SMC_velocity(float new_target) {

	shaft_velocity = shaftVelocity();
	electrical_angle = electricalAngle();
	current = getFOCCurrents(electrical_angle);

	if (_isset(new_target))
		target = new_target;

	// velocity set point
	shaft_velocity_sp = target;
	SMC_vel.error = shaft_velocity_sp - shaft_velocity;
	SMC_vel.error_est = SMC_vel.w_est_prev - shaft_velocity;
	SMC_vel = SMC_V(SMC_vel);
	current_sp = SMC_vel.output_prev;
}

struct SMC SMC_A(struct SMC SMC) {
	unsigned long timestamp_now = micros();
	float Ts = (timestamp_now - SMC.timestamp_prev) * 1e-6;
	//fix for micros overflow
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;

	//Dynamic equations
	float an = (3.0f * (pole_pairs * pole_pairs) * flux_linkage * Ts) / (4.0f * inertia);
//	float bn = (pole_pairs * Ts) / inertia ;
	float cn = (pole_pairs * damping) / inertia;
	//ESDMO
	//input_SM_observer = neta * sign(error_estimated)
	float u_smo = SMC.neta * _sign(SMC.error_est);
	// Dif angle estimated
	float da_est = (an * current.q) + (an * SMC.current_q_prev)
			- (cn * SMC.a_est_prev) + (SMC.r_est_prev) + u_smo;
	// Dif error estimated
	float dr_est = SMC.g * u_smo;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)

	float a_est = SMC.a_est_prev + Ts * 0.5f * (da_est + SMC.da_est_prev);
	float r_est = SMC.r_est_prev + Ts * 0.5f * (dr_est + SMC.dr_est_prev);

	//SMC
	float eq_s = SMC.k / (SMC.ep + (1.0f + 1.0f / (fabs(SMC.error)) - SMC.ep) * expf(-SMC.delta * fabs(SMC.error)));
	float output = (an * (current.q + SMC.current_q_prev)) - (cn * shaft_angle) - r_est + (eq_s * _sign(SMC.error));
//	float output = (1 / an) * ( - (an * SMC.current_q_prev) + (cn * shaft_angle)
//				 - r_est + (eq_s * _sign(SMC.error)));
	SMC.debug_eq = eq_s;
	SMC.debug_output = eq_s * _sign(SMC.error)  ;

	output = _constrain(output, -SMC.limit, SMC.limit);

	LPF_SMC_out.x = output;
	LPF_SMC_out = LowPassFilter(LPF_SMC_out);
	output = LPF_SMC_out.y_prev;  // filter values

	SMC.current_q_prev = current.q ;
	SMC.eq_s_prev = eq_s ;
	SMC.output_prev = output ;
	SMC.r_est_prev = r_est ;
	SMC.dr_est_prev = dr_est ;
	SMC.a_est_prev = a_est ;
	SMC.da_est_prev = da_est ;
	SMC.timestamp_prev = timestamp_now;
	SMC.dtx = Ts ;
	return SMC;
}

void SMC_angle(float new_target) {

	shaft_angle = shaftAngle();
	shaft_velocity = shaftVelocity();
	electrical_angle = electricalAngle();
	current = getFOCCurrents(electrical_angle);

	//PID - SMC -----------------------------------------------------------------------
	if (_isset(new_target))
		target = new_target;

// angle set point
	shaft_angle_sp = target;
// calculate velocity set point
	angle.error = shaft_angle_sp - shaft_angle;
	angle = PID(angle);

	shaft_angle = shaftAngle();
	shaft_velocity = shaftVelocity();
	electrical_angle = electricalAngle();
	current = getFOCCurrents(electrical_angle);

	SMC_vel.error = angle.output_prev - shaft_velocity;
	SMC_vel.error_est = SMC_vel.w_est_prev - shaft_velocity;
	SMC_vel = SMC_V(SMC_vel);
	current_sp = SMC_vel.output_prev;
}

struct fusion fusion_V(struct fusion fusion)
{
	unsigned long timestamp_now = micros();
	float Ts = (timestamp_now - fusion.timestamp_prev) * 1e-6;
	//fix for micros overflow
	if (Ts <= 0 || Ts > 0.0008f)
		Ts = 0.0008f;
	dtx = Ts;
	//Calculate SMC-----------------------------------------------------------------------------------------------------------
	//Dynamic equations
	float an = (3 * (pole_pairs * pole_pairs) * flux_linkage) / (2 * inertia);
//	float bn = pole_pairs / inertia ;
	float cn = damping / inertia;

	//Calculate PID-----------------------------------------------------------------------------------------------------------

	//output mode---------------------------------------------------------------------------------------------------------------

	if ((1.5 * (fabs(shaft_angle)) < (fabs(Command_setpoint)))) {
		float sliding_surface = fusion.error ;
		float sliding_surface_est = fusion.error_est ;
		//ESDMO
		//input_SM_observer = neta * sign(error_estimated)
		float u_smo = fusion.neta * _sign(sliding_surface_est);
		// Dif velocity estimated
		float dw_est = (an * current.q) - (cn * fusion.w_est_prev) + (fusion.r_est_prev) + u_smo;
		// Dif error estimated
		float dr_est = fusion.g * u_smo;
		// Tustin transform of the integral part
		// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
		float w_est = fusion.w_est_prev + Ts * 0.5f * (dw_est + fusion.dw_est_prev);
		float r_est = fusion.r_est_prev + Ts * 0.5f * (dr_est + fusion.dr_est_prev);

		LPF_w_est.x = w_est;
		LPF_w_est = LowPassFilter(LPF_w_est);
		w_est = LPF_w_est.y_prev;  // filter values
		LPF_r_est.x = r_est;
		LPF_r_est = LowPassFilter(LPF_r_est);
		r_est = LPF_r_est.y_prev;  // filter values

		float eq_s = fusion.k/ (fusion.ep + (1 + 1 / (fabs(fusion.error)) - fusion.ep)* expf(-fusion.delta * fabs(sliding_surface)));
		float output_SMC = (1 / an)* ((cn * shaft_velocity) - r_est+ (eq_s * _sign(sliding_surface)));
		LPF_SMC_out.x = output_SMC;
		LPF_SMC_out = LowPassFilter(LPF_SMC_out);
		output_SMC = LPF_SMC_out.y_prev;
		output_SMC = _constrain(output_SMC, -fusion.limit, fusion.limit);
		fusion.output_prev = output_SMC;
		fusion.r_est_prev = r_est ;
		fusion.dr_est_prev = dr_est ;
		fusion.w_est_prev = w_est ;
		fusion.dw_est_prev = dw_est ;
	}
	else {
		float proportional = fusion.P * fusion.error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
		float integral = fusion.integral_prev
				+ fusion.I * Ts * 0.5f * (fusion.error + fusion.error_prev);
		if ((state_enable == 0) || (integral > 23.5) || (integral < -23.5)) {
			integral = 0.0f;
		}
	// antiwindup - limit the output
		integral = _constrain(integral, -fusion.anti_windup, fusion.anti_windup);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
		float derivative = fusion.D * (fusion.error - fusion.error_prev) / Ts;
		float output_PID = proportional + integral + derivative;
	// antiwindup - limit the output variable
		output_PID = _constrain(output_PID, -fusion.limit, fusion.limit);
		fusion.output_prev = output_PID;
		fusion.error_prev = fusion.error ;
		fusion.integral_prev = integral;
	}

	fusion.error_est_prev = fusion.error_est ;

	fusion.timestamp_prev = timestamp_now;
	fusion.dtx = dtx ;
	return fusion;
}

void fusion_angle(float new_target) {

	shaft_angle = shaftAngle();
	shaft_velocity = shaftVelocity();
	electrical_angle = electricalAngle();
	current = getFOCCurrents(electrical_angle);

	//PID - SMC -----------------------------------------------------------------------
	if (_isset(new_target))
		target = new_target;

// angle set point
	shaft_angle_sp = target;
// calculate velocity set point
	angle.error = shaft_angle_sp - shaft_angle;
	angle = PID(angle);

	shaft_angle = shaftAngle();
	shaft_velocity = shaftVelocity();
	electrical_angle = electricalAngle();
	current = getFOCCurrents(electrical_angle);

	fusion_vel.error = angle.output_prev - shaft_velocity;
	fusion_vel.error_est = fusion_vel.w_est_prev - shaft_velocity;
	fusion_vel = fusion_V(fusion_vel);
	current_sp = fusion_vel.output_prev;
}
// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void move_velocity(float new_target) {
// get angular velocity
	shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

	if (_isset(new_target))
		target = new_target;

// velocity set point
	shaft_velocity_sp = target;

// calculate the torque command
	velocity.error = shaft_velocity_sp - shaft_velocity;
	velocity = PID(velocity);
	current_sp = velocity.output_prev;
// if current/foc_current torque control
// if torque controlled through voltage control
//  voltage.q = current_sp*phase_resistance;
//  voltage.d = 0;

}

void move_angle(float new_target) {
// get angular velocity
	shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

// downsampling (optional)
// if(motion_cnt++ < motion_downsample) return;
// motion_cnt = 0;
// set internal target variable
	if (_isset(new_target))
		target = new_target;

// angle set point
	shaft_angle_sp = target;
// calculate velocity set point
	angle.error = shaft_angle_sp - shaft_angle;
	angle = PID(angle);
	shaft_velocity_sp = angle.output_prev;
// calculate the torque command
	velocity.error = shaft_velocity_sp - shaft_velocity;
	velocity = PID(velocity);
	current_sp = velocity.output_prev;

//  voltage.q = current_sp*phase_resistance;
//  voltage.d = 0;
}

void move_haptic(float new_target, float passivity_gain) {
// get angular velocity
	shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

// downsampling (optional)
// if(motion_cnt++ < motion_downsample) return;
// motion_cnt = 0;
// set internal target variable
	if (_isset(new_target))
		target = new_target;
// angle set point
	shaft_angle_sp = target;
// calculate velocity set point
	haptic.error = shaft_angle_sp - shaft_angle;
	haptic = PID(haptic);
	current_sp = haptic.output_prev + (shaft_velocity * passivity_gain);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float velocityOpenloop(float target_velocity) {
// get current timestamp
	unsigned long now_us = micros();
// calculate the sample time from last call
	float Ts = (now_us - open_loop_timestamp) * 1e-6;
// quick fix for strange cases (micros overflow + timestamp not defined)
	if (Ts <= 0 || Ts > 0.5)
		Ts = 1e-3;

// calculate the necessary angle to achieve target velocity
	shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
// for display purposes
	shaft_velocity = target_velocity;

// use voltage limit or current limit
	float Uq = voltage_limit;  //24V
//  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance;
//  voltage_sensor_align
// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

// save timestamp for next call
	open_loop_timestamp = now_us;

	return Uq;
}

void move_velocity_openloop(float target) {
	shaft_velocity_sp = target;
	voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
	voltage.d = 0;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float angleOpenloop(float target_angle) {
	unsigned long now_us = micros();
// calculate the sample time from last call
	float Ts = (now_us - open_loop_timestamp) * 1e-6;
// quick fix for strange cases (micros overflow + timestamp not defined)
	if (Ts <= 0 || Ts > 0.5)
		Ts = 1e-3;

// calculate the necessary angle to move from current position towards target angle
// with maximal velocity (velocity_limit)
	if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts)) {
		shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit)
				* Ts;
		shaft_velocity = velocity_limit;
	} else {
		shaft_angle = target_angle;
		shaft_velocity = 0;
	}

// use voltage limit or current limit
	float Uq = voltage_limit;
//  if(_isset(phase_resistance))
//	  Uq =  current_limit*phase_resistance;

// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

	open_loop_timestamp = now_us;
	return Uq;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

