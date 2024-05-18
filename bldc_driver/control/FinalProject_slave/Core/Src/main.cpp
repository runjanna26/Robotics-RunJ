/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "math.h"
#include "string.h"
#include "HelloRunJ.h"

HelloRunJ runx;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Define for calculation
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
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) ) // Check it later
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
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

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
long angle_prev_ts = 0; // timestamp of last call to getAngle, used for velocity
float vel_angle_prev = 0; // angle at last call to getVelocity, used for velocity
long vel_angle_prev_ts = 0; 	// last velocity calculation timestamp
int32_t full_rotations = 0; 	// full rotation tracking
int32_t vel_full_rotations = 0; // previous full rotation value for velocity calculation
float getVelocity;

//===Current Sensor===
struct DQCurrent_s // dq current structure
{
	float d;
	float q;
};
struct PhaseCurrent_s // phase current structure
{
	float a;
	float b;
	float c;
};
struct DQVoltage_s // d-q voltage struct
{
	float d;
	float q;
};
double offset_ia; //!< zero current A voltage value (center of the adc reading)
double offset_ib; //!< zero current B voltage value (center of the adc reading)
double offset_ic; //!< zero current C voltage value (center of the adc reading)
float gain_a;     //!< phase A gain
float gain_b;     //!< phase B gain
float gain_c;     //!< phase C gain
float R_sense;
#define CurrentSense_resistance 0.1
#define CurrentSense_gain 10
//ADC DMA variable
uint32_t adcResultDMA[3];  // to store the ADC value
const int adcChannelCount = sizeof(adcResultDMA) / sizeof(adcResultDMA[0]);
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
struct DQVoltage_s voltage;      //!< current d and q voltage set to the motor
struct DQCurrent_s current;      //!< current d and q current measure

//Low pass filter
struct LPF // LPF struct
{
	float x;							//!< (INPUT)
	unsigned long timestamp_prev;  	//!< Last execution timestamp
	float y_prev; 		//!< filtered value in previous execution step (OUTPUT)
	float Tf;
} LPF_angle_s, LPF_velocity_s, LPF_current_q_s, LPF_current_d_s;

//PID
struct PID {
	float error;					//!< INPUT
	float integral_prev; 			//!< last integral component value
	float error_prev; 				//!< last tracking error value
	unsigned long timestamp_prev; 	//!< Last execution timestamp
	float output_prev;  			//!< last pid output value (OUTPUT)
	float output_ramp; 			//!< Maximum speed of change of the output value
	float limit; 					//!< Maximum output value
	float P; 						//!< Proportional gain
	float I; 						//!< Integral gain
	float D; 						//!< Derivative gain
} PID_current_d_s, PID_current_q_s, PID_velocity_s, PID_angle_s,PID_haptic_s;
float passivity_gain ;

// ======motor physical parameters (SET HERE !!!)======
// motor configuration parameters
float voltage_sensor_align;		//!< sensor and motor align voltage parameter
float velocity_index_search;	//!< target velocity for index search

float phase_resistance = 7.1; 	//!< motor phase resistance
int pole_pairs = 14;			//!< motor pole pairs number
float voltage_power_supply;

// limiting variables
float voltage_limit; 			//!< Voltage limitting variable - global limit
float current_limit; 			//!< Current limitting variable - global limit
float velocity_limit; 			//!< Velocity limitting variable - global limit

//Position sensor variable
float sensor_offset = 0; //!< user defined sensor zero offset
float zero_electric_angle = NOT_SET; //!< absolute zero electric angle - if available
int sensor_direction = NOT_SET; //!< if sensor_direction == Direction::CCW then direction will be flipped to CW
enum Direction {
	CW = 1,  //clockwise
	CCW = -1, // counter clockwise
	UNKNOWN = 0   //not yet known or invalid state
};

long open_loop_timestamp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

__STATIC_INLINE void DWT_Init(void); 						//checked
__STATIC_INLINE void delay_us(uint32_t us);				//dont use it
__STATIC_INLINE uint32_t micros(void);						//checked

//CAN function
void CAN_init_446();
void CAN_init_103();
void float2Bytes(float val, uint8_t *bytes_array);  		//checked
float Bytes2float(uint8_t *bytes_array);					//checked

//ADC Read
uint16_t getValueCH3();	//SOA								//dont use it
uint16_t getValueCH8();	//SOB								//dont use it
uint16_t getValueCH9();	//SOC								//dont use it

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

void Sensor_init();
void updateSensor();												//checked
float getMechanicalAngle();											//checked
float getAngle();													//checked
int32_t getFullRotations();												//

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs);
float electricalAngle();
// shaft angle calculation
float shaftAngle();
// shaft velocity calculation
float shaftVelocity();

//PWM function
void writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c);  //checked
void setPhaseVoltage(float Uq, float Ud, float angle_el);   //checked

//Motion control function
int needsSearch();											//checked
int absoluteZeroSearch();									//don't use
int alignSensor();											//checked
int initFOC(float zero_electric_offset, enum Direction _sensor_direction); //checked
void loopFOC();												//

struct LPF LowPassFilter(struct LPF LPF); 					//checked
struct PID PID(struct PID PID);								//checked

//Closed Loop
void move_velocity(float new_target);		//closed loop			//checked
void move_angle(float new_target);	 //closed loop			//checked
void move_haptic(float new_target, float passivity_gain);
void move_torque(float new_target);  //closed loop			//checked
//Open Loop
float velocityOpenloop(float target_velocity);		//checked 50 rpm so smooth
void move_velocity_openloop(float target);			//checked 50 rpm so smooth
float angleOpenloop(float target_angle);					//

//Serial Write
int _write(int file, char *ptr, int len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

int datacheck = 0;
int do_onetime = 0;
int do_sethome = 0;
int count = 0;

int num = 0;

float float_example = 0.1023445;
uint8_t bytes[4];
float float_final = 0.0f;
uint32_t CAN_error;
int count1, count2;

//CAN:103
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &RxHeader, RxData) != HAL_OK)
	{
		CAN_error = HAL_CAN_GetError(hcan);
		Error_Handler();
	}
	datacheck = 1;
	if ((RxHeader.StdId == 0x446) && (RxHeader.DLC == 4))
	{
		bytes[0] = RxData[0];
		bytes[1] = RxData[1];
		bytes[2] = RxData[2];
		bytes[3] = RxData[3];
		float_final = Bytes2float(bytes);
	}
	count2++;
}

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


float pp_check;

unsigned long t1 = 0, ts, t2 = 0, t3, t4 = 0, t5 = 0;
uint8_t MSG[35] = { '\0' };
float Command_setpoint = 30.0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	runx.setSomeVariable(10); // Use setter method to set the value

	// Set a breakpoint here to check the value
	int value = runx.getSomeVariable();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_CAN_Init();
	/* USER CODE BEGIN 2 */

	//Delay SETUP
	DWT_Init();

	//Timer Interrupt tim2,tim4

	HAL_TIM_Base_Start_IT(&htim4);

	//Driver SETUP
	voltage_sensor_align = 3; // aligning voltage [V]
	velocity_index_search = 3; // index search velocity [rad/s]

	voltage_power_supply = 24.0;

	voltage_limit = 24.0;
	current_limit = 20.0;		// current_sp maximum
	velocity_limit = 20.0;       // maximal velocity of the position control

	//Control system configuration

//====Motor====
	LPF_current_d_s.y_prev = 0.0;
	LPF_current_d_s.Tf = 0.001;
	PID_current_d_s.P = 1.0;  //1.0
	PID_current_d_s.I = 0.0; //713.0
	PID_current_d_s.D = 0.0;
	PID_current_d_s.output_ramp = 1000.0;
	PID_current_d_s.limit = voltage_limit;

	LPF_current_q_s.y_prev = 0.0;
	LPF_current_q_s.Tf = 0.001;
	PID_current_q_s.P = 1.0;  // 1.0
	PID_current_q_s.I = 0.0; //10.0
	PID_current_q_s.D = 0.0;
	PID_current_q_s.output_ramp = 1000.0;
	PID_current_q_s.limit = voltage_limit;

	LPF_velocity_s.y_prev = 0.0;
	LPF_velocity_s.Tf = 0.01;
	PID_velocity_s.P = 0.5;   // P > 0.5 oscillate
	PID_velocity_s.I = 55.0;  // I > 55 under-overshoot
	PID_velocity_s.D = 0.0;
	PID_velocity_s.output_ramp = 1000;
	PID_velocity_s.limit = current_limit;  // current_limit

	LPF_angle_s.y_prev = 0.0;
	LPF_angle_s.Tf = 0.01;
	PID_angle_s.P = 50.0;
	PID_angle_s.I = 0.0;
	PID_angle_s.D = 0.2;
	PID_angle_s.output_ramp = 0;
	PID_angle_s.limit = velocity_limit;

	PID_haptic_s.P = 40.0;
	PID_haptic_s.I = 0.1;
	PID_haptic_s.D = 0.4;
	PID_haptic_s.output_ramp = 0;
	PID_haptic_s.limit = velocity_limit;
	passivity_gain = 0.2f;

	//SPI SETUP
	MagneticSensorSPI_init(AS5048A_SPI);
	//POSITION SENSOR SETUP
	Sensor_init();

	//CURRENT SENSE SETUP
	HAL_ADC_Start_DMA(&hadc1, adcResultDMA, 3);
	initCurrentsense(CurrentSense_resistance, CurrentSense_gain);
//  calibrateOffsets();

	//PWM SETUP
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);    //pinMode
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	//pinMode
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	//pinMode

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);  // Enable
//	  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);  // Disable

	//FOC SETUP
	zero_electric_angle = 0.397302926;
	initFOC(zero_electric_angle, CW); //M1 3.85949397
	//1.98957574+1.98957574+1.98420465
//	initFOC(zero_electric_angle, UNKNOWN); //Not yet calibrate find the best init value

	//CAN SETUP ID: 0x103
	CAN_init_103();
	t1 = micros();
	t2 = micros();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//Drive PWM
//	  writeDutyCycle3PWM(0.2, 0.5, 0.8);
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	HAL_Delay(1000);

		//Sine PWM Testing  Checked
//	  currentStepA = currentStepA + 1;  //Add 1 to make the motor move step by step.
//	  currentStepB = currentStepA + 16; //add 120 deg of phase to whatever position StepA is.
//	  currentStepC = currentStepA + 32; //add 240 deg of phase to whatever position StepA is.
//	  currentStepA = currentStepA % 48; //I used remainder operation or modulo to "wrap" the values between 0 and 47
//	  currentStepB = currentStepB % 48;
//	  currentStepC = currentStepC % 48;
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmSin[currentStepA]);
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwmSin[currentStepB]);
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwmSin[currentStepC]);
//	  HAL_Delay(2);

		//ADC multi-channel Testing Checked // 686us
//	  sprintf(MSG, "CH1=%d \t CH2=%d \t CH3=%d \r\n",adcResultDMA[0],adcResultDMA[1],adcResultDMA[2] );
//	  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100); 			// 259 us

		//current test
		//Phase A and B to Load
//	  sensortest = getPhaseCurrents();
//	  LPF_current_q_s.x = sensortest.b;
//	  LPF_current_q_s = LowPassFilter(LPF_current_q_s);
//	  sprintf(MSG, "%.3f,%.3f,%.3f \n",sensortest.a,sensortest.b,sensortest.c);
//	  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);

		//Position sensor testing
	    updateSensor();
//	    getvelocity;
//		electrical_angle = getAngle();
//		shaft_angle = shaftAngle();
//		shaft_velocity = shaftVelocity();
//		sprintf(MSG, "%.3f \n",shaft_velocity );
//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);



//=================Open-loop testing=================
//		move_velocity_openloop(30.0);
//		angleOpenloop(_PI_2);
//		HAL_Delay(3);

//  	  ts = micros()-t1;
//  	  t1 = micros();
//	  	  t3++;

//=================Closed-loop testing=================
		//Speed control
//	  	if(micros() - t1 >= 3000000)
//	  	{
//	  		t1 = micros();
//
//	  		Command_setpoint = Command_setpoint + 3;
//	  		if(Command_setpoint > 12)
//	  			Command_setpoint = 0;
//	  	}
//		if (float_final <= 0.01) float_final = 0.0f ;
//		move_velocity(float_final); 									//136us --> 72us
//		loopFOC();  													//1190us --> 485us

//	  sprintf(MSG, "%.3f,%.3f \n",float_final,shaft_velocity);  		// 134us
//	  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);			  	// 181us

//		Position control
	  	loopFOC(); 														//1190us --> 495 us
	  	move_angle(0.0f); 												//161 us --> 112 us
//		sprintf(MSG, "%.3f,%.3f \n",Command_setpoint,shaft_angle);
//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);

		//Torque control
//		move_torque(5*(float_final-shaft_angle));
//		loopFOC(); 						//1190us --> 495 us
//		sprintf(MSG, "%.3f,%.3f,%.3f \n",0.0,current.d,current.q);
//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
//		sprintf(MSG, "%.3f,%.3f,%.3f \n", 0.0, PID_current_q_s.output_prev, PID_current_d_s.output_prev);
//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//Toggle LED Test for uploading.
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(1);
		// 259 us
//	  	sprintf(MSG, " %d \n", );
//	  	HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);



		//=================CAN BUS=================
//	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0 )
//	{
//		float2Bytes(shaft_angle, &bytes[0]);
//		TxData[0] = bytes[0];
//		TxData[1] = bytes[1];
//		TxData[2] = bytes[2];
//		TxData[3] = bytes[3];
//		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)!= HAL_OK)
//		{
//			CAN_error = HAL_CAN_GetError(&hcan);
//			Error_Handler();
//		}
//		HAL_Delay(1); // important !!!
//		datacheck = 0;
//		count1++;
//	}

		//============Haptic Control============
//		if (t2 - t1 >= 3000000)
//		{
//			do_sethome = 1;
//			move_haptic(float_final,passivity_gain);
//			loopFOC(); 						//1190us --> 495 us
//		}
//		else
//		{
//			//Set Home
//			if (do_sethome == 0)
//			{
//				t2 = micros();
//				move_angle(0.0f); 	//161 us --> 112 us
//				loopFOC(); 			//1190us --> 495 us
//			}
//		}
//		t5 = micros() - t4;
//		t4 = micros();

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	//DRV8323RH Unity Gain Bandwidth = 1MHz
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 6;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim2.Init.Period = ARR_MAX_CA;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 2000000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_NSS_Pin */
	GPIO_InitStruct.Pin = SPI1_NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 EN_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Delay function
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

//Timer interrupt routine
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	//Interrupt Timer2
//	if(htim == &htim2)
//	{
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	}
//
//	//Interrupt Timer4
	if (htim == &htim4) //interrupt every 1ms
			{
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}
}

//CAN function
void CAN_init_446() {
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0; // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x103 << 5;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x103 << 5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; // doesn't matter in single can controllers

	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	TxHeader.DLC = 4;  // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x446;  // ID
}
void CAN_init_103() {
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0; // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canfilterconfig.FilterIdHigh = 0x446 << 5;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x446 << 5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 10; // doesn't matter in single can controllers
	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	TxHeader.DLC = 4;  // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x103;  // ID
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

//ADC Read
void ADC_Select_CH3(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH8(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH9(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

uint16_t getValueCH3() {
	uint16_t val;
	ADC_Select_CH3();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return val;
}

uint16_t getValueCH8() {
	uint16_t val;
	ADC_Select_CH8();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return val;
}

uint16_t getValueCH9() {
	uint16_t val;
	ADC_Select_CH9();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return val;
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
		offset_ia += adcResultDMA[0];
		offset_ib += adcResultDMA[1];
		offset_ic += adcResultDMA[2];
		HAL_Delay(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia / calibration_rounds;
	offset_ib = offset_ib / calibration_rounds;
	offset_ic = offset_ic / calibration_rounds;
}

//// read all three phase currents (if possible 2 or 3)
struct PhaseCurrent_s getPhaseCurrents() {
	struct PhaseCurrent_s current;
	current.a = ((3.3 / 2)
			- (adcResultDMA[0] - 0) * ((3.05 - 0.25) / (3785.0 - 311.0)))
			/ (R_sense * gain_a);
	current.b = ((3.3 / 2)
			- (adcResultDMA[1] - 0) * ((3.05 - 0.25) / (3785.0 - 311.0)))
			/ (R_sense * gain_b);
	current.c = ((3.3 / 2)
			- (adcResultDMA[2] - 0) * ((3.05 - 0.25) / (3785.0 - 311.0)))
			/ (R_sense * gain_c);
//    current.b = -current.a-current.c;
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
	// if only two measured currents
	i_alpha = current.a;
	i_beta = (-(_1_SQRT3) * current.a) + (-(_2_SQRT3) * current.c);
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
	float ct = _cos(angle_el);
	float st = _sin(angle_el);
	struct DQCurrent_s return_current;
	return_current.d = i_alpha * ct + i_beta * st;
	return_current.q = i_beta * ct - i_alpha * st;
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
	angle_register =
			config.angle_registers ?
					config.angle_registers : DEF_ANGLE_REGISTER;
	// register maximum value (counts per revolution)
	cpr = pow(2, config.bit_resolution);
	bit_resolution = config.bit_resolution;

	command_parity_bit = config.command_parity_bit; // for backwards compatibility
	command_rw_bit = config.command_rw_bit; // for backwards compatibility
	data_start_bit = config.data_start_bit; // for backwards compatibility

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
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
		command |=
				((uint16_t) spiCalcEvenParity(command) << command_parity_bit);
	}

	//SPI - begin transaction

	//Send the command
	//  spi->transfer16(command);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &command,
			(uint8_t*) &register_value,
			sizeof(register_value) / sizeof(uint16_t), 100);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

//  delay_us(1);

	command = 0x0000;
	//Now read the response (NO_OPERATION_COMMAND = 0x0000)
	//  uint16_t register_value = spi->transfer16(0x00);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &command,
			(uint8_t*) &register_value,
			sizeof(register_value) / sizeof(uint16_t), 100);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

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
	float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
	// quick fix for strange cases (micros overflow)
	if (Ts <= 0)
		Ts = 1e-3f;
	// velocity calculation
	float vel = ((float) (full_rotations - vel_full_rotations) * _2PI
			+ (angle_prev - vel_angle_prev)) / Ts;
	// save variables for future pass
	vel_angle_prev = angle_prev;
	vel_full_rotations = full_rotations;
	vel_angle_prev_ts = angle_prev_ts;
	return vel;
}

// shaft angle calculation
float shaftAngle() {
	LPF_angle_s.x = getAngle();
	LPF_angle_s = LowPassFilter(LPF_angle_s);
	return sensor_direction * LPF_angle_s.y_prev - sensor_offset;
}
// shaft velocity calculation
float shaftVelocity() {
	LPF_velocity_s.x = getvelocity();
	LPF_velocity_s = LowPassFilter(LPF_velocity_s);
	return sensor_direction * LPF_velocity_s.y_prev;
}

//Conversion shaft angle to elec angle
float electricalAngle() {
	return _normalizeAngle(
			(float) (sensor_direction * pole_pairs) * getMechanicalAngle()
					- zero_electric_angle);
}

//Write PWM fsw = 25kHzfloat Ts
void writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ARR_MAX_CA*dc_a);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ARR_MAX_CA*dc_b);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ARR_MAX_CA*dc_c);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
void setPhaseVoltage(float Uq, float Ud, float angle_el) {
	float Uout;
	// a bit of optitmisation
	if (Ud) {
		// only if Ud and Uq set
		// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_limit;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	} else {
		// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_limit;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	// find the sector we are in currently
	int sector = floor(angle_el / _PI_3) + 1;
	// calculate the duty cycles
	float T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
	float T2 = _SQRT3 * _sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
//  float T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2
	float T0 = 0; // pulled to 0 - better for low power supply voltage

	// calculate the duty cycles(times)
	float Ta, Tb, Tc;
	switch (sector) {
	case 1:
		Ta = T1 + T2 + T0 / 2;
		Tb = T2 + T0 / 2;
		Tc = T0 / 2;
		break;
	case 2:
		Ta = T1 + T0 / 2;
		Tb = T1 + T2 + T0 / 2;
		Tc = T0 / 2;
		break;
	case 3:
		Ta = T0 / 2;
		Tb = T1 + T2 + T0 / 2;
		Tc = T2 + T0 / 2;
		break;
	case 4:
		Ta = T0 / 2;
		Tb = T1 + T0 / 2;
		Tc = T1 + T2 + T0 / 2;
		break;
	case 5:
		Ta = T2 + T0 / 2;
		Tb = T0 / 2;
		Tc = T1 + T2 + T0 / 2;
		break;
	case 6:
		Ta = T1 + T2 + T0 / 2;
		Tb = T0 / 2;
		Tc = T1 + T0 / 2;
		break;
	default:
		// possible error state
		Ta = 0;
		Tb = 0;
		Tc = 0;
	}

	// calculate the phase voltages
	Ua = Ta * voltage_limit;
	Ub = Tb * voltage_limit;
	Uc = Tc * voltage_limit;

	// set the voltages in hardware
	// limit the voltage in driver
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
	// calculate duty cycle
	float dc_a;  //duty cycle phase A [0, 1]
	float dc_b;  //duty cycle phase B [0, 1]
	float dc_c;  //duty cycle phase C [0, 1]
	// limited in [0,1]
	dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
	dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
	dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
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
	if (!_isset(sensor_direction)) //sensor_direction == -12345.0
			{
		// check if sensor needs zero search
		if (needsSearch()) //needSearch == 0 because use Magnetic sensor
			exit_flag = absoluteZeroSearch(); // o
		if (!exit_flag)
			return exit_flag;

		// find natural direction
		// move one electrical revolution forward
		for (int i = 0; i <= 500; i++) {
			float angle = _3PI_2 + _2PI * i / 500.0;
			setPhaseVoltage(voltage_sensor_align, 0, angle);
			HAL_Delay(2);
		}
		updateSensor();
		// take and angle in the middle
		float mid_angle = getAngle();
		// move one electrical revolution backwards
		for (int i = 500; i >= 0; i--) {
			float angle = _3PI_2 + _2PI * i / 500.0;
			setPhaseVoltage(voltage_sensor_align, 0, angle);
			HAL_Delay(2);
		}
		updateSensor();
		float end_angle = getAngle();
		setPhaseVoltage(0, 0, 0);
		HAL_Delay(200);
		// determine the direction the sensor moved
		if (mid_angle == end_angle) {
			return 0; // failed calibration
		} else if (mid_angle < end_angle) {
			sensor_direction = CCW;
		} else {
			sensor_direction = CW;
		}
		// check pole pair number

		float moved = fabs(mid_angle - end_angle);
		if (fabs(moved * pole_pairs - _2PI) > 0.5) { // 0.5 is arbitrary number it can be lower or higher!
			pp_check = _2PI / moved;
		}
	}

	// zero electric angle not known
	if (!_isset(zero_electric_angle)) {
		// align the electrical phases of the motor and sensor
		// set angle -90(270 = 3PI/2) degrees
		setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);
		HAL_Delay(700);
		zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction * getAngle(), pole_pairs));
		HAL_Delay(20);
		// stop everything
		setPhaseVoltage(0, 0, 0);
		HAL_Delay(200);
	}
	return exit_flag;
}

// zero_electric_offset , _sensor_direction : from Run code "find_sensor_offset_and_direction"
// sensor : Encoder , Hall sensor , Magnetic encoder
int initFOC(float zero_electric_offset, enum Direction _sensor_direction) {
	int exit_flag = 1;
	// align motor if necessary
	// alignment necessary for encoders.
	if (_isset(zero_electric_offset)) {
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
	LPF_current_q_s.x = current.q;
	LPF_current_q_s = LowPassFilter(LPF_current_q_s);
	current.q = LPF_current_q_s.y_prev;   // filter values

	LPF_current_d_s.x = current.d;
	LPF_current_d_s = LowPassFilter(LPF_current_d_s);
	current.d = LPF_current_d_s.y_prev;   // filter values

	// calculate the phase voltages
	PID_current_q_s.error = current_sp - current.q;
	PID_current_q_s = PID(PID_current_q_s);
	voltage.q = PID_current_q_s.output_prev;

	PID_current_d_s.error = 0 - current.d;
	PID_current_d_s = PID(PID_current_d_s);
	voltage.d = PID_current_d_s.output_prev;

	// set the phase voltage - FOC heart function :)
	setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

//Low-Pass Filter
struct LPF LowPassFilter(struct LPF LPF) {
	unsigned long timestamp = micros();

	float dt = (timestamp - LPF.timestamp_prev) * 1e-6f;

	if (dt < 0.0f)
		dt = 1e-3f;
	else if (dt > 0.3f) {
		LPF.y_prev = LPF.x;
		LPF.timestamp_prev = timestamp;
		return LPF;
	}

	float alpha = LPF.Tf / (LPF.Tf + dt);
	float y = alpha * LPF.y_prev + (1.0f - alpha) * LPF.x;

	LPF.y_prev = y;
	LPF.timestamp_prev = timestamp;

	return LPF;
}

//float PID(float error,float P, float I, float D, float output_ramp, float limit, unsigned long timestamp_prev, float integral_prev, float error_prev , float output_prev)
struct PID PID(struct PID PID) {

	// calculate the time from the last call
	unsigned long timestamp_now = micros();
	float Ts = (timestamp_now - PID.timestamp_prev) * 1e-6;
	// quick fix for strange cases (micros overflow)
	if (Ts <= 0 || Ts > 0.5)
		Ts = 1e-3;
	dtx = Ts;
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	float proportional = PID.P * PID.error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float integral = PID.integral_prev
			+ PID.I * Ts * 0.5f * (PID.error + PID.error_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -PID.limit, PID.limit);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	float derivative = PID.D * (PID.error - PID.error_prev) / Ts;

	// sum all the components
	float output = proportional + integral + derivative;
	// antiwindup - limit the output variable
	output = _constrain(output, -PID.limit, PID.limit);

	// if output ramp defined
	float output_ramp = PID.output_ramp;
	if (output_ramp > 0) {
		// limit the acceleration by ramping the output
		float output_rate = (output - PID.output_prev) / Ts;
		if (output_rate > output_ramp)
			output = PID.output_prev + output_ramp * Ts;
		else if (output_rate < -output_ramp)
			output = PID.output_prev - output_ramp * Ts;
	}

	// saving for the next pass
	PID.integral_prev = integral;
	PID.output_prev = output;
	PID.error_prev = PID.error;
	PID.timestamp_prev = timestamp_now;
	return PID;
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
	PID_velocity_s.error = shaft_velocity_sp - shaft_velocity;
	PID_velocity_s = PID(PID_velocity_s);
	current_sp = PID_velocity_s.output_prev;
	// if current/foc_current torque control
	// if torque controlled through voltage control
//  voltage.q = current_sp*phase_resistance;
//  voltage.d = 0;

}

void move_angle(float new_target)
{
  // get angular velocity
  shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

  // downsampling (optional)
  // if(motion_cnt++ < motion_downsample) return;
  // motion_cnt = 0;
  // set internal target variable
  if(_isset(new_target))
	  target = new_target;

  // angle set point
  shaft_angle_sp = target;
  // calculate velocity set point
  PID_angle_s.error = shaft_angle_sp - shaft_angle;
  PID_angle_s = PID(PID_angle_s);
  shaft_velocity_sp = PID_angle_s.output_prev;
  // calculate the torque command
  PID_velocity_s.error = shaft_velocity_sp - shaft_velocity ;
  PID_velocity_s = PID(PID_velocity_s);
  current_sp = PID_velocity_s.output_prev;

//  voltage.q = current_sp*phase_resistance;
//  voltage.d = 0;
}

void move_haptic(float new_target, float passivity_gain)
{
	// get angular velocity
	  shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

	  // downsampling (optional)
	  // if(motion_cnt++ < motion_downsample) return;
	  // motion_cnt = 0;
	  // set internal target variable
	  if(_isset(new_target))
		  target = new_target;
	  // angle set point
	  shaft_angle_sp = target;
	  // calculate velocity set point
	  PID_haptic_s.error = shaft_angle_sp - shaft_angle;
	  PID_haptic_s = PID(PID_haptic_s);
	  current_sp = PID_haptic_s.output_prev + (shaft_velocity * passivity_gain);
}

void move_torque(float new_target) {
//      if(!_isset(phase_resistance))
//    	  voltage.q = new_target;
//      else
//    	  voltage.q =  new_target*phase_resistance;
//    	  voltage.d = 0;
	current_sp = new_target * phase_resistance; // if current/foc_current torque control
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
void Error_Handler(void) {
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

