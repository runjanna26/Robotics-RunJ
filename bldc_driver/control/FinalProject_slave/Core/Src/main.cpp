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
#include "motor_param.h"

#include "simpleFOC.h"
simpleFOC simpleFOC;



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


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

//Serial Write
int _write(int file, char *ptr, int len) 
{
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



unsigned long t1 = 0, ts, t2 = 0, t3, t4 = 0, t5 = 0;
float loop_freq = 0;
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

	//SPI SETUP
	simpleFOC.initSensors();

	//PWM SETUP
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   //pinMode
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	//pinMode
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	//pinMode

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);  // Enable
//	  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);  // Disable

	//CAN SETUP ID: 0x103
	CAN_init_103();
	t1 = micros();
	t2 = micros();

	//FOC SETUP
	simpleFOC.initFOC(5.88972235, CW); 			// Do not search!! checked
//	simpleFOC.initFOC(NOT_SET, UNKNOWN); 		//Not yet calibrate find the best init value

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
//		simpleFOC.move_velocity_openloop(5.0);
//		

//		simpleFOC.move_angle(0);
//		simpleFOC.loopFOC();


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
//	    encoder.updateSensor();
	    simpleFOC.readEncoderOnly();
//		Sx = simpleFOC.Encoder.getSensorVelocity();
//	    getMechanicalVelocity;
//		electrical_angle = get_full_rotation_angle();
//		shaft_angle = shaftAngle();
//		shaft_velocity = shaftVelocity();
//		sprintf(MSG, "%.3f \n",shaft_velocity );
//		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);



//=================Open-loop testing=================
//		move_velocity_openloop(30.0);
//		angleOpenloop(_PI_2);
//		HAL_Delay(3);

//  	  ts = micros()-t1;

//=================Closed-loop testing=================
		if(micros() - t1 >= 3000000)
	    {
	    	t1 = micros();
	    	Command_setpoint = Command_setpoint + _PI_2;
	    }
	  	if(Command_setpoint > _2PI)
	  		Command_setpoint = 0.0;
//		move_velocity(float_final); 									//136us --> 72us
//		loopFOC();  													//1190us --> 485us

//	  sprintf(MSG, "%.3f,%.3f \n",float_final,shaft_velocity);  		// 134us
//	  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);			  	// 181us

//		Position control
	  	simpleFOC.loopFOC(); 														//1190us --> 495 us
	  	simpleFOC.move_angle(Command_setpoint); 												//161 us --> 112 us
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
		t5 = micros() - t4;
		t4 = micros();
		loop_freq = 1.0 / (t5 * 1e-6);

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







// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
// void move_velocity(float new_target) 
// {
// 	// get angular velocity
// 	shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

// 	if (_isset(new_target))
// 		target = new_target;

// 	// velocity set point
// 	shaft_velocity_sp = target;

// 	// calculate the torque command
// 	PID_velocity_s.error = shaft_velocity_sp - shaft_velocity;
// 	PID_velocity_s = PID(PID_velocity_s);
// 	current_sp = PID_velocity_s.output_prev;
// 	// if current/foc_current torque control
// 	// if torque controlled through voltage control
//  voltage.q = current_sp*phase_resistance;
//  voltage.d = 0;

// }





//void move_torque(float new_target) {
////      if(!_isset(phase_resistance))
////    	  voltage.q = new_target;
////      else
////    	  voltage.q =  new_target*phase_resistance;
////    	  voltage.d = 0;
//	current_sp = new_target * phase_resistance; // if current/foc_current torque control
//}







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

