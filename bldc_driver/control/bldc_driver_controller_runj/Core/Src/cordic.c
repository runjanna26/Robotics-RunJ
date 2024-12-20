/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cordic.h"

/* USER CODE BEGIN 0 */
#include<math.h>
/* USER CODE END 0 */

extern CORDIC_HandleTypeDef hcordic;

/* CORDIC init function */
void MX_CORDIC_Init(void)
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

//void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
//{
//
//  if(cordicHandle->Instance==CORDIC)
//  {
//  /* USER CODE BEGIN CORDIC_MspInit 0 */
//
//  /* USER CODE END CORDIC_MspInit 0 */
//    /* CORDIC clock enable */
//    __HAL_RCC_CORDIC_CLK_ENABLE();
//  /* USER CODE BEGIN CORDIC_MspInit 1 */
//
//  /* USER CODE END CORDIC_MspInit 1 */
//  }
//}

//void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
//{
//
//  if(cordicHandle->Instance==CORDIC)
//  {
//  /* USER CODE BEGIN CORDIC_MspDeInit 0 */
//
//  /* USER CODE END CORDIC_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_CORDIC_CLK_DISABLE();
//  /* USER CODE BEGIN CORDIC_MspDeInit 1 */
//
//  /* USER CODE END CORDIC_MspDeInit 1 */
//  }
//}

/* USER CODE BEGIN 1 */

/**************convert 32 bits float to a notation integer*****************/

inline int f32_to_q31( float input )
{
	const float Q_31_MAX_F = 1.0f ;// 0x0.FFFFFFp0F;//
	const float Q_31_MIN_F =  -1.0f;
	return (int)roundf(scalbnf(fmaxf(fminf(input,Q_31_MAX_F),Q_31_MIN_F),31));

}

#define pi  3.14159265359

/*****convert  notation integer to  32 bits float*********/

#define q31_to_f32(x) ldexp((int32_t)x, -31)   //q31 represent no in range -1,1

float cordic_q31_cosf(float x)
{

  CORDIC_ConfigTypeDef sConfig;
  int32_t input_q31 = f32_to_q31(fmod(x,2.0f*pi)/(2.0f*pi))<<1;
  int32_t output_q31;

//  int32_t input_q31 = x;
//  int32_t output_q31;

  sConfig.Function = CORDIC_FUNCTION_COSINE;
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;
  sConfig.Scale = CORDIC_SCALE_0;
  sConfig.NbWrite = CORDIC_NBWRITE_1;
  sConfig.NbRead = CORDIC_NBREAD_1;
  sConfig.InSize = CORDIC_INSIZE_32BITS;
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;

  HAL_CORDIC_Configure(&hcordic, &sConfig);
  HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31,1 ,0);
 //HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31,1 ,0);



   return q31_to_f32(output_q31);
  //return output_q31;

}
//static inline
float cordic_q31_sinf(float x)
{

  CORDIC_ConfigTypeDef sConfig;
  int32_t input_q31 = f32_to_q31(fmod(x,2.0f*pi)/(2.0f*pi))<<1;
  int32_t output_q31;

  sConfig.Function = CORDIC_FUNCTION_SINE ;
  sConfig.Precision = CORDIC_PRECISION_6CYCLES;
  sConfig.Scale = CORDIC_SCALE_0;
  sConfig.NbWrite = CORDIC_NBWRITE_1;
  sConfig.NbRead = CORDIC_NBREAD_1;
  sConfig.InSize = CORDIC_INSIZE_32BITS;
  sConfig.OutSize = CORDIC_OUTSIZE_32BITS;

  HAL_CORDIC_Configure(&hcordic, &sConfig);
  HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31,1 ,0);


  return q31_to_f32(output_q31);

}
/* USER CODE END 1 */
