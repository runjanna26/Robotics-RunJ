/*
 * CurrentSense.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <CurrentSense.h>
#include "stm32f1xx_hal.h"  // Include the HAL header for your specific MCU


CurrentSense::CurrentSense() 
{
	
}

CurrentSense::~CurrentSense() 
{
	// TODO Auto-generated destructor stub
}

/**
 * @brief Initialize Direct Memory Access (DMA) for Analog to Digital Convertor (ADC)
 * 			, which use to read current sensor signals 
*/
void CurrentSense::initCurrentsense(float _shunt_resistor, float _gain) 
{
	HAL_ADC_Start_DMA(&hadc1, adcResultDMA, 3);

	R_sense = _shunt_resistor;
	gain_a = _gain;
	gain_b = _gain;
	gain_c = _gain;
}

/**
 * @brief Calibrate current offset in initial steady state (first 1000 iterations)
*/
void CurrentSense::calibrateOffsets() 
{
	const int calibration_rounds = 1000;
	// find adc offset = zero current voltage
	offset_ia = 0;
	offset_ib = 0;
	offset_ic = 0;
	// read the adc voltage 1000 times ( arbitrary number )
	for (int i = 0; i < calibration_rounds; i++) 
	{
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

/**
 * @brief Read phase current in ADC memory by using DMA method
 * @note it can be 2 phase current sensor, you need to uncomment the last line --(1)
 * 		i_a + i_b + i_c = 0  where's based on 3-phase current theorem
 * @note the value 3.05 to 0.25 is the range of ADC signal from DRV8323RH, please see Datasheet.
 * 
 * @return  Phase current in Struct PhaseCurrent_s (Amperes)
*/
struct PhaseCurrent_s CurrentSense::getPhaseCurrents() 
{
	struct PhaseCurrent_s current;
	current.a = ((3.3 / 2) - (adcResultDMA[0] - 0) * ((3.05 - 0.25) / (3785.0 - 311.0))) / (R_sense * gain_a);
	current.b = ((3.3 / 2) - (adcResultDMA[1] - 0) * ((3.05 - 0.25) / (3785.0 - 311.0))) / (R_sense * gain_b);
	current.c = ((3.3 / 2) - (adcResultDMA[2] - 0) * ((3.05 - 0.25) / (3785.0 - 311.0))) / (R_sense * gain_c);
//    current.b = - current.a  - current.c; // --(1)
	return current;
}

/**
 * @brief Calculate DQ currents from Phase currents
 * @note function calculate by Clarke-Park transform of the phase currents
 * 
 * @return  DQ current in Struct DQCurrent_s (Amperes)
*/
struct DQCurrent_s CurrentSense::getFOCCurrents(float angle_el) 
{
	// read current phase currents
	struct PhaseCurrent_s current = getPhaseCurrents(); //Ia,Ib,Ic

	// calculate clarke transform
	float i_alpha, i_beta;

    // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
    float mid = (1.f/3) * (current.a + current.b + current.c);
    float a = current.a - mid;
    float b = current.b - mid;
    i_alpha = a;
    i_beta = _1_SQRT3 * a + _2_SQRT3 * b;


	// calculate park transform
	float ct = _cos(angle_el);
	float st = _sin(angle_el);
	struct DQCurrent_s dq_current;
	dq_current.d = i_alpha * ct + i_beta  * st;
	dq_current.q = i_beta  * ct - i_alpha * st;
	return dq_current;
}
