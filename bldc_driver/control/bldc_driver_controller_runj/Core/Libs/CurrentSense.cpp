/*
 * CurrentSense.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <CurrentSense.h>
#include "stm32g4xx_hal.h"  // Include the HAL header for your specific MCU


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
	HAL_ADC_Start_DMA(&hadc1, adcResultDMA_a, 1);
	HAL_ADC_Start_DMA(&hadc2, adcResultDMA_c, 1);

	R_sense = _shunt_resistor;
	gain_a = _gain;
	gain_b = _gain;
	gain_c = _gain;
	ekf_current.ekf_initialize(&_ekf_s_current, Pdiag);
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
		offset_ia += adcResultDMA_a[0];
//		offset_ib += adcResultDMA[0];
		offset_ic += adcResultDMA_c[0];
		HAL_Delay(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia / calibration_rounds;
//	offset_ib = offset_ib / calibration_rounds;
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
//	struct PhaseCurrent_s current;
	current.a = ((offset_ia - adcResultDMA_a[0]) * (3.3 / 4096.0)) / (R_sense * gain_a);
//	current.b = ((offset_ib - adcResultDMA_b[0]) * (3.3 / 4096.0)) / (R_sense * gain_b);
	current.c = ((offset_ic - adcResultDMA_c[0]) * (3.3 / 4096.0)) / (R_sense * gain_c);

//	float current_measurement[3] = {current.a, current.b, current.c};
//
//
//    fx[0] = _ekf_s_current.x[0] * (1 - ((CurrentSense_resistance + phase_resistance) * Ts / phase_inductance)); // Ia update
//    fx[1] = _ekf_s_current.x[1] * (1 - ((CurrentSense_resistance + phase_resistance) * Ts / phase_inductance)); // Ib update
//    fx[2] = _ekf_s_current.x[2] * (1 - ((CurrentSense_resistance + phase_resistance) * Ts / phase_inductance));	// Ic update
//    ekf_current.ekf_predict(&_ekf_s_current, fx, F, Q);
//
//    _float_t hx[EKF_N];
//    hx[0] = _ekf_s_current.x[0];  // Ia Predicted measurement
//    hx[1] = _ekf_s_current.x[1];  // Ib Predicted measurement
//    hx[2] = _ekf_s_current.x[2];  // Ic Predicted measurement
//    ekf_current.ekf_update(&_ekf_s_current, current_measurement, hx, H, R);
//
//
//    current_a_prev_EKF = _ekf_s_current.x[0];
//	current_b_prev_EKF = _ekf_s_current.x[1];
//	current_c_prev_EKF = _ekf_s_current.x[2];

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
//	struct PhaseCurrent_s current = getPhaseCurrents(); //Ia, Ib, Ic
	current = getPhaseCurrents(); //Ia,Ib,Ic

	// calculate clarke transform
	float i_alpha, i_beta;

//    // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
//    float mid = (1.f/3) * (current.a + current.b + current.c);
//    float a = current.a - mid;
//    float b = current.b - mid;
//    i_alpha = a;
//    i_beta = _1_SQRT3 * a + _2_SQRT3 * b;

	i_alpha = current.a;
	i_beta = (-(_1_SQRT3) * current.a) + (-(_2_SQRT3) * current.c);

	// calculate park transform
	float ct = _cos(angle_el);
	float st = _sin(angle_el);

//	struct DQCurrent_s dq_current;		// Id, Iq

	dq_current.d = i_alpha * ct + i_beta  * st;
	dq_current.q = i_beta  * ct - i_alpha * st;
	return dq_current;
}
