/*
 * pwm_drivers.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <pwm_drivers.h>
#include "stm32f1xx_hal.h"  // Include the HAL header for your specific MCU
#include "motor_param.h"
pwm_drivers::pwm_drivers() {
	// TODO Auto-generated constructor stub

}

pwm_drivers::~pwm_drivers() {
	// TODO Auto-generated destructor stub
}


//Write PWM fsw = 25kHzfloat Ts
void pwm_drivers::writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c) 
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ARR_MAX_CA*dc_a);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ARR_MAX_CA*dc_b);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ARR_MAX_CA*dc_c);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
void pwm_drivers::setPhaseVoltage(float Uq, float Ud, float angle_el) 
{
	float Uout;
	// a bit of optitmisation
	if (Ud) {
		// only if Ud and Uq set
		// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrtApprox(Ud*Ud + Uq*Uq) / voltage_limit;
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
