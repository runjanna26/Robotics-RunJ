/*
 * pwm_drivers.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <pwm_drivers.h>
#include "stm32g4xx_hal.h"  // Include the HAL header for your specific MCU
#include "motor_param.h"
pwm_drivers::pwm_drivers() {
	// TODO Auto-generated constructor stub

}

pwm_drivers::~pwm_drivers() {
	// TODO Auto-generated destructor stub
}

void pwm_drivers::initDriver()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   //pinMode
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	//pinMode
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	//pinMode
}

//Write PWM fsw = 25kHzfloat Ts
void pwm_drivers::writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c) 
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, fsw*dc_a);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fsw*dc_b);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fsw*dc_c);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//void pwm_drivers::setPhaseVoltage(float Uq, float Ud, float angle_el)
//{
//// ===========================================================================
////	// Inverse Park + Clarke transformation
////
////	// angle normalization in between 0 and 2pi
////	// only necessary if using _sin and _cos - approximation functions
////		angle_el = _normalizeAngle(angle_el);
////	// Inverse park transform
////		float _ca = _cos(angle_el);
////		float _sa = _sin(angle_el);
////		float iv_alpha, iv_beta;
////
////		iv_alpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
////		iv_beta = _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;
////
////	// center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
////		float center = voltage_limit / 2;
////        float Umin = min(Ua, min(Ub, Uc));
////        float Umax = max(Ua, max(Ub, Uc));
////        center -= (Umax + Umin) / 2;
////
////	// Inverse Clarke transform
////		Ua = iv_alpha + center;
////		Ub = -0.5f * iv_alpha + _SQRT3_2 * iv_beta + center;
////		Uc = -0.5f * iv_alpha - _SQRT3_2 * iv_beta + center;
//
//
//// ===========================================================================[Old]
//	float Uout;
//	// a bit of optitmisation
//	if (Ud)
//	{
//		// only if Ud and Uq set
//		// _sqrt is an approx of sqrt (3-4% error)
//		Uout = _sqrtApprox(Ud*Ud + Uq*Uq) / voltage_limit;
//		// angle normalisation in between 0 and 2pi
//		// only necessary if using _sin and _cos - approximation functions
//		angle_el = _normalizeAngle(angle_el + _atan2(Uq, Ud));
//	}
//	else
//	{
//		// only Uq available - no need for atan2 and sqrt
//		Uout = Uq / voltage_limit;
//		// angle normalisation in between 0 and 2pi
//		// only necessary if using _sin and _cos - approximation functions
//		angle_el = _normalizeAngle(angle_el + _PI_2);
//	}
//	// find the sector we are in currently
//	int sector = floor(angle_el / _PI_3) + 1;
//	// calculate the duty cycles
//	float T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
//	float T2 = _SQRT3 * _sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
//    float T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2
////	float T0 = 0; // pulled to 0 - better for low power supply voltage
//
//	// calculate the duty cycles(times)
//	float Ta, Tb, Tc;
//	switch (sector) {
//	case 1:
//		Ta = T1 + T2 + T0 / 2;
//		Tb = T2 + T0 / 2;
//		Tc = T0 / 2;
//		break;
//	case 2:
//		Ta = T1 + T0 / 2;
//		Tb = T1 + T2 + T0 / 2;
//		Tc = T0 / 2;
//		break;
//	case 3:
//		Ta = T0 / 2;
//		Tb = T1 + T2 + T0 / 2;
//		Tc = T2 + T0 / 2;
//		break;
//	case 4:
//		Ta = T0 / 2;
//		Tb = T1 + T0 / 2;
//		Tc = T1 + T2 + T0 / 2;
//		break;
//	case 5:
//		Ta = T2 + T0 / 2;
//		Tb = T0 / 2;
//		Tc = T1 + T2 + T0 / 2;
//		break;
//	case 6:
//		Ta = T1 + T2 + T0 / 2;
//		Tb = T0 / 2;
//		Tc = T1 + T0 / 2;
//		break;
//	default:
//		// possible error state
//		Ta = 0;
//		Tb = 0;
//		Tc = 0;
//	}
//	// calculate the phase voltages
//	Ua = Ta * voltage_limit;
//	Ub = Tb * voltage_limit;
//	Uc = Tc * voltage_limit;
//// ===========================================================================
//
//	// set the voltages in hardware
//	// limit the voltage in driver
//	Ua = _constrain(Ua, 0.0f, voltage_limit);
//	Ub = _constrain(Ub, 0.0f, voltage_limit);
//	Uc = _constrain(Uc, 0.0f, voltage_limit);
//	// calculate duty cycle
//	float dc_a;  //duty cycle phase A [0, 1]
//	float dc_b;  //duty cycle phase B [0, 1]
//	float dc_c;  //duty cycle phase C [0, 1]
//	// limited in [0,1]
//	dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 0.94f);
//	dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 0.94f);
//	dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 0.94f);
//	writeDutyCycle3PWM(dc_a, dc_b, dc_c);
//}

void pwm_drivers::setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	// =========================================================================== Inverse Park + Clarke transformation
	//	//
	//
	//	// angle normalization in between 0 and 2pi
	//	// only necessary if using _sin and _cos - approximation functions
	//		angle_el = _normalizeAngle(angle_el);
	//	// Inverse park transform
	//		float _ca = _cos(angle_el);
	//		float _sa = _sin(angle_el);
	//		float iv_alpha, iv_beta;
	//
	//		iv_alpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
	//		iv_beta = _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;
	//
	//	// center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
	//		float center = voltage_limit / 2;
	//        float Umin = min(Ua, min(Ub, Uc));
	//        float Umax = max(Ua, max(Ub, Uc));
	//        center -= (Umax + Umin) / 2;
	//
	//	// Inverse Clarke transform
	//		Ua = iv_alpha + center;
	//		Ub = -0.5f * iv_alpha + _SQRT3_2 * iv_beta + center;
	//		Uc = -0.5f * iv_alpha - _SQRT3_2 * iv_beta + center;
	// =========================================================================== Space Vector PWM (SVPWM)


    // Constants
    const float MAX_DUTY_CYCLE = 0.94f;  // Maximum duty cycle limit
    const float CENTER_OFFSET = voltage_limit / 2; // Modulation center for voltage

    // Normalization of the electrical angle
    angle_el = _normalizeAngle(angle_el);

    // Calculate output voltage magnitude and adjust angle
    float Uout, iv_alpha, iv_beta;
    if (Ud) {
        // Both Ud and Uq are set
        Uout = _sqrtApprox(Ud * Ud + Uq * Uq) / voltage_limit;
        angle_el = _normalizeAngle(angle_el + _atan2(Uq, Ud));
    } else {
        // Only Uq is set, avoid sqrt and atan2
        Uout = Uq / voltage_limit;
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }

    // Sector determination
    int sector = (int)(angle_el / _PI_3) + 1;

    // Calculate duty cycles
    float T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    float T2 = _SQRT3 * _sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
    float T0 = 1 - T1 - T2; // Modulation centered around `voltage_limit / 2`

    // Adjust modulation for low voltage operation if needed
    // T0 = 0; // Uncomment if required for low voltage supply operation

    // Duty cycle times for each phase
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
            // Error handling for invalid sector
            Ta = Tb = Tc = 1.0f / 3.0f; // Equal distribution as fallback
            break;
    }

    // Calculate phase voltages
    Ua = Ta * voltage_limit;
    Ub = Tb * voltage_limit;
    Uc = Tc * voltage_limit;

    // Enforce voltage limits
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    // Ensure the total voltage is within the hardware voltage limit
    float total_voltage = Ua + Ub + Uc;
    if (total_voltage > voltage_limit) {
        float scale_factor = voltage_limit / total_voltage;
        Ua *= scale_factor;
        Ub *= scale_factor;
        Uc *= scale_factor;
    }

    // Calculate duty cycles for PWM
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, MAX_DUTY_CYCLE);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, MAX_DUTY_CYCLE);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, MAX_DUTY_CYCLE);

    // Apply duty cycles to the hardware
    writeDutyCycle3PWM(dc_a, dc_b, dc_c);
}
