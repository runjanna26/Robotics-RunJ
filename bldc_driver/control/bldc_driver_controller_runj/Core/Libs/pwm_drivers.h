/*
 * pwm_drivers.h
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#ifndef PWM_DRIVERS_H_
#define PWM_DRIVERS_H_

#include "main.h"
#include "foc_utils.h"
#include "AS5048Ainterface.h"


extern TIM_HandleTypeDef htim1;
extern float voltage_power_supply;
extern float voltage_limit;

class pwm_drivers {
public:
	pwm_drivers();
	virtual ~pwm_drivers();

	//PWM function
	void writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c);  //checked
	void setPhaseVoltage(float Uq, float Ud, float angle_el);   //checked

private:
//	simpleFOC simplefoc;
//	fast_math fastmath;
//	AS5048A_interface Encoder;
	//SVPWM
	float Ua, Ub, Uc;	// Current phase voltages Ua,Ub,Uc set to motor

};

#endif /* PWM_DRIVERS_H_ */
