/*
 * simpleFOC.h
 *
 *  Created on: May 19, 2024
 *      Author: runj
 */

#ifndef SIMPLEFOC_H_
#define SIMPLEFOC_H_

#include "main.h"
#include "math.h"
#include <cstdint>  // for uint32_t
#include "motor_param.h"

#include "AS5048Ainterface.h"
#include "CurrentSense.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "foc_utils.h"

#include "pwm_drivers.h"

extern SPI_HandleTypeDef hspi1;

extern float velocity_limit;
extern int pole_pairs;
extern LowPassFilter LPF_current_q;
extern LowPassFilter LPF_current_d;
extern PIDController PID_current_d;
extern PIDController PID_current_q;
extern PIDController PID_velocity;
extern PIDController PID_position;
extern float phase_resistance;

class simpleFOC {
public:
	simpleFOC();
	virtual ~simpleFOC();

	void initSensors();

	//Initialize and Search "zero_electric_angle"
	int needsSearch();											//checked
	int absoluteZeroSearch();									//don't use
	int alignSensor();											//checked
	int initFOC(float zero_electric_offset, enum Direction _sensor_direction); //checked
	
	void readEncoderOnly();
	//Main loop FOC
	void loopFOC();

	//Closed Loop
	// void move_velocity(float new_target);		//closed loop			//checked
	void move_angle(float new_target);	 //closed loop			//checked
	// void move_haptic(float new_target, float passivity_gain);
	// void move_torque(float new_target);  //closed loop			//checked
	
	//Open Loop
	float velocityOpenloop(float target_velocity);		//checked 50 rpm so smooth
	void move_velocity_openloop(float target);			//checked 50 rpm so smooth
	float angleOpenloop(float target_angle);			//
	
	static uint32_t micros(void);											//checked

	AS5048A_interface Encoder;
	CurrentSense CurrentSensor;
	pwm_drivers driver;

private:
	////Control Variable
	float target;             			//!< current target value - depends of the controller
	float shaft_angle;        			//!< current motor angle
	float electrical_angle;   			//!< current electrical angle
	float shaft_velocity;     			//!< current motor velocity
	float current_sp;         			//!< target current ( q current )
	float shaft_velocity_sp;  			//!< current target velocity
	float shaft_angle_sp;     			//!< current target angle
	struct DQVoltage_s voltage;      	//!< current d and q voltage set to the motor
	struct DQCurrent_s current;      	//!< current d and q current measure



	long open_loop_timestamp;

	float pp_check;
	float voltage_sensor_align = 3;		//!< sensor and motor align voltage parameter
	float velocity_index_search = 3;	//!< target velocity for index search
};

#endif /* SIMPLEFOC_H_ */
