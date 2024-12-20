/*
 * motor_param.cpp
 *
 *  Created on: May 20, 2024
 *      Author: WINDOWS 11
 */
#include "motor_param.h"

	// motor configuration parameters


	float phase_resistance = 7.1; 	//!< motor phase resistance
	int pole_pairs = 14;			//!< motor pole pairs number
	float voltage_power_supply = 24.0;

	// limiting variables
	float voltage_limit = 24.0; 			//!< Voltage limitting variable - global limit
	float current_limit = 20.0; 			//!< Current limitting variable - global limit (current_sp maximum)
	float velocity_limit = 20.0; 			//!< Velocity limitting variable - global limit (maximum velocity of the position control)

	
	PIDController PID_current_d {1.0, 0.0, 0.0, 1000.0, voltage_limit};
	PIDController PID_current_q {1.0, 0.0, 0.0, 1000.0, voltage_limit};
	PIDController PID_velocity  {0.5, 55.0, 0.0, 1000.0, current_limit};
	PIDController PID_position  {20.0, 0.0, 1.0, 0, velocity_limit};

	LowPassFilter LPF_current_q	{0.001};
	LowPassFilter LPF_current_d	{0.001};