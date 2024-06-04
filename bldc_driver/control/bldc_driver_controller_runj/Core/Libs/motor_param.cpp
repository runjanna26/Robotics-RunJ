/*
 * motor_param.cpp
 *
 *  Created on: May 20, 2024
 *      Author: WINDOWS 11
 */
#include "motor_param.h"


// current sensor parameter
float CurrentSense_resistance = 0.010;
float CurrentSense_gain = 5.0;

// motor configuration parameters
float phase_resistance = 0.15; 	//!< motor phase resistance
float phase_inductance = 33e-6;
int pole_pairs = 21;			//!< motor pole pairs number
float voltage_power_supply = 24.0;
// limiting variables
float voltage_limit = 24.0; 			//!< Voltage limitting variable - global limit
float current_limit = 10.0; 			//!< Current limitting variable - global limit (current_sp maximum)
float velocity_limit = 20.0; 			//!< Velocity limitting variable - global limit (maximum velocity of the position control)

PIDController PID_current_d {1.0, 0.0, 0.0, 1000.0, voltage_limit};
PIDController PID_current_q {1.0, 0.0, 0.0, 1000.0, voltage_limit};
PIDController PID_velocity  {5.0, 0.0, 0.0, 1000.0, current_limit};
PIDController PID_position  {1.0, 0.0, 0.0, 0, velocity_limit};

LowPassFilter LPF_current_q	{0.1f}; // 1 is very slow
LowPassFilter LPF_current_d	{0.1f}; // 1 is very slow
LowPassFilter LPF_position	{0.01}; // 1 is very slow
LowPassFilter LPF_velocity	{0.0};  // 1 is very slow (EKF)