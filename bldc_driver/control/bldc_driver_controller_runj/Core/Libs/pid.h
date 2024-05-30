/*
 * pid.h
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#ifndef PID_H_
#define PID_H_
#include "main.h"
#include "math.h"
#include <cstdint>  // for uint32_t
#include "foc_utils.h"


class PIDController
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator() (float error);
    void reset();
	static uint32_t micros(void);	

    float P; 				//!< Proportional gain 
    float I; 				//!< Integral gain 
    float D; 				//!< Derivative gain 
    float output_ramp; 		//!< Maximum speed of change of the output value
    float limit; 			//!< Maximum output value

protected:
    float error_prev; 				//!< last tracking error value
    float output_prev;  			//!< last pid output value
    float integral_prev; 			//!< last integral component value
    unsigned long timestamp_prev; 	//!< Last execution timestamp
};

#endif /* PID_H_ */
