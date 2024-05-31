/*
 * AS5048Ainterface.h
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#ifndef AS5048AINTERFACE_H_
#define AS5048AINTERFACE_H_

#include "main.h"
#include "math.h"
#include <cstdint>  // for uint32_t
#include "foc_utils.h"
#include "lowpass_filter.h"

extern SPI_HandleTypeDef hspi1;

#define DEF_ANGLE_REGISTER 0x3FFF

struct MagneticSensorSPIConfig_s 
{
	int bit_resolution;
	int angle_registers;
	int data_start_bit;
	int command_rw_bit;
	int command_parity_bit;
};



class AS5048A_interface {
public:
	AS5048A_interface();
	virtual ~AS5048A_interface();

	void MagneticSensorSPI_init(); 	//checked
	void Sensor_init();
	uint8_t spiCalcEvenParity(uint16_t value);								//checked
	uint16_t read(uint16_t angle_register);									//checked
	int getRawCount();														//checked
	float getSensorAngle();													//checked
	float get_full_rotation_angle();										//checked
	float getMechanicalAngle();												//checked
	int32_t getFullRotations();												//
	// float _electricalAngle(float shaft_angle, int pole_pairs);
	float electricalAngle();
	// float _normalizeAngle(float angle);
	float getMechanicalVelocity();
	float getSensorVelocity();  					 						//checked

	void updateSensor();													//checked

	// Filtered values
	float getShaftAngle();
	float getShaftVelocity();


	static uint32_t micros(void);											//checked




	//===Magnetic Sensor===//
	float cpr; 							// Maximum range of the magnetic sensor
	int bit_resolution; 				//!< the number of bites of angle data
	int command_parity_bit; 			//!< the bit where parity flag is stored in command
	int command_rw_bit; 				//!< the bit where read/write flag is stored in command
	int data_start_bit; 				//!< the the position of first bit
	int angle_register; 				//!< SPI angle register to read

	float angle_prev = 0; 				// result of last call to getSensorAngle(), used for full rotations and velocity
	long angle_prev_ts = 0; 			// timestamp of last call to get_full_rotation_angle, used for velocity
	float vel_angle_prev = 0; 			// angle at last call to getVelocity, used for velocity
	long vel_angle_prev_ts = 0;			// last velocity calculation timestamp
	float vel_prev = 0;					// velocity at last call 
	float vel_prev_LPF = 0;
	int32_t full_rotations = 0; 		// full rotation tracking
	int32_t vel_full_rotations = 0; 	// previous full rotation value for velocity calculation

	//Position sensor variable
	float sensor_offset = UNKNOWN; //!< user defined sensor zero offset
	float zero_electric_angle = NOT_SET; //!< absolute zero electric angle - if available



	int sensor_direction = NOT_SET; //!< if sensor_direction == Direction::CCW then direction will be flipped to CW
//	int sensor_direction = CW;
	
	int pole_pairs = 18; 

	LowPassFilter LPF_position	{0.01};
	LowPassFilter LPF_velocity	{0.2};
};

#endif /* AS5048AINTERFACE_H_ */
