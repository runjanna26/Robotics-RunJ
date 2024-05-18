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

extern SPI_HandleTypeDef hspi1;

//Define for calculation
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) ) // Check it later
#define _sqrt(a) (_sqrtApprox(a))
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _isset(a) ( (a) != (NOT_SET) )
#define NOT_SET -12345.0

#define DEF_ANGLE_REGISTER 0x3FFF

	struct MagneticSensorSPIConfig_s {
		int bit_resolution;
		int angle_registers;
		int data_start_bit;
		int command_rw_bit;
		int command_parity_bit;
	};

	//Low pass filter
	struct LPF // LPF struct
	{
		float x;							//!< (INPUT)
		unsigned long timestamp_prev;  	//!< Last execution timestamp
		float y_prev; 		//!< filtered value in previous execution step (OUTPUT)
		float Tf;
	};

class simpleFOC {
public:
	simpleFOC();
	virtual ~simpleFOC();

	void MagneticSensorSPI_init(); 	//checked
	void Sensor_init();
	uint8_t spiCalcEvenParity(uint16_t value);								//checked
	uint16_t read(uint16_t angle_register);									//checked
	int getRawCount();														//checked
	float getSensorAngle();													//checked
	float shaftAngle();
	float getAngle();														//checked
	float getMechanicalAngle();												//checked
	int32_t getFullRotations();												//
	float _electricalAngle(float shaft_angle, int pole_pairs);
	float electricalAngle();

	float _normalizeAngle(float angle);

	void updateSensor();													//checked


	struct LPF LowPassFilter(struct LPF LPF); 								//checked

	float getvelocity();  					 								//checked
	float shaftVelocity();

	static uint32_t micros(void);									//checked

private:
	//===Magnetic Sensor===//
	float cpr; // Maximum range of the magnetic sensor
	int bit_resolution; 	//!< the number of bites of angle data
	int command_parity_bit; //!< the bit where parity flag is stored in command
	int command_rw_bit; 	//!< the bit where read/write flag is stored in command
	int data_start_bit; 	//!< the the position of first bit
	int angle_register; 	//!< SPI angle register to read

	float angle_prev = 0; // result of last call to getSensorAngle(), used for full rotations and velocity
	long angle_prev_ts = 0; // timestamp of last call to getAngle, used for velocity
	float vel_angle_prev = 0; // angle at last call to getVelocity, used for velocity
	long vel_angle_prev_ts = 0; 	// last velocity calculation timestamp
	float vel_prev = 0;
	int32_t full_rotations = 0; 	// full rotation tracking
	int32_t vel_full_rotations = 0; // previous full rotation value for velocity calculation




	//Position sensor variable
	float sensor_offset = 0; //!< user defined sensor zero offset
	float zero_electric_angle = NOT_SET; //!< absolute zero electric angle - if available
	int sensor_direction = NOT_SET; //!< if sensor_direction == Direction::CCW then direction will be flipped to CW
	
	

	int pole_pairs = 14;	

};

#endif /* SIMPLEFOC_H_ */
