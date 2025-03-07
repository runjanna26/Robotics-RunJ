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

#include "ekf.h"

extern SPI_HandleTypeDef hspi1;
extern int pole_pairs;
extern LowPassFilter LPF_position;
extern LowPassFilter LPF_velocity;

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
	int32_t getFullRotations();												//checked
	float electricalAngle();
	float getMechanicalVelocity();
	float getSensorVelocity();  					 						//checked

	void updateSensor();													//checked
	void updateVelocity();
	// Filtered values
	float getShaftAngle();
	float getShaftVelocity();

	static uint32_t micros(void);											//checked

	//===Magnetic Encoder===//
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
	float vel_prev_EKF = 0;
	int32_t full_rotations = 0; 		// full rotation tracking
	int32_t vel_full_rotations = 0; 	// previous full rotation value for velocity calculation

	// Position sensor variable
	float sensor_offset = UNKNOWN; //!< user defined sensor zero offset
	float zero_electric_angle = NOT_SET; //!< absolute zero electric angle - if available
	int sensor_direction = NOT_SET; //!< if sensor_direction == Direction::CCW then direction will be flipped to CW

private:

	// EKF
	EKF ekf_encoder{2,1};
	ekf_t _ekf_s_encoder;

	static constexpr int EKF_N = 2;  // State vector dimension: [angle, velocity]
	static constexpr int EKF_M = 1;  // Measurement vector dimension: [angle]

	const float Pdiag[EKF_N] = {1, 1};

    _float_t x[EKF_N];          // State vector
    _float_t P[EKF_N * EKF_N];  // Prediction error covariance

    static constexpr _float_t Ts = 0.001f; // Sample time

    // Process noise covariance: Higher values indicate more uncertainty
    // Q00 [Position]: small because encoder is good. (1e-9)
    // Q11 [Velocity]: large because velocity calculation is bad. (1.0)
    const _float_t Q[EKF_N * EKF_N] = {1e-9, 0,
    								   0, 1e-6};
    // Measurement noise covariance
    // Lower values indicate higher confidence in measurements
    const _float_t R[EKF_M * EKF_M] = {1e-9};

    // State transition model
    const _float_t F[EKF_N * EKF_N] = {1, Ts, 	// Position
    								   0, 1};   // Velocity
    // Observation model: defines how measurements are mapped to the state space.
    const _float_t H[EKF_M * EKF_N] = {1, 0};
};

#endif /* AS5048AINTERFACE_H_ */
