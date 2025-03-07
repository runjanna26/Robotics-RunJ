/*
 * AS5048Ainterface.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <AS5048Ainterface.h>
#include "stm32g4xx_hal.h" // Include the HAL header for your specific MCU

struct MagneticSensorSPIConfig_s AS5048A_SPI = {
	    .bit_resolution = 14,
	    .angle_registers = 0x3FFF,
	    .data_start_bit = 13,
	    .command_rw_bit = 14,
	    .command_parity_bit = 15
};



AS5048A_interface::AS5048A_interface() {
	// TODO Auto-generated constructor stub

}

AS5048A_interface::~AS5048A_interface() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief Initialize SPI for Magnetic Sensor with AS5048A_SPI
*/
void AS5048A_interface::MagneticSensorSPI_init() 
{
	struct MagneticSensorSPIConfig_s config = AS5048A_SPI;
	// angle read register of the magnetic sensor
	angle_register = config.angle_registers ? config.angle_registers : DEF_ANGLE_REGISTER;
	// register maximum value (counts per revolution)
	cpr = pow(2, config.bit_resolution);
	bit_resolution = config.bit_resolution;

	command_parity_bit = config.command_parity_bit; 	// for backwards compatibility
	command_rw_bit = config.command_rw_bit; 			// for backwards compatibility
	data_start_bit = config.data_start_bit; 			// for backwards compatibility

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Initialize the variable of encoder
*/
void AS5048A_interface::Sensor_init()
{
	// initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
	getSensorAngle(); 
	vel_angle_prev = getSensorAngle();
	vel_angle_prev_ts = micros();
	HAL_Delay(1);		// Wait for collecting data
	getSensorAngle();
	angle_prev = getSensorAngle(); 
	angle_prev_ts = micros();


	ekf_encoder.ekf_initialize(&_ekf_s_encoder, Pdiag);
}

/**
 * @brief Utility function used to calculate even parity of word
 */
uint8_t AS5048A_interface::spiCalcEvenParity(uint16_t value) 
{
	uint8_t cnt = 0;
	uint8_t i;
	for (i = 0; i < 16; i++) 
	{
		if (value & 0x1)
			cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}

/**
 * @brief Read a register from the SPI encoder sensor
 * 		  Takes the address of the register as a 16 bit word
 * @return the value of the register
 */
uint16_t AS5048A_interface::read(uint16_t angle_register) 
{
	uint16_t register_value;
	uint16_t command = angle_register;

	if (command_rw_bit > 0)
	{
		command = angle_register | (1 << command_rw_bit);
	}
	if (command_parity_bit > 0)
	{
		//Add a parity bit on the the MSB
		command |= ((uint16_t) spiCalcEvenParity(command) << command_parity_bit);
	}

	//>>>> SPI - begin transaction <<<<
	//Send the command
	//  spi->transfer16(command);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &command, (uint8_t*) &register_value, sizeof(register_value) / sizeof(uint16_t), 100);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

//	command = 0x0000;
	//Now read the response (NO_OPERATION_COMMAND = 0x0000)
	//  uint16_t register_value = spi->transfer16(0x00);
//	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) & command, (uint8_t*) &register_value, sizeof(register_value) / sizeof(uint16_t), 100);
//	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

	//>>>> SPI - end transaction <<<<

	register_value = register_value >> (1 + data_start_bit - bit_resolution); //this should shift data to the rightmost bits of the word
	uint16_t data_mask = 0xFFFF >> (16 - bit_resolution);
	return register_value & data_mask; // Return the data, stripping the non data (e.g parity) bits
}

// 

/**
 * @brief Reading the raw counter of the magnetic sensor
 * 
 * @return raw data from SPI signal [16-bits word]
*/
int AS5048A_interface::getRawCount() 
{
	return (int) read(angle_register);
}

/**
 * @brief Get absolute angular position from raw data of encoder
 *
 * @return absolute angular position [radians]
 */
float AS5048A_interface::getSensorAngle() 
{
	return (getRawCount() / (float) cpr) * _2PI;
}

/**
 * @brief Get absolute angular position with number of round (full rotation)
 * - if turn off the encoder full_rotations will be reset.
 * 
 * @return absolute angular position with number of round [radians]
 * 
*/
float AS5048A_interface::get_full_rotation_angle() 
{
	return (float) full_rotations * _2PI + angle_prev;
}


/**
 * @brief Get absolute angular position from last call updateSensor function
*/
float AS5048A_interface::getMechanicalAngle() 
{
	return angle_prev;
}

/**
 * @brief Get number of round (full ratations)
 * 
 * @return number of full rotations [int32_t]
*/
int32_t AS5048A_interface::getFullRotations() 
{
	return full_rotations;
}

/**
 * @brief Calculate electrical angular position from absoulute angular position 
 * 
 * @return electrical angular position [radians]
*/
float AS5048A_interface::electricalAngle() 
{
	return _normalizeAngle((float) (sensor_direction * pole_pairs) * getMechanicalAngle() - zero_electric_angle);
}


/**
 * @brief Get angular velocity from angular position
 * 
 * @return angular velocity [radians/second]
*/
float AS5048A_interface::getSensorVelocity() 
{
	// calculate sample time
	float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
	// quick fix for strange cases (micros overflow)
	if (Ts <= 0)
		Ts = 1e-3f;
	// velocity calculation
	vel_prev = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;
	// save variables for future pass
	vel_angle_prev = angle_prev;
	vel_full_rotations = full_rotations;
	vel_angle_prev_ts = angle_prev_ts;
	return vel_prev;
}

/**
 * @brief Get absolute angular velocity from last call updateSensor function
*/
float AS5048A_interface::getMechanicalVelocity() 
{
	return vel_prev;
}

/**
 * @brief Gather system clock and convert to microsecond
*/
uint32_t AS5048A_interface::micros(void) 
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}


/**
 * @brief Update parameter of encoder 
 * 	- should be used in the loop
*/
void AS5048A_interface::updateSensor() 
{
	float angle_current = getSensorAngle();
	angle_prev_ts = micros();
	float d_angle = angle_current - angle_prev;
	// if overflow happened track it as full rotation
	if (abs(d_angle) > (0.8f * _2PI))
		full_rotations += (d_angle > 0) ? -1 : 1;
	angle_prev = angle_current;

//	getShaftVelocity();
}

void AS5048A_interface::updateVelocity()
{
	// calculate sample time
	float Ts = 1e-3f;

	// [2] EKF Velocity estimation
	_float_t angle_measurement = get_full_rotation_angle();
	
    _float_t fx[EKF_N];
    fx[0] = _ekf_s_encoder.x[0] + _ekf_s_encoder.x[1] * Ts;   		// Angle update
    fx[1] = _ekf_s_encoder.x[1];                    				// Velocity remains the same
    ekf_encoder.ekf_predict(&_ekf_s_encoder, fx, F, Q);

    _float_t hx[EKF_N];
    hx[0] = _ekf_s_encoder.x[0];  // Predicted measurement
    ekf_encoder.ekf_update(&_ekf_s_encoder, &angle_measurement, hx, H, R);

    vel_prev_EKF = LPF_velocity(_ekf_s_encoder.x[1]);



    // [1] Velocity calculation
    vel_prev = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;

	// save variables for next iteration
	vel_angle_prev = angle_prev;
	vel_full_rotations = full_rotations;
	// Low pass filter
//	vel_prev_LPF = sensor_direction * LPF_velocity(vel_prev);
}




/*
 * Get filtered absolute angular position from encoder with sensor direction
 *
 * @return absolute angular position with sensor direction [radians]
*/
float AS5048A_interface::getShaftAngle() 
{
	return sensor_direction * LPF_position(get_full_rotation_angle()) - sensor_offset;
}

/*
 * Get filtered absolute angular velocity from encoder with sensor direction
 *
 * @return absolute angular velocity with sensor direction [radians/second]
*/
float AS5048A_interface::getShaftVelocity() 
{
	return vel_prev_EKF;
}



