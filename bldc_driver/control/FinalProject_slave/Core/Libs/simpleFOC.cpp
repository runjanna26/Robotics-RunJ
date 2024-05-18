/*
 * simpleFOC.cpp
 *
 *  Created on: May 19, 2024
 *      Author: runj
 */

#include <simpleFOC.h>
#include "stm32f1xx_hal.h"  // Include the HAL header for your specific MCU

struct MagneticSensorSPIConfig_s AS5048A_SPI = {
	    .bit_resolution = 14,
	    .angle_registers = 0x3FFF,
	    .data_start_bit = 13,
	    .command_rw_bit = 14,
	    .command_parity_bit = 15
};

struct LPF LPF_angle_s = {
		.y_prev = 0.0,
		.Tf = 0.01,
};

struct LPF LPF_velocity_s = {
		.y_prev = 0.0,
		.Tf = 0.01,
};

simpleFOC::simpleFOC() {

}

simpleFOC::~simpleFOC() {
	// TODO Auto-generated destructor stub
}

// initialize SPI for Magnetic Sensor
void simpleFOC::MagneticSensorSPI_init() {
	struct MagneticSensorSPIConfig_s config = AS5048A_SPI;
	// angle read register of the magnetic sensor
	angle_register = config.angle_registers ? config.angle_registers : DEF_ANGLE_REGISTER;
	// register maximum value (counts per revolution)
	cpr = pow(2, config.bit_resolution);
	bit_resolution = config.bit_resolution;

	command_parity_bit = config.command_parity_bit; // for backwards compatibility
	command_rw_bit = config.command_rw_bit; // for backwards compatibility
	data_start_bit = config.data_start_bit; // for backwards compatibility

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

void simpleFOC::Sensor_init(){
	// initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
	getSensorAngle(); // call once

	vel_angle_prev = getSensorAngle(); // call again
	vel_angle_prev_ts = micros();
	HAL_Delay(1);
	getSensorAngle(); // call once

	angle_prev = getSensorAngle(); // call again
	angle_prev_ts = micros();
}

/**
 * Utility function used to calculate even parity of word
 */
uint8_t simpleFOC::spiCalcEvenParity(uint16_t value) {
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++) {
		if (value & 0x1)
			cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}

/*
 * Read a register from the SPI encoder sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
uint16_t simpleFOC::read(uint16_t angle_register) {
	uint16_t register_value;
	uint16_t command = angle_register;

	if (command_rw_bit > 0) {
		command = angle_register | (1 << command_rw_bit);
	}
	if (command_parity_bit > 0) {
		//Add a parity bit on the the MSB
		command |=
				((uint16_t) spiCalcEvenParity(command) << command_parity_bit);
	}

	//SPI - begin transaction

	//Send the command
	//  spi->transfer16(command);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &command,
			(uint8_t*) &register_value,
			sizeof(register_value) / sizeof(uint16_t), 100);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

//  delay_us(1);

	command = 0x0000;
	//Now read the response (NO_OPERATION_COMMAND = 0x0000)
	//  uint16_t register_value = spi->transfer16(0x00);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &command,
			(uint8_t*) &register_value,
			sizeof(register_value) / sizeof(uint16_t), 100);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

	//SPI - end transaction

	register_value = register_value >> (1 + data_start_bit - bit_resolution); //this should shift data to the rightmost bits of the word
	uint16_t data_mask = 0xFFFF >> (16 - bit_resolution);
	return register_value & data_mask; // Return the data, stripping the non data (e.g parity) bits
}

// function reading the raw counter of the magnetic sensor
int simpleFOC::getRawCount() {
	return (int) read(angle_register);
}

//  Shaft angle calculation
//  angle is in radians [rad]
float simpleFOC::getSensorAngle() {
	return (getRawCount() / (float) cpr) * _2PI;
}

// shaft angle calculation
float simpleFOC::shaftAngle() {
	LPF_angle_s.x = getAngle();
	LPF_angle_s = LowPassFilter(LPF_angle_s);
	return sensor_direction * LPF_angle_s.y_prev - sensor_offset;
}


float simpleFOC::getAngle() {
	return (float) full_rotations * _2PI + angle_prev;
}

float simpleFOC::getMechanicalAngle() {
	return angle_prev;
}

int32_t simpleFOC::getFullRotations() {
	return full_rotations;
}

// Electrical angle calculation
float simpleFOC::_electricalAngle(float shaft_angle, int pole_pairs) {
	return (shaft_angle * pole_pairs);
}

//Conversion shaft angle to elec angle
float simpleFOC::electricalAngle() {
	return _normalizeAngle(
			(float) (sensor_direction * pole_pairs) * getMechanicalAngle()
					- zero_electric_angle);
}

//normalizing radian angle to [0,2PI]
float simpleFOC::_normalizeAngle(float angle) {
	float a = fmod(angle, _2PI);
	return a >= 0 ? a : (a + _2PI);      //always project from 0 degree
}

void simpleFOC::updateSensor() {
	float val = getSensorAngle();
	angle_prev_ts = micros();
	float d_angle = val - angle_prev;
	// if overflow happened track it as full rotation
	if (abs(d_angle) > (0.8f * _2PI))
		full_rotations += (d_angle > 0) ? -1 : 1;
	angle_prev = val;

	getvelocity();
}

float simpleFOC::getvelocity() {
	// calculate sample time
	float Ts = (micros() - vel_angle_prev_ts) * 1e-6;
	// quick fix for strange cases (micros overflow)
	if (Ts <= 0)
		Ts = 1e-3f;
	// velocity calculation
	vel_prev = ((float) (full_rotations - vel_full_rotations) * _2PI
			+ (angle_prev - vel_angle_prev)) / Ts;
	// save variables for future pass
	vel_angle_prev = angle_prev;
	vel_angle_prev_ts = angle_prev_ts;
	vel_full_rotations = full_rotations;

	return vel_prev;
}

// shaft velocity calculation
float simpleFOC::shaftVelocity() {
	LPF_velocity_s.x = getvelocity();
	LPF_velocity_s = LowPassFilter(LPF_velocity_s);
	return sensor_direction * LPF_velocity_s.y_prev;
}

//Low-Pass Filter
struct LPF simpleFOC::LowPassFilter(struct LPF LPF) {
	unsigned long timestamp = micros();

	float dt = (timestamp - LPF.timestamp_prev) * 1e-6f;

	if (dt < 0.0f)
		dt = 1e-3f;
	else if (dt > 0.3f) {
		LPF.y_prev = LPF.x;
		LPF.timestamp_prev = timestamp;
		return LPF;
	}

	float alpha = LPF.Tf / (LPF.Tf + dt);
	float y = alpha * LPF.y_prev + (1.0f - alpha) * LPF.x;

	LPF.y_prev = y;
	LPF.timestamp_prev = timestamp;

	return LPF;
}



uint32_t simpleFOC::micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}
