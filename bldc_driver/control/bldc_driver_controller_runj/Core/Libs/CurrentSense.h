/*
 * CurrentSense.h
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#ifndef CURRENTSENSE_H_
#define CURRENTSENSE_H_

#include "main.h"
#include "math.h"
#include <cstdint>  // for uint32_t
#include "foc_utils.h"
#include "motor_param.h"
#include "ekf.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern float phase_resistance;
extern float phase_inductance;
extern float CurrentSense_resistance;
extern float CurrentSense_gain;



class CurrentSense {
public:
	CurrentSense();
	virtual ~CurrentSense();

	//===Current Sensor function===//
	void initCurrentsense(float _shunt_resistor, float _gain);  	//
	void calibrateOffsets();									//
	struct PhaseCurrent_s getPhaseCurrents();					//checked
	struct DQCurrent_s getFOCCurrents(float angle_el);			//checked

	//===ADC DMA variable===
	uint32_t adcResultDMA_a[1], adcResultDMA_c[1];  // to store the ADC value

	float current_a_prev_EKF = 0;
	float current_b_prev_EKF = 0;
	float current_c_prev_EKF = 0;
private:
	//===Current Sensor===
	double offset_ia; //!< zero current A voltage value (center of the adc reading)
	double offset_ib; //!< zero current B voltage value (center of the adc reading)
	double offset_ic; //!< zero current C voltage value (center of the adc reading)
	float gain_a;     //!< phase A gain
	float gain_b;     //!< phase B gain
	float gain_c;     //!< phase C gain
	float R_sense;

	struct PhaseCurrent_s current;
	struct DQCurrent_s dq_current;



	// EKF	
	EKF ekf_current{3,3};
	ekf_t _ekf_s_current;

	static constexpr int EKF_N = 3;  // State vector dimension: [ia, ib, ic]
	static constexpr int EKF_M = 3;  // Measurement vector dimension: [ia, ib, ic]

	const float Pdiag[EKF_N] = {1, 1, 1};

	_float_t fx[EKF_N];

    _float_t x[EKF_N];          // State vector
    _float_t P[EKF_N * EKF_N];  // Prediction error covariance

    static constexpr _float_t Ts = 500e-6; // Sample time

    // Process noise covariance: Higher values indicate more uncertainty
    // Q00 [Position]: small because encoder is good. (1e-9)
    // Q11 [Velocity]: large because velocity calculation is bad. (1.0)
    const _float_t Q[EKF_N * EKF_N] = {1e-9, 0, 	0,
    								   0, 1e-9, 	0,
									   0,    0,  1e-9};
    // Measurement noise covariance
    // Lower values indicate higher confidence in measurements
    const _float_t R[EKF_M * EKF_M] = {1e-3, 0, 	0,
    								   0, 1e-6, 	0,
									   0,    0,  1e-3};
	// State transition model
	const _float_t F[EKF_N * EKF_N] = {	1 - ((CurrentSense_resistance + phase_resistance) * Ts / phase_inductance), 0, 0,
										0, 1 - ((CurrentSense_resistance + phase_resistance) * Ts / phase_inductance), 0,
										0, 0, 1 - ((CurrentSense_resistance + phase_resistance) * Ts / phase_inductance)};
	// Observation model: defines how measurements are mapped to the state space
	const _float_t H[EKF_M * EKF_N] = {	1, 0, 0,
										0, 1, 0,
										0, 0, 1};

									
// // Input model
// const _float_t B[EKF_N * EKF_N] = 
// {(Ts / L), 0, 0,
// 0, (Ts / L), 0,
// 0, 0, (Ts / L)};



};

#endif /* CURRENTSENSE_H_ */
