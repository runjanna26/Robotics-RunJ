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



extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

#define CurrentSense_resistance 0.01
#define CurrentSense_gain 5.0





class CurrentSense {
public:
	CurrentSense();
	virtual ~CurrentSense();

	//===Current Sensor function===//
	void initCurrentsense(float _shunt_resistor, float _gain);  	//
	void calibrateOffsets();									//
	struct PhaseCurrent_s getPhaseCurrents();					//checked
	struct DQCurrent_s getFOCCurrents(float angle_el);			//checked
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

	//===ADC DMA variable===
	uint32_t adcResultDMA_a[1], adcResultDMA_c[1];  // to store the ADC value
//	const int adcChannelCount = sizeof(adcResultDMA) / sizeof(adcResultDMA[0]);
//	volatile int adcConversionComplete = 0; // set by callback

};

#endif /* CURRENTSENSE_H_ */
