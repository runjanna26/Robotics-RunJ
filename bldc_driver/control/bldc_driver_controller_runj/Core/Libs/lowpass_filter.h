/*
 * lowpass_filter.h
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_
#include "math.h"
#include <cstdint>  // for uint32_t


class LowPassFilter
{
public:
    /**
     * @param Tf - Low pass filter time constant
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    float operator() (float x);         // Functors
    float Tf; //!< Low pass filter time constant
	static uint32_t micros(void);											//checked

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step
};


#endif /* LOWPASS_FILTER_H_ */
