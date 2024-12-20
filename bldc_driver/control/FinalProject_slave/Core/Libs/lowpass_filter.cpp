/*
 * lowpass_filter.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <lowpass_filter.h>
#include "stm32f1xx_hal.h"  // Include the HAL header for your specific MCU

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = micros();
}


float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}

/**
 * @brief Gather system clock and convert to microsecond
*/
uint32_t LowPassFilter::micros(void) 
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}