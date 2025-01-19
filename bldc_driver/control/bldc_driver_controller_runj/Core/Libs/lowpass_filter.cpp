#include <lowpass_filter.h>
#include "stm32g4xx_hal.h"  // Include the HAL header for your specific MCU

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = micros();
}

float LowPassFilter::operator()(float x)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev) * 1e-6f;

    // Handle timestamp wrap-around (CYCCNT overflow)
    if (timestamp < timestamp_prev) {
        // Adjust dt to account for wrap-around
        dt += (1ULL << 32) / (SystemCoreClock / 1000000U);  // Adding wrap-around duration in microseconds
    }

    // Ensure dt is within a reasonable range
    if (dt < 0.0f) {
        dt = 1e-3f;
    } else if (dt > 0.3f) {
        // Reset the filter if dt is too large, indicating a possible error or reset condition
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    // Low-pass filter calculation
    float alpha = Tf / (Tf + dt);
    float y = alpha * y_prev + (1.0f - alpha) * x;

    // Update previous output and timestamp
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
