/*
 * pid.cpp
 *
 *  Created on: May 19, 2024
 *      Author: WINDOWS 11
 */

#include <pid.h>
#include "stm32g4xx_hal.h" // Include the HAL header for your specific MCU

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // output derivative limit [volts/second]
    , limit(limit)         // output supply limit     [volts]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros();
}

// PID controller "Functors" (see https://www.geeksforgeeks.org/functors-in-cpp/)
float PIDController::operator()(float error)
{
    // Calculate the time difference (Ts) from the last call
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;

    // Handle strange Ts values due to overflow or high-frequency issues
    if (Ts <= 0.0f || Ts > 0.5f) {
        // Reset Ts to a default reasonable value if it's negative or too large
        Ts = 1e-3f;
    }

    // Proportional part
    float proportional = P * error;

    // Integral part (Tustin transform)
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
    integral = _constrain(integral, -limit, limit);  // Anti-windup

    // Derivative part
    float derivative = D * (error - error_prev) / Ts;

    // Combine all components
    float output = proportional + integral + derivative;

    // Anti-windup - limit the output
    output = _constrain(output, -limit, limit);

    // If output ramp is defined, apply ramping
    if (output_ramp > 0)
    {
        // Limit the acceleration by ramping the output
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp * Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp * Ts;
    }

    // Save values for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;

    return output;
}


void PIDController::reset()
{
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
}

/**
 * @brief Gather system clock and convert to microsecond
*/
uint32_t PIDController::micros(void) 
{
    static uint32_t last_count = 0;
    static uint32_t overflow_count = 0;

    uint32_t current_count = DWT->CYCCNT;

    if (current_count < last_count) {
        // Overflow detected: increment overflow_count
        overflow_count++;
    }
    last_count = current_count;

    // Calculate total microseconds considering overflows
    uint32_t total_microseconds = (overflow_count * (UINT32_MAX / (SystemCoreClock / 1000000U))) +
                                  (current_count / (SystemCoreClock / 1000000U));

    return total_microseconds;
}

