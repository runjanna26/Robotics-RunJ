#ifndef CPG_RBF_H
#define CPG_RBF_H

#include <stdio.h>
#include <math.h>
#include "esp_log.h" // Include ESP-IDF logging header

/**
 * @brief CPG_SO2 Structure (Second-Order Central Pattern Generator)
 * * This structure holds all the state variables and parameters for the CPG model.
 */
typedef struct {
    // Initial and current output values for the CPG neurons
    float out0_t;
    float out1_t;
    
    // Initial and current phase difference between neurons
    float phi; // Renamed 'ϕ' to 'phi' for C standard
    
    // Scaling factor for feedback
    float alpha;
    
    // Connection weights between neurons (W matrix)
    float w00;
    float w01;
    float w10;
    float w11;
    
    // Current activation values (a_t) and next-step activation values (a_t1)
    float a0_t;
    float a1_t;
    float a0_t1;
    float a1_t1;
    
    // Output weights for the CPG neurons
    float output_cpg_weight;
    
    // Sensory input terms (s0 and s1)
    float s0;
    float s1;
} cpg_so2_t;

/**
 * @brief Initializes the CPG_SO2 structure with default or provided values.
 * * @param cpg_so2_ptr Pointer to the CPG structure to initialize.
 * @param o0_init Initial output value for neuron 0.
 * @param o1_init Initial output value for neuron 1.
 * @param phi_init Initial phase difference (ϕ) in radians.
 * @param alpha Scaling factor for feedback.
 */
void cpg_so2_init( cpg_so2_t *cpg_so2_ptr, float o0_init, float o1_init, float phi_init, float alpha);

/**
 * @brief Implementation of the CPG_SO2 update function.
 * @param phi_input     : Frequency of CPG outputs.
 * @param pause_input   : Pause input for the CPG {0,1}. Default is 0.0.
 * @param rewind_input  : Rewind input for the CPG {-1,1}. Default is 0.0.
 */
void cpg_so2_update(cpg_so2_t *cpg_rbf_ptr, float phi_input, float pause_input, float rewind_input); 

/**
 * @brief Utility function to set sensory input terms.
 * * @param cpg_so2_ptr Pointer to the CPG structure.
 * @param s0_val New value for sensory input s0.
 * @param s1_val New value for sensory input s1.
 */
void cpg_so2_set_sensory_input(cpg_so2_t *cpg_so2_ptr, float s0_val, float s1_val);

#endif // CPG_RBF_H