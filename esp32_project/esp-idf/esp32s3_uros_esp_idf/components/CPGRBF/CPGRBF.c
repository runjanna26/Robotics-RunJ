#include "CPGRBF.h"
#include "esp_log.h"
static const char *CPG_RBF_TAG = "CPG_RBF";

/**
 * @brief Hyperbolic tangent function (tanh(x)).
 * * This is a common activation function for CPGs. We use the standard C library function.
 */
#define TANH(x) tanhf(x) // Use single-precision float version for embedded systems

/**
 * @brief Implementation of the CPG_SO2 initialization function.
 */
void cpg_so2_init( cpg_so2_t *cpg_rbf_ptr, float o0_init, float o1_init, float phi_init, float alpha) 
{
    if (cpg_rbf_ptr == NULL) 
    {
        ESP_LOGE(CPG_RBF_TAG, "CPG structure pointer is NULL.");
        return;
    }

    cpg_rbf_ptr->out0_t = o0_init;
    cpg_rbf_ptr->out1_t = o1_init;
    cpg_rbf_ptr->phi = phi_init;
    cpg_rbf_ptr->alpha = alpha;
    
    // Initialize Connection Weights (W matrix)
    cpg_rbf_ptr->w00 = cpg_rbf_ptr->alpha * cosf(cpg_rbf_ptr->phi);
    cpg_rbf_ptr->w01 = cpg_rbf_ptr->alpha * sinf(cpg_rbf_ptr->phi);
    cpg_rbf_ptr->w10 = cpg_rbf_ptr->alpha * (-sinf(cpg_rbf_ptr->phi));
    cpg_rbf_ptr->w11 = cpg_rbf_ptr->alpha * cosf(cpg_rbf_ptr->phi);
    
    // Initialize Activation Values
    cpg_rbf_ptr->a0_t = 0.0f;
    cpg_rbf_ptr->a1_t = 0.0f;
    cpg_rbf_ptr->a0_t1 = 0.0f;
    cpg_rbf_ptr->a1_t1 = 0.0f;
    
    // Initialize Output Weight and Sensory Inputs
    cpg_rbf_ptr->output_cpg_weight = 1.0f;
    cpg_rbf_ptr->s0 = 0.0f;
    cpg_rbf_ptr->s1 = 0.0f;

    ESP_LOGI(CPG_RBF_TAG, "CPG_SO2 initialized. Initial phi: %.4f, alpha: %.2f", cpg_rbf_ptr->phi, cpg_rbf_ptr->alpha);
}

/**
 * @brief Implementation of the CPG_SO2 update function.
 * @param phi_input     : Frequency of CPG outputs.
 * @param pause_input   : Pause input for the CPG {0,1}. Default is 0.0.
 * @param rewind_input  : Rewind input for the CPG {-1,1}. Default is 0.0.
 */
void cpg_so2_update(cpg_so2_t *cpg_rbf_ptr, float phi_input, float pause_input, float rewind_input) 
{
    if (cpg_rbf_ptr == NULL) 
    {
        ESP_LOGE(CPG_RBF_TAG, "CPG structure pointer is NULL.");
        return;
    }


        cpg_rbf_ptr->phi = phi_input;
        

        // rewind work
        // puase not work

        float phi_00 = (pause_input * M_PI/2.01f) + ((1.0f - pause_input) * rewind_input *  phi_input);
        float phi_01 =                              ((1.0f - pause_input) * rewind_input *  phi_input);
        float phi_10 =                              ((1.0f - pause_input) * rewind_input *  phi_input);
        float phi_11 = (pause_input * M_PI/2.01f) + ((1.0f - pause_input) * rewind_input *  phi_input);

        // float phi_00 = phi_input;
        // float phi_01 = phi_input;
        // float phi_10 = phi_input;
        // float phi_11 = phi_input;


        // Update the weights for the next cycle
        cpg_rbf_ptr->w00 = cpg_rbf_ptr->alpha * cosf(phi_00);
        cpg_rbf_ptr->w01 = cpg_rbf_ptr->alpha * sinf(phi_01);
        cpg_rbf_ptr->w10 = cpg_rbf_ptr->alpha * (-sinf(phi_10));
        cpg_rbf_ptr->w11 = cpg_rbf_ptr->alpha * cosf(phi_11);

        cpg_rbf_ptr->output_cpg_weight = (pause_input  / (cpg_rbf_ptr->alpha*cosf(M_PI/2.01f)))  + (1.0f - (float)(pause_input)) ; 
    

        ESP_LOGI(CPG_RBF_TAG, " %.3f, %.3f,  %.3f,  %.3f // %.3f", 
            phi_00,
            phi_01,
            phi_10,
            phi_11, 
            cpg_rbf_ptr->output_cpg_weight);

        // ESP_LOGI(CPG_RBF_TAG, " %.3f, %.3f 
        //     rewind_input, 
        //     cpg_rbf_ptr->output_cpg_weight);

    
    
    // Calculate the next activation values (a_t1)
    // [Neuron 0]: a0_t1 = w00*out0_t + w01*out1_t - s0 * cos(a0_t)
    cpg_rbf_ptr->a0_t1 = (cpg_rbf_ptr->w00 * cpg_rbf_ptr->out0_t) + (cpg_rbf_ptr->w01 * cpg_rbf_ptr->out1_t) - (cpg_rbf_ptr->s0 * cosf(cpg_rbf_ptr->a0_t));
                         
    // [Neuron 1]: a1_t1 = w10*out0_t + w11*out1_t - s1 * sin(a1_t)
    cpg_rbf_ptr->a1_t1 = (cpg_rbf_ptr->w10 * cpg_rbf_ptr->out0_t) + (cpg_rbf_ptr->w11 * cpg_rbf_ptr->out1_t) - (cpg_rbf_ptr->s1 * sinf(cpg_rbf_ptr->a1_t));

    // Calculate the next output values (out_t1)
    // Activation function: output_t1 = output_cpg_weight * tanh(a_t1)
    float out0_t1 = cpg_rbf_ptr->output_cpg_weight * TANH(cpg_rbf_ptr->a0_t1);
    float out1_t1 = cpg_rbf_ptr->output_cpg_weight * TANH(cpg_rbf_ptr->a1_t1);

    // Save outputs for the next iteration (t -> t-1)
    cpg_rbf_ptr->out0_t = out0_t1;
    cpg_rbf_ptr->out1_t = out1_t1;
    cpg_rbf_ptr->a0_t = cpg_rbf_ptr->a0_t1;
    cpg_rbf_ptr->a1_t = cpg_rbf_ptr->a1_t1;
}

/**
 * @brief Implementation of the utility function to set sensory input terms. (P'Joe's work)
 */
void cpg_so2_set_sensory_input(cpg_so2_t *cpg_rbf_ptr, float s0_val, float s1_val) 
{
    if (cpg_rbf_ptr == NULL) 
    {
        ESP_LOGE(CPG_RBF_TAG, "CPG structure pointer is NULL.");
        return;
    }
    cpg_rbf_ptr->s0 = s0_val;
    cpg_rbf_ptr->s1 = s1_val;
}

// class CPG_LOCO:
//     def __init__(self):
//         self.cpg = CPG_SO2()
    
//     def modulate_cpg(self, ϕ, α, β):
//         '''
//         Parameters:
//             ϕ  (float): Frequency of CPG outputs.
//             α  (float): Pause input for the CPG {0,1}. Default is 0.0.
//             β  (float): Rewind input for the CPG {-1,1}. Default is 0.0.
//         '''
        
//         # Adaptive Weights
//         ϕ00 = (α * np.pi/2.01) + ((1 - α) * β *  ϕ)
//         ϕ01 = (1 - α) * β *  ϕ
//         ϕ10 = (1 - α) * β *  ϕ
//         ϕ11 = (α * np.pi/2.01) + ((1 - α) * β *  ϕ)
        
//         self.cpg.w00 = self.cpg.alpha *  np.cos(ϕ00)
//         self.cpg.w01 = self.cpg.alpha *  np.sin(ϕ01)
//         self.cpg.w10 = self.cpg.alpha * -np.sin(ϕ10)
//         self.cpg.w11 = self.cpg.alpha *  np.cos(ϕ11)
        
//         self.cpg.output_cpg_weight = 

// (α  / (self.cpg.alpha * np.cos(np.pi/2.01))) + (1 - α) * (np.abs(β))
        

//         # Update CPG outputs
//         self.cpg.update_cpg(None)
            
//         return {'cpg_output_0': self.cpg.out0_t,
//                 'cpg_output_1': self.cpg.out1_t}