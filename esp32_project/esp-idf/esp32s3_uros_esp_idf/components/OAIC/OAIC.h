#ifndef MUSCLE_MODEL_H
#define MUSCLE_MODEL_H


#include <math.h> // For fabs (absolute value)

// Define the structure equivalent to the Python class
typedef struct {
    double pos_init;
    
    double a;
    double b;
    double beta;
    double gamma; // Corresponds to self.γ
    
    double pos_des;
    double pos_fb;
    double pos_des_prev;
    
    double vel_des;
    double vel_fb;
    
    double K; // Stiffness
    double D; // Damping
    double F; // Feedforward force calculation intermediate
    double tau; // Output torque/force
    
    // Time tracking (using microseconds from esp_timer_get_time)
    int64_t timestamp_now_us;
    int64_t timestamp_prev_us;
    double Ts;
    
} MuscleModel;

// Function to initialize the MuscleModel structure
void MuscleModel_init(MuscleModel* model, double _a, double _b, double _beta);

// Core calculation function
void MuscleModel_calculate(MuscleModel* model, double pos_des, double vel_des, double pos_fb, double vel_fb);

// Error calculation functions (private/helper)
double MuscleModel_gen_pos_error(MuscleModel* model);
double MuscleModel_gen_vel_error(MuscleModel* model);
double MuscleModel_gen_track_error(MuscleModel* model);
double MuscleModel_gen_adapt_scalar(MuscleModel* model);

// Utility function (private/helper)
double MuscleModel_limit_value(double value, double min, double max);

// Getter functions
double MuscleModel_get_stiffness(MuscleModel* model);
double MuscleModel_get_damping(MuscleModel* model);
double MuscleModel_get_feedforward_force(MuscleModel* model);

#endif // MUSCLE_MODEL_H