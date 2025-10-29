#include "OAIC.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
/**
 * @brief Initializes the MuscleModel structure.
 */
void MuscleModel_init(MuscleModel* model, double _a, double _b, double _beta) 
{
    model->a            = _a;
    model->b            = _b;
    model->beta         = _beta;

    model->pos_des      = 0.0;        
    model->pos_fb       = 0.0;
    
    model->vel_des      = 0.0;
    model->vel_fb       = 0.0;

    model->K            = 0.0;
    model->D            = 0.0;
    model->F            = 0.0;
    model->tau          = 0.0;
    
    model->timestamp_now_us  = esp_timer_get_time();
    model->timestamp_prev_us = model->timestamp_now_us;
}


double MuscleModel_gen_pos_error(MuscleModel* model) 
{
    return (model->pos_fb - model->pos_des);
}


double MuscleModel_gen_vel_error(MuscleModel* model) 
{
    return (model->vel_fb - model->vel_des);
}


double MuscleModel_gen_track_error(MuscleModel* model) 
{
    return (MuscleModel_gen_pos_error(model) + model->beta * MuscleModel_gen_vel_error(model));
}


double MuscleModel_gen_adapt_scalar(MuscleModel* model) 
{
    double track_error = MuscleModel_gen_track_error(model);
    return model->a / (1.0 + (model->b * (track_error*track_error))); 
}


void MuscleModel_calculate(MuscleModel* model, double pos_des, double vel_des, double pos_fb, double vel_fb) 
{
    double Ts;
    int64_t time_diff_us;

    model->pos_des = pos_des;
    model->vel_des = vel_des;

    model->pos_fb  = pos_fb;
    model->vel_fb  = vel_fb;

    model->F = MuscleModel_gen_track_error(model) / MuscleModel_gen_adapt_scalar(model);
    model->K = model->F * MuscleModel_gen_pos_error(model);
    model->D = model->F * MuscleModel_gen_vel_error(model);

    // model->F = 0.0f;
    // model->K = 20.0f;
    // model->D = 4.0f;

    model->tau = - model->K * MuscleModel_gen_pos_error(model) - model->D * MuscleModel_gen_vel_error(model) - model->F;

}



double MuscleModel_get_stiffness(MuscleModel* model) 
{
    return MuscleModel_limit_value(model->K, 0.0, 500.0);
}

double MuscleModel_get_damping(MuscleModel* model) 
{
    return MuscleModel_limit_value(model->D, 0.0, 5.0);
}

double MuscleModel_get_feedforward_force(MuscleModel* model) 
{
    return -MuscleModel_limit_value(model->F, -15.0, 15.0);
}

double MuscleModel_limit_value(double value, double min, double max) 
{
    if (value >= max) 
        return max;
    else if (value <= min) 
        return min;
    else 
        return value;
}
