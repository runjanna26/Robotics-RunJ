#include "driver/gpio.h"
#include "esp_log.h"

#include "driver/twai.h"
#include <math.h>

#ifndef AKMOTORS_H
#define AKMOTORS_H

#define _PI 3.14159265359

struct motor_struct {
    int id;
    // Feedback
    float position;
    float velocity;
    float current;
    float torque;
    float voltage;
    int temperature;
    uint16_t error;
    
    float kp;
    float kd;
    float tff;

    int64_t current_timestamp;
    uint8_t connection_status;
    uint8_t connection_error;
};

typedef struct {
    float P_MIN;
    float P_MAX;
    float V_MIN;
    float V_MAX;
    float T_MIN;
    float T_MAX;
    float Kp_MIN;
    float Kp_MAX;
    float Kd_MIN;
    float Kd_MAX;
    float Kt;
} motor_config_t;

// extern motor_config_t AK60_6_V3_0;
extern motor_config_t RMD_X4_10;

#define SINGLE  0x140
#define MULTI  0x240
#define RECEIVE_ID 0x240
#define READ_MULTI_TURN_OUTPUT_SHAFT_ANGLE_ID   0x60
#define READ_SINGLE_TURN_OUTPUT_SHAFT_ANGLE_ID  0x94
#define READ_MOTOR_STATUS_1_ID                  0x9A
#define READ_MOTOR_STATUS_2_ID                  0x9C
#define READ_MOTOR_STATUS_3_ID                  0x9D
#define READ_MOTOR_MODEL_ID                     0xB5
#define READ_ACCELATION_ID                      0x42



typedef enum servo_mode {
    DutyCycleMode           = 0,
    CurrentMode             = 1,
    CurrentBrakeMode        = 2,
    VelocityMode            = 3,
    PositionMode            = 4,
    SetOriginMode           = 5,
    PositionSpeedMode       = 6,
    MITControlMode          = 0x400
}servo_mode_t;


esp_err_t CAN_Init();
// Servo Mode
// void send_motor_dutycycle(uint32_t motor_id, float duty_cycle);
// void send_motor_current(uint32_t motor_id, float current);
// void send_motor_velocity(uint32_t motor_id, float velocity_rpm);
// void send_motor_position( uint32_t motor_id, float position);
// void send_motor_position_velocity(uint32_t motor_id, float position, int16_t velocity_erpm, int16_t acceleration_erpmps2);

void motor_reboot(struct motor_struct *motor);
void request_motor_struct(uint32_t motor_id, uint16_t request_id);
void motor_update(struct motor_struct *motor);

// MIT Mode
void send_mit_force_command(struct motor_struct *motor, motor_config_t motor_config, float p_des, float v_des, float kp, float kd, float t_ff);
void unpack_reply(twai_message_t rx_message, struct motor_struct *feedback,  motor_config_t motor_config);


uint16_t float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index);
#endif  // AKMOTORS_H


