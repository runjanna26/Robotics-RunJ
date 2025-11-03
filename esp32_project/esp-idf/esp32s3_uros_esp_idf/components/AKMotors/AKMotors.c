#include "AKMotors.h"

// Motor configuration
motor_config_t AK60_6_V3_0 = {
    .P_MIN     = -12.56f,
    .P_MAX     =  12.56f,
    .V_MIN     = -60.0f,
    .V_MAX     =  60.0f,
    .T_MIN     = -12.0f,
    .T_MAX     =  12.0f,
    .Kp_MIN    =  0,
    .Kp_MAX    =  500.0f,
    .Kd_MIN    =  0,
    .Kd_MAX    =  5.0f,
    .Kt        =  0.135,
};

motor_config_t AK10_9_V3_0 = {
    .P_MIN     = -12.56f,
    .P_MAX     =  12.56f,
    .V_MIN     = -28.0f,
    .V_MAX     =  28.0f,
    .T_MIN     = -54.0f,
    .T_MAX     =  54.0f,
    .Kp_MIN    =  0,
    .Kp_MAX    =  500.0f,
    .Kd_MIN    =  0,
    .Kd_MAX    =  5.0f,
    .Kt        =  0.16,
};

motor_config_t AK70_9_V3_0 = {
    .P_MIN     = -12.56f,
    .P_MAX     =  12.56f,
    .V_MIN     = -30.0f,
    .V_MAX     =  30.0f,
    .T_MIN     = -32.0f,
    .T_MAX     =  32.0f,
    .Kp_MIN    =  0,
    .Kp_MAX    =  500.0f,
    .Kd_MIN    =  0,
    .Kd_MAX    =  5.0f,
    .Kt        =  0.16,
};

motor_config_t RMD_X4_10 = {
    .P_MIN     = -12.5f,
    .P_MAX     =  12.5f,
    .V_MIN     = -45.0f,
    .V_MAX     =  45.0f,
    .T_MIN     = -24.0f,
    .T_MAX     =  24.0f,
    .Kp_MIN    =  0,
    .Kp_MAX    =  500.0f,
    .Kd_MIN    =  0,
    .Kd_MAX    =  5.0f,
    .Kt        =  0.85,
};




esp_err_t CAN_Init()
{
    // CAN Bus Initialization
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_16, GPIO_NUM_17, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.rx_queue_len = 10; // Set RX queue length for DMA
    g_config.tx_queue_len = 10; // Set TX queue length for DMA

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK){
        printf("Driver installed\n");
    }else{
        printf("Failed to install driver\n");
        return 0;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK){
        printf("Driver started\n");
    }else{
        printf("Failed to start driver\n");
        return 0;
    }

    uint32_t alerts_to_enable = TWAI_ALERT_ALL; // Enable all alerts
    ESP_ERROR_CHECK(twai_reconfigure_alerts(alerts_to_enable, NULL));


    return ESP_OK;
}


// self.send_can_msg(0x280, [STOP_MOTOR_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]) 
void motor_all_stop()
{
    twai_message_t message;
    
    message.identifier = 0x280;
    message.extd = 0; // Extended Frame
    message.rtr = 0;  // Data Frame
    message.data_length_code = 8; 

    // can transmitt
    message.data[0] = 0x81;
    message.data[1] = 0x00;
    message.data[2] = 0x00;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;

    // Send the TWAI (CAN) message
    if (twai_transmit(&message, pdMS_TO_TICKS(1)) != ESP_OK){}
        // ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id); 
}

void motor_reboot(struct motor_struct *motor) 
{
    twai_message_t message;
    
    message.identifier = 0x140 + motor->id;
    message.extd = 0; // Extended Frame
    message.rtr = 0;  // Data Frame
    message.data_length_code = 8; 

    // can transmitt
    message.data[0] = 0x76;
    message.data[1] = 0x00;
    message.data[2] = 0x00;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;

    // Send the TWAI (CAN) message
    if (twai_transmit(&message, pdMS_TO_TICKS(1)) != ESP_OK){}
        // ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id); 
}


void motor_update(struct motor_struct *motor)
{
    request_motor_struct(motor->id, READ_MULTI_TURN_OUTPUT_SHAFT_ANGLE_ID);
    request_motor_struct(motor->id, READ_MOTOR_STATUS_1_ID);
    request_motor_struct(motor->id, READ_MOTOR_STATUS_2_ID);
}


void request_motor_struct(uint32_t motor_id, uint16_t request_id)
{
    twai_message_t message;
    
    message.identifier = SINGLE + motor_id; 
    message.extd = 0; // Extended Frame
    message.rtr = 0;  // Remote Frame
    message.data_length_code = 8; 

    // can transmitt
    message.data[0] = request_id;
    message.data[1] = 0x00;
    message.data[2] = 0x00;
    message.data[3] = 0x00;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;

    // Send the TWAI (CAN) message
    if (twai_transmit(&message, pdMS_TO_TICKS(1)) != ESP_OK){}
        // ESP_LOGE("CAN", "Failed to send feedback request"); 
}

void send_mit_force_command(struct motor_struct *motor, motor_config_t motor_config, 
    float p_des, float v_des, float kp, float kd, float t_ff)
{
    twai_message_t message;

    message.identifier = 0x400 + motor->id;
    message.extd = 0; // Extended Frame
    message.rtr = 0;  // Data Frame
    message.data_length_code = 8; // 4 bytes for speed data

    p_des   = fminf(fmaxf(motor_config.P_MIN,   p_des),     motor_config.P_MAX);
    v_des   = fminf(fmaxf(motor_config.V_MIN,   v_des),     motor_config.V_MAX);
    kp      = fminf(fmaxf(motor_config.Kp_MIN,  kp),        motor_config.Kp_MAX);
    kd      = fminf(fmaxf(motor_config.Kd_MIN,  kd),        motor_config.Kd_MAX);
    t_ff    = fminf(fmaxf(motor_config.T_MIN,   t_ff),      motor_config.T_MAX);

    motor->kp = kp;
    motor->kd = kd;
    motor->tff = t_ff;


    /// convert floats to unsigned ints ///
    uint16_t p_int   = float_to_uint(p_des, motor_config.P_MIN, motor_config.P_MAX, 16);
    uint16_t v_int   = float_to_uint(v_des, motor_config.V_MIN, motor_config.V_MAX, 12);
    uint16_t kp_int  = float_to_uint(kp, motor_config.Kp_MIN, motor_config.Kp_MAX, 12);
    uint16_t kd_int  = float_to_uint(kd, motor_config.Kd_MIN, motor_config.Kd_MAX, 12);
    uint16_t t_int   = float_to_uint(t_ff, motor_config.T_MIN, motor_config.T_MAX, 12);

    /// pack ints into the can buffer ///
    message.data[0] = (p_int >> 8) & 0xFF;
    message.data[1] = (p_int & 0xFF);
    message.data[2] = (v_int >> 4) & 0xFF;
    message.data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);
    message.data[4] = (kp_int & 0xFF);
    message.data[5] = ((kd_int >> 4) & 0xFF);
    message.data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    message.data[7] = (t_int & 0xFF);


    // Send the TWAI (CAN) message
    if (twai_transmit(&message, pdMS_TO_TICKS(1)) != ESP_OK)  
        ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor->id);
}


/*
0 indicating no fault
1 indicating motor over-temperaturefault
2 indicating over-current fault
3 indicating over-voltage fault
4 indicating under-voltagefault
5 indicating encoder fault
6 indicating MOSFET over-temperature fault
7 indicatingmotor lock-up.
*/
void unpack_reply(twai_message_t rx_message, struct motor_struct *motor,  motor_config_t motor_config)
{

    if (rx_message.identifier == RECEIVE_ID + motor->id)
    {
        if (rx_message.data[0] == READ_MULTI_TURN_OUTPUT_SHAFT_ANGLE_ID)
        {
            int32_t raw = (rx_message.data[7] << 24 | rx_message.data[6] << 16 | rx_message.data[5] << 8 | rx_message.data[4]);
            if (raw & 0x80000000)
                raw -= 0x100000000;
            motor->position = (float)(raw * (2.0 * _PI) / (262144.0));
        }
        if (rx_message.data[0] == READ_MOTOR_STATUS_2_ID)
        {
            uint8_t temperature             = rx_message.data[1];
            int32_t torque_current_motor    = (rx_message.data[2] | (rx_message.data[3] << 8));
            int32_t velocity                = (rx_message.data[4] | (rx_message.data[5] << 8));
            int32_t position                = (rx_message.data[6] | (rx_message.data[7] << 8));
            if (torque_current_motor & 0x8000)
                torque_current_motor -= 0x10000;
            if (velocity & 0x8000)
                velocity -= 0x10000;
            if (position & 0x8000)
                position -= 0x10000;

            motor->temperature   = (float)(temperature);                             // Celsius
            motor->current       = torque_current_motor * 0.01;                      // Amperes
            motor->torque        = torque_current_motor * 0.01 * motor_config.Kt;    // Nm
            motor->velocity      = velocity * _PI / 180.0;                           // rad/s
        }
        if (rx_message.data[0] == READ_MOTOR_STATUS_1_ID)
        {
            uint16_t voltage        = rx_message.data[4] | (rx_message.data[5] << 8);
            uint16_t error          = rx_message.data[6] | (rx_message.data[7] << 8);

            motor->voltage       = voltage * 0.1;    // Volts
            motor->error         = error;
        }
    }
}


// Static function
uint16_t float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (uint16_t) ((x- x_min)*((float)((1<<bits)/span)));
}
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) 
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}
void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) 
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}




// /**
//  * @brief Send the motor command with duty cycle in Servo mode.
//  * 
//  * @param motor_id : Motor driver identification
//  * @param duty_cycle : Desired duty cycle in the range of [0,1]
//  */
// void send_motor_dutycycle(uint32_t motor_id, float duty_cycle) 
// {
//     twai_message_t message;
    
//     message.identifier = (DutyCycleMode << 8) | motor_id;
//     message.extd = 1; // Extended Frame
//     message.rtr = 0;  // Data Frame
//     message.data_length_code = 4; // 4 bytes for speed data

//     // Convert velocity to byte array
//     int32_t send_index = 0;
//     uint8_t buffer[4];
//     buffer_append_int32(buffer, (int32_t)(duty_cycle * 100000.0), &send_index);
//     // can transmitt
//     message.data[0] = buffer[0];
//     message.data[1] = buffer[1];
//     message.data[2] = buffer[2];
//     message.data[3] = buffer[3];

//     // Send the TWAI (CAN) message
//     if (twai_transmit(&message, 0) != ESP_OK){}
//         // ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id); 
// }


// /**
//  * @brief Send the motor command with current in Servo mode.
//  * 
//  * @param motor_id : Motor driver identification
//  * @param current : Desired current range of -60 to 60 Amperes
//  */
// void send_motor_current(uint32_t motor_id, float current) 
// {
//     twai_message_t message;
    
//     message.identifier = (CurrentMode << 8) | motor_id;
//     message.extd = 1; // Extended Frame
//     message.rtr = 0;  // Data Frame
//     message.data_length_code = 4; // 4 bytes for speed data

//     // Convert velocity to byte array
//     int32_t send_index = 0;
//     uint8_t buffer[4];
//     buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
//     // can transmitt
//     message.data[0] = buffer[0];
//     message.data[1] = buffer[1];
//     message.data[2] = buffer[2];
//     message.data[3] = buffer[3];

//     // Send the TWAI (CAN) message
//     if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK)  ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id);

// }

// /**
//  * @brief Send the motor command with current for braking in Servo mode.
//  * 
//  * @param motor_id : Motor driver identification
//  * @param current_brake : Desired current brake range of 0 to 60 Amperes
//  */
// void send_motor_current_brake(uint32_t motor_id, float current_brake) 
// {
//     twai_message_t message;
    
//     message.identifier = (CurrentBrakeMode << 8) | motor_id;
//     message.extd = 1; // Extended Frame
//     message.rtr = 0;  // Data Frame
//     message.data_length_code = 4; // 4 bytes for speed data

//     // Convert velocity to byte array
//     int32_t send_index = 0;
//     uint8_t buffer[4];
//     buffer_append_int32(buffer, (int32_t)(current_brake * 1000.0), &send_index);    
//     // can transmitt
//     message.data[0] = buffer[0];
//     message.data[1] = buffer[1];
//     message.data[2] = buffer[2];
//     message.data[3] = buffer[3];

//     // Send the TWAI (CAN) message
//     if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK)  ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id);

// }

// /**
//  * @brief Send the motor command with velocity in Servo mode.
//  * 
//  * @param motor_id : Motor driver identification
//  * @param velocity_rpm : Desired velocity range from -100000 to 100000 electrical RPM.
//  */
// void send_motor_velocity(uint32_t motor_id, float velocity_rpm) 
// {
//     twai_message_t message;
    
//     message.identifier = (VelocityMode << 8) | motor_id;
//     message.extd = 1; // Extended Frame
//     message.rtr = 0;  // Data Frame
//     message.data_length_code = 4; // 4 bytes for speed data

//     int32_t send_index = 0;
//     uint8_t buffer[4];
//     buffer_append_int32(buffer, (int32_t)velocity_rpm, &send_index);

//     // can transmitt
//     message.data[0] = buffer[0];
//     message.data[1] = buffer[1];
//     message.data[2] = buffer[2];
//     message.data[3] = buffer[3];

//     // Send the TWAI (CAN) message
//     if (twai_transmit(&message, 0) != ESP_OK){}
//         // ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id); 

// }

// /**
//  * @brief Send the motor command with position in Servo mode.
//  * 
//  * @param motor_id : Motor driver identification
//  * @param position : Desired position range from -36000° to 36000°.
//  */
// void send_motor_position( uint32_t motor_id, float position) 
// {
//     twai_message_t message;
    
//     message.identifier = (PositionMode << 8) | motor_id;
//     message.extd = 1; // Extended Frame
//     message.rtr = 0;  // Data Frame
//     message.data_length_code = 4; // 4 bytes for position data

//     // Convert position to byte array
//     int32_t send_index = 0;
//     uint8_t buffer[4];
//     buffer_append_int32(buffer, (int32_t)(position * 10000.0), &send_index);
//     // can transmitt
//     message.data[0] = buffer[0];
//     message.data[1] = buffer[1];
//     message.data[2] = buffer[2];
//     message.data[3] = buffer[3];
//     // Send the TWAI (CAN) message
//     if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK)  ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id);

// }

// /**
//  * @brief Send the motor command with position, velocity and acceleration (motion control) in Servo mode.
//  * 
//  * @param motor_id : Motor driver identification.
//  * @param position : Desired position range from -36000° to 36000°.
//  * @param velocity_erpm : Desired velocity range from -327680 to 327680 electrical RPM.
//  * @param acceleration_erpmps2 : Desired acceleration range from 0 to 327670, with 1 unit equal to 10 electrical RPM/s²
//  */
// void send_motor_position_velocity(uint32_t motor_id, float position, int16_t velocity_erpm, int16_t acceleration_erpmps2) 
// {
//     twai_message_t message;
    
//     message.identifier = (PositionSpeedMode << 8) | motor_id;
//     message.extd = 1; // Extended Frame
//     message.rtr = 0;  // Data Frame
//     message.data_length_code = 8; // 4 bytes for position data

//     // Convert position to byte array
//     int32_t send_index = 0;
//     int16_t send_index1 = 4;
//     uint8_t buffer[8];
//     buffer_append_int32(buffer, (int32_t)(position * 10000.0), &send_index);
//     buffer_append_int16(buffer, velocity_erpm/10.0, & send_index1);
//     buffer_append_int16(buffer, acceleration_erpmps2/10.0, & send_index1);
//     // can transmitt
//     message.data[0] = buffer[0];
//     message.data[1] = buffer[1];
//     message.data[2] = buffer[2];
//     message.data[3] = buffer[3];
//     message.data[4] = buffer[4];
//     message.data[5] = buffer[5];
//     message.data[6] = buffer[6];
//     message.data[7] = buffer[7];
//     // Send the TWAI (CAN) message
//     if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK)  ESP_LOGE("CAN", "Motor ID: %ld is failed to send command", motor_id);

// }