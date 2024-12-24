import can

CAN_INTERFACE = 'socketcan'  # Use the appropriate interface for your hardware
CHANNEL = 'can0'  # Adjust channel name based on your device
BITRATE = 1000000  # Set bitrate to match your network
MIT_Params = {
        'AK80-9':{
            'P_min' : -12.5,
            'P_max' : 12.5,
            'V_min' : -50.0,
            'V_max' : 50.0,
            'T_min' : -18.0,
            'T_max' : 18.0,
            'Kp_min': 0.0,
            'Kp_max': 500.0,
            'Kd_min': 0.0,
            'Kd_max': 5.0,
            'Kt_TMotor' : 0.091, # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 0.59, # to correct the qaxis current 
            'Kt_actual': 0.115,# Need to use the right constant -- 0.115 by our calcs, 0.091 by theirs. At output leads to 1.31 by them and 1.42 by us.
            'GEAR_RATIO': 9.0, # hence the 9 in the name
            'Use_derived_torque_constants': True, # true if you have a better model
            'a_hat' : [0.0, 1.15605006e+00, 4.17389589e-04, 2.68556072e-01, 4.90424140e-02]
            #'a_hat' : [0.0,  8.23741648e-01, 4.57963164e-04,     2.96032614e-01, 9.31279510e-02]# [7.35415941e-02, 6.26896231e-01, 2.65240487e-04,     2.96032614e-01,  7.08736309e-02]# [-5.86860385e-02,6.50840079e-01,3.47461078e-04,8.58635580e-01,2.93809281e-01]
        }
}
def limit_value(value, min, max):
    """
    Limits value to be between min and max
    Args:
        value: The value to be limited.
        min: The lowest number allowed (inclusive) for value
        max: The highest number allowed (inclusive) for value
    """
    if value >= max:
        return max
    elif value <= min:
        return min
    else:
        return value

def float_to_uint(x,x_min,x_max,num_bits):
    """
    Interpolates a floating point number to an unsigned integer of num_bits length.
    A number of x_max will be the largest integer of num_bits, and x_min would be 0.
    args:
        x: The floating point number to convert
        x_min: The minimum value for the floating point number
        x_max: The maximum value for the floating point number
        num_bits: The number of bits for the unsigned integer
    """
    span = x_max-x_min
    bitratio = float((1<<num_bits)/span)
    x = limit_value(x,x_min,x_max-(2/bitratio))
    # (x - x_min)*(2^num_bits)/span
    
    return limit_value(int((x- x_min)*( bitratio )),0,int((x_max-x_min)*bitratio) )

def MIT_controller(bus, motor_id, motor_type, position, velocity, Kp, Kd, I):
    """
    Sends an MIT style control signal to the motor. This signal will be used to generate a 
    current for the field-oriented controller on the motor control chip, given by this expression:
        q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I
    Args:
        motor_id: The CAN ID of the motor to send the message to
        motor_type: A string noting the type of motor, ie 'AK80-9'
        position: The desired position in rad
        velocity: The desired velocity in rad/s
        Kp: The position gain
        Kd: The velocity gain
        I: The additional current
    """
    position_uint16 = float_to_uint(position, MIT_Params[motor_type]['P_min'], 
                                                MIT_Params[motor_type]['P_max'], 16)
    velocity_uint12 = float_to_uint(velocity, MIT_Params[motor_type]['V_min'], 
                                                MIT_Params[motor_type]['V_max'], 12)
    Kp_uint12 = float_to_uint(Kp, MIT_Params[motor_type]['Kp_min'], 
                                                MIT_Params[motor_type]['Kp_max'], 12)
    Kd_uint12 = float_to_uint(Kd, MIT_Params[motor_type]['Kd_min'], 
                                                MIT_Params[motor_type]['Kd_max'], 12)
    I_uint12 = float_to_uint(I, MIT_Params[motor_type]['T_min'], 
                                                MIT_Params[motor_type]['T_max'], 12)
    data = [
        position_uint16 >> 8,
        position_uint16 & 0x00FF,
        (velocity_uint12) >> 4,
        ((velocity_uint12&0x00F)<<4) | (Kp_uint12) >> 8,
        (Kp_uint12&0x0FF),
        (Kd_uint12) >> 4,
        ((Kd_uint12&0x00F)<<4) | (I_uint12) >> 8,
        (I_uint12&0x0FF)
    ]
    send_can_message(bus, motor_id, data)

def send_can_message(bus, message_id, data):
    """Send a CAN message."""
    msg = can.Message(
        arbitration_id=message_id,
        data=data,
        is_extended_id=False  # Set True for extended IDs (29-bit)
    )
    try:
        bus.send(msg)
        print(f"Message sent: ID=0x{message_id:X}, Data={data}")
    except can.CanError as e:
        print(f"Failed to send message: {e}")

def main():
    try:
        bus = can.interface.Bus(channel='PCAN_USBBUS1', interface='pcan', bitrate=500000)

    except Exception as e:
        print(f"Failed to connect to CAN bus: {e}")
        return

    # MESSAGE_ID = 0x123
    # DATA = [0x01, 0x02, 0x03, 0x04]

    # send_can_message(bus, MESSAGE_ID, DATA)
    MIT_controller(bus, 0x1, 'AK80-9', 3.14, 6.28, 10.0, 1.0, 0.0)

if __name__ == "__main__":
    main()
