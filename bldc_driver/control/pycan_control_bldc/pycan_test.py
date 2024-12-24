import can

import os
print(os.path.exists(r"C:\WINDOWS\system32\PCANBasic.dll"))

import platform
print(platform.architecture())

import ctypes
try:
    ctypes.windll.LoadLibrary(r"C:\WINDOWS\system32\PCANBasic.dll")
    print("PCANBasic.dll loaded successfully!")
except Exception as e:
    print(f"Failed to load PCANBasic.dll: {e}")

# PCAN Configuration (Example)
CAN_INTERFACE = 'pcan'  # Use the appropriate interface for your hardware
CHANNEL = 'PCAN_USBBUS1'  # Adjust channel name based on your device
BITRATE = 500000  # Set bitrate to match your network

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

    MESSAGE_ID = 0x123
    DATA = [0x01, 0x02, 0x03, 0x04]

    send_can_message(bus, MESSAGE_ID, DATA)

if __name__ == "__main__":
    main()
