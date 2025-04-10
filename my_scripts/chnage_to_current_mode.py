#!/usr/bin/env python3

import os
import sys
import time

# Try to import dynamixel_sdk after ensuring the serial package is installed
try:
    from dynamixel_sdk import *
except ModuleNotFoundError:
    print("Error: PySerial package is missing. Install it with:")
    print("    pip3 install pyserial")
    print("or")
    print("    sudo apt-get install python3-serial")
    sys.exit(1)

# Control table addresses for Dynamixel motors
# Note: These addresses are for X-series. Adjust based on your model
ADDR_OPERATING_MODE = 11       # Address for Operating Mode
ADDR_TORQUE_ENABLE = 64        # Address for Torque Enable

# Operating mode values
# Note: For most Dynamixel models, Current Control Mode is 0
# But for some models like X series, it's 0. Check your manual.
DXL_CURRENT_CONTROL_MODE = 0   # Value for Current Control Mode

# Protocol version
PROTOCOL_VERSION = 2.0         # Protocol version for Dynamixel motors (adjust if using v1.0)

# Default setting
BAUDRATE = 1000000             # Baudrate for Dynamixel communication
DEVICENAME = '/dev/ttyUSB0'    # Check your port name. Could be ttyUSB0, ttyACM0, etc.

def change_operating_mode_to_current():
    # Initialize PortHandler and PacketHandler
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    # Open the port
    try:
        if not portHandler.openPort():
            print("Failed to open the port!")
            return False
        
        # Set port baudrate
        if not portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the baudrate!")
            portHandler.closePort()
            return False
        
        # Loop through motor IDs 11 to 17
        for dxl_id in range(11, 18):
            print(f"Processing motor ID {dxl_id}...")
            
            # 1. Disable Torque to change the operating mode
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to disable torque for motor ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
                continue
            elif dxl_error != 0:
                print(f"Error disabling torque for motor ID {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
                continue
            
            # 2. Change Operating Mode to Current Control Mode
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, dxl_id, ADDR_OPERATING_MODE, DXL_CURRENT_CONTROL_MODE)
            
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to change operating mode for motor ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
                continue
            elif dxl_error != 0:
                print(f"Error changing operating mode for motor ID {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
                continue
            
            print(f"Successfully changed motor ID {dxl_id} to current control mode")
            
            # Small delay to ensure command is processed
            time.sleep(0.1)
        
        print("Completed processing all motors")
        # Close the port
        portHandler.closePort()
        return True
        
    except Exception as e:
        print(f"An error occurred: {e}")
        if portHandler.is_open:
            portHandler.closePort()
        return False

if __name__ == '__main__':
    print("Starting to change Dynamixel motors (ID 11-17) to current control mode...")
    result = change_operating_mode_to_current()
    if result:
        print("Successfully changed operating mode for all motors to current control mode")
    else:
        print("Failed to change operating mode for some or all motors")