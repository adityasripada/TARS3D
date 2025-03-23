import time
import dynamixel_sdk as dxl

# ------------------- Control Table Addresses -------------------
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_GOAL_VELOCITY = 104
LEN_GOAL_VELOCITY = 4

# ------------------- Settings -------------------
PROTOCOL_VERSION = 2.0
DEVICENAME = "COM7"  # Change as needed
BAUDRATE = 1000000

# ------------------- Motor Limits -------------------
MAX_VELOCITY = 1  # Dynamixel max velocity (full speed)

# ------------------- Gait Position Constants -------------------
HOME_POSITIONS = {5: 2048, 6: 2975, 7: 3072, 8: 2975, 9: 1536, 10: 2975, 11: 3072, 12: 2500}

GAIT_POSITIONS = {

    
    "step0": {6: 2975, 8: 2975, 9: 1536, 10: 2975, 12: 2500},
    "step1": {6: 2200, 8: 2200, 9: 1792, 10: 2200, 12: 3750},
    "step2": {6: 2975, 8: 2975, 9: 1536,  10: 2500, 12: 2975},

}

# ------------------- Motor Groups -------------------
ALL_MOTORS = list(HOME_POSITIONS.keys())

# ------------------- Initialize PortHandler & PacketHandler -------------------
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("Failed to open the port")
    exit()
print("Succeeded to open the port")

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    exit()
print(f"Baudrate set to {BAUDRATE}")

# ------------------- Enable Torque -------------------
for motor_id in ALL_MOTORS:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error:
        print(f"Error enabling torque on motor {motor_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
print("All motors torque enabled.")

# ------------------- Move Motors as Fast as Possible -------------------
def move_motors_bang_bang(target_positions):
    """
    Moves motors immediately to target positions with max speed.
    """
    groupSyncWriteVelocity = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
    groupSyncWritePosition = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Set max velocity for all motors
    for motor_id in target_positions.keys():
        param_goal_velocity = [
            dxl.DXL_LOBYTE(dxl.DXL_LOWORD(MAX_VELOCITY)),
            dxl.DXL_HIBYTE(dxl.DXL_LOWORD(MAX_VELOCITY)),
            dxl.DXL_LOBYTE(dxl.DXL_HIWORD(MAX_VELOCITY)),
            dxl.DXL_HIBYTE(dxl.DXL_HIWORD(MAX_VELOCITY))
        ]
        groupSyncWriteVelocity.addParam(motor_id, param_goal_velocity)

    groupSyncWriteVelocity.txPacket()  # Fire velocity settings

    # Move motors immediately to target positions
    for motor_id, target_pos in target_positions.items():
        param_goal_position = [
            dxl.DXL_LOBYTE(dxl.DXL_LOWORD(target_pos)),
            dxl.DXL_HIBYTE(dxl.DXL_LOWORD(target_pos)),
            dxl.DXL_LOBYTE(dxl.DXL_HIWORD(target_pos)),
            dxl.DXL_HIBYTE(dxl.DXL_HIWORD(target_pos))
        ]
        groupSyncWritePosition.addParam(motor_id, param_goal_position)

    groupSyncWritePosition.txPacket()  # Fire position commands
    time.sleep(0.05)  # Very short delay before next command

# ------------------- Execution -------------------
print("Sending all motors to initial positions...")
move_motors_bang_bang(HOME_POSITIONS)
print("Initialization complete.")
time.sleep(5)
print("Executing Step 1...")
move_motors_bang_bang(GAIT_POSITIONS["step1"])
time.sleep(0.5)

print("Executing Step 2...")
move_motors_bang_bang(GAIT_POSITIONS["step2"])

time.sleep(0.5)

print("Executing Step 2...")
move_motors_bang_bang(GAIT_POSITIONS["step3"])

# ------------------- Close Port -------------------
portHandler.closePort()
print("Port closed. Execution complete.")
