import time
import numpy as np
import dynamixel_sdk as dxl
from scipy.interpolate import CubicSpline  # For smooth interpolation

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
MAX_VELOCITY = 100  # Increased for smoother movement
INTERPOLATION_STEPS = 5  # More steps for better smoothness

# ------------------- Gait Position Constants -------------------
INNER_LEG_EXTEND_POS = 3275
INNER_LEG_CONTRACT_POS = 2675
OUTER_LEG_SWING_FORWARD_POS = 2148
OUTER_LEG_SWING_BACKWARD_POS = 1948
INNER_LEG_SWING_FORWARD_POS = 1948
INNER_LEG_SWING_BACKWARD_POS = 2148

# ------------------- Motor Groups -------------------
INNER_LEG_EXTEND = [8, 10]
OUTER_LEG_SWING = [7, 11]  
ADDITIONAL_EXTEND_CONTRACT = [6, 12]
IDLE_MOTORS = [5, 9]  
ALL_MOTORS = INNER_LEG_EXTEND + OUTER_LEG_SWING + ADDITIONAL_EXTEND_CONTRACT + IDLE_MOTORS

# ------------------- Position Values -------------------
# ------------------- Gait Position Constants -------------------
HOME_POSITIONS = {5: 2048, 6: 2975, 7: 3072, 8: 2975, 9: 1536, 10: 2975, 11: 3072, 12: 2500}

GAIT_POSITIONS = {
    "step1": {6: 2200, 8: 2200, 10: 2200, 12: 3750},
    "step2": { 9: 1790},
    "step3": {9: 1536},
    "step4": {6: 2975, 8: 2500, 10: 2975, 12: 2975},
}

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

# ------------------- Cubic Interpolation -------------------
def interpolate_positions(start_pos, target_pos, steps=INTERPOLATION_STEPS):
    """
    Generates a smooth cubic trajectory from start to target position.

    :param start_pos: Initial motor position
    :param target_pos: Final motor position
    :param steps: Number of interpolation points
    :return: List of interpolated positions
    """
    x = [0, steps - 1]  # Start and end points
    y = [start_pos, target_pos]  # Corresponding positions
    spline = CubicSpline(x, y)  # Create cubic spline function

    return [int(spline(i)) for i in range(steps)]  # Generate interpolated steps

# ------------------- Move Motors with Interpolation -------------------
def move_motors_smoothly(target_positions):
    """
    Moves motors smoothly using cubic interpolation for better trajectory control.

    :param target_positions: Dictionary of motor IDs to target positions
    """
    groupSyncWriteVelocity = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
    groupSyncWritePosition = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Read current positions of motors
    motor_trajectories = {}
    max_steps = INTERPOLATION_STEPS

    for motor_id, target_pos in target_positions.items():
        current_pos = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION)[0]
        motor_trajectories[motor_id] = interpolate_positions(current_pos, target_pos, max_steps)

    # Execute smooth trajectory
    for step in range(max_steps):
        groupSyncWriteVelocity.clearParam()
        groupSyncWritePosition.clearParam()

        for motor_id, pos_list in motor_trajectories.items():
            if step < len(pos_list):
                desired_position = pos_list[step]

                # Set scaled velocity (proportional to movement)
                distance = abs(target_positions[motor_id] - pos_list[0])
                velocity = max(1, int((distance / max(distance, 1)) * MAX_VELOCITY))

                param_goal_velocity = [
                    dxl.DXL_LOBYTE(dxl.DXL_LOWORD(velocity)),
                    dxl.DXL_HIBYTE(dxl.DXL_LOWORD(velocity)),
                    dxl.DXL_LOBYTE(dxl.DXL_HIWORD(velocity)),
                    dxl.DXL_HIBYTE(dxl.DXL_HIWORD(velocity))
                ]
                groupSyncWriteVelocity.addParam(motor_id, param_goal_velocity)

                param_goal_position = [
                    dxl.DXL_LOBYTE(dxl.DXL_LOWORD(desired_position)),
                    dxl.DXL_HIBYTE(dxl.DXL_LOWORD(desired_position)),
                    dxl.DXL_LOBYTE(dxl.DXL_HIWORD(desired_position)),
                    dxl.DXL_HIBYTE(dxl.DXL_HIWORD(desired_position))
                ]
                groupSyncWritePosition.addParam(motor_id, param_goal_position)

        groupSyncWriteVelocity.txPacket()
        groupSyncWritePosition.txPacket()
        time.sleep(0.02)  # Small delay for smooth execution

# ------------------- Initialization -------------------
print("Sending all motors to initial positions...")
move_motors_smoothly(HOME_POSITIONS)
print("Initialization complete.")
input()
move_motors_smoothly(GAIT_POSITIONS["step1"])
move_motors_smoothly(GAIT_POSITIONS["step2"])
move_motors_smoothly(GAIT_POSITIONS["step3"])
move_motors_smoothly(GAIT_POSITIONS["step4"])


# ------------------- Close Port -------------------
portHandler.closePort()
print("Port closed. Execution complete.")