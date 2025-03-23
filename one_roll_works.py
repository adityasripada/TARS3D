from pathlib import Path
import mujoco
import mujoco.viewer
import time
import numpy as np

# ----------------------------------------------------------------------
#  Load the MuJoCo model
# ----------------------------------------------------------------------
model_dir = Path("tars3d_mujocofiles")
model_xml = model_dir / "scene.xml"
model = mujoco.MjModel.from_xml_path(str(model_xml))
data = mujoco.MjData(model)

# ----------------------------------------------------------------------
#  Dynamixel-like Position Data
# ----------------------------------------------------------------------
HOME_POSITIONS = {
    # 5: 2048,  # (Only relevant if you have an actuator for motor 5)
    6: 2975,
    7: 3072,
    8: 2975,
    9: 1536,
    10: 2975,
    11: 3072,
    12: 2975
}

# HOME_POSITIONS = {5: 2048, 6: 2975, 7: 3072, 8: 2975, 9: 1536, 10: 2975, 11: 3072, 12: 2500}

GAIT_POSITIONS = {
    "step1": {6: 2975, 7: 3072, 8: 3750, 9: 1536, 10: 2975, 11: 3072, 12: 2975},
    "step2": {6: 2975, 7: 3072, 8: 2975, 9: 1536, 10: 2975, 11: 3072, 12: 3750},
    "step3": {6: 3750, 7: 3072, 8: 2975, 9: 1536, 10: 2975, 11: 3072, 12: 2975},
    "step4": {6: 2975, 7: 3072, 8: 2200, 9: 1536, 10: 2975, 11: 3072, 12: 2975},
    "step5": {6: 2975, 7: 3072, 8: 2975, 9: 1536, 10: 2975, 11: 3072, 12: 2975},


}


# GAIT_POSITIONS = [
#     {8: 3375, 10: 3375, 7: 2048, 11: 2048},
#     {8: 2575, 10: 2575, 7: 2048, 11: 2048},
#     {8: 3375, 10: 3375, 7: 2048, 11: 2048},
# ]

# ----------------------------------------------------------------------
#  Map Dynamixel IDs to MuJoCo Actuators & Convert Angles
# ----------------------------------------------------------------------
# Make sure the joint names in scene.xml exactly match these references
actuator_map = {
    7: 0,  # "7_leg_1_rot"
    9: 1,  # "9_leg2_rot"
    11: 2,  # "11_leg3_rot"
    12: 3,  # "12_leg4_pri"
    10: 4,  # "10_leg3_pri"
    8: 5,  # "8_leg2_pri"
    6: 6,  # "6_leg1_pri"
    # If you eventually add motor_id=5 as an actuator, put it here
}


def dxl_to_radians(dxl_pos):
    """
    Convert [0..4095] ticks into radians, with 2048 = 0 rad.
    """
    EXTRA_SCALE = 2
    return (dxl_pos - 2048) * (2.0 * np.pi / 4096.0)


def dxl_to_meters(dxl_pos, motor_id):
    """
    Convert Dynamixel positions to meters for prismatic joints.
    Each motor has a different mapping according to the maximum stroke.
    """
    if motor_id in [6, 10, 12]:
        # Stroke: -17mm to 16mm, Dynamixel positions from 2175 to 3775
        return (dxl_pos - 2975) * (33 / 1600.0) / 1000  # convert mm to meters
    elif motor_id == 8:
        # Stroke: -13.5mm to 19.5mm, Dynamixel positions from 2175 to 3775
        return 0.003 + ((dxl_pos - 2975) * (33 / 1600.0)) / 1000  # convert mm to meters


# Global storage for the last known positions of each actuator
last_positions = {i: 2048 for i in actuator_map}  # Initialize with mid-position for all


def set_mujoco_actuators_positions(position_dict):
    """
    Set MuJoCo's data.ctrl[...] for each motor_id in 'position_dict'.
    Ensure that all actuators are always given a position value, using the last known
    positions for those not explicitly mentioned in 'position_dict'.
    """
    global last_positions
    # Update last_positions with any new positions provided
    for motor_id, dxl_pos in position_dict.items():
        last_positions[motor_id] = dxl_pos

    # Apply the positions from last_positions to the actuators
    for motor_id, dxl_pos in last_positions.items():
        if motor_id in actuator_map:
            ctrl_index = actuator_map[motor_id]
            if motor_id in [6, 8, 10, 12]:  # Prismatic actuators
                data.ctrl[ctrl_index] = dxl_to_meters(dxl_pos, motor_id)
                print("motor", motor_id, ":", dxl_to_meters(dxl_pos, motor_id))
            else:  # Rotational actuators
                data.ctrl[ctrl_index] = dxl_to_radians(dxl_pos)


# Update the movement function to use global positions
def move_motors_interpolated(
    start_positions, end_positions, steps=4, sim_steps_per_step=50
):
    """
    Smoothly move motors from start_positions to end_positions in 'steps'
    increments. Each increment calls mj_step many times, and we update
    the viewer after each step to see motion in real time.
    """
    for step_i in range(1, steps + 1):
        alpha = step_i / float(steps)
        interpolated_positions = interpolate_positions(
            start_positions, end_positions, alpha
        )
        print("Interpolated positions for step", step_i, ":", interpolated_positions)
        set_mujoco_actuators_positions(interpolated_positions)

        # Advance the simulation enough steps so it visibly transitions
        for _ in range(sim_steps_per_step):
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)  # Sleep a tiny bit to avoid 100% CPU (optional)


# Then use this updated function in the rest of your code to handle the movements.


def interpolate_positions(start_dict, end_dict, alpha):
    """
    Linear interpolation between two {motor_id: dxl_position} dicts.
    """
    all_keys = set(start_dict.keys()) | set(end_dict.keys())
    result = {}
    for k in all_keys:
        start_val = start_dict.get(k, 2048)  # default mid if missing
        end_val = end_dict.get(k, 2048)
        result[k] = start_val + (end_val - start_val) * alpha
    return result


# ----------------------------------------------------------------------
#  Main Execution Loop with a Passive Viewer
# ----------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("MuJoCo viewer launched. Close the window to stop.")

    # Optionally move from "HOME" to "HOME" to initialize
    move_motors_interpolated(HOME_POSITIONS, HOME_POSITIONS)
    current_positions = HOME_POSITIONS.copy()
    time.sleep(2)

    while viewer.is_running():
        # move_motors_interpolated(HOME_POSITIONS, HOME_POSITIONS)

        # Cycle through GAIT_POSITIONS
        for gait_key in GAIT_POSITIONS:
            gait_positions = GAIT_POSITIONS[gait_key]
            move_motors_interpolated(
                current_positions, gait_positions, steps=2, sim_steps_per_step=85
            )
            # time.sleep(1)
            # input("Press Enter to continue...")

    current_positions = gait_positions.copy()

        # Repeat...
