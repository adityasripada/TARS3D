import mujoco_py
from os.path import join, dirname

# Path to your XML file
model_path = "/home/asripada/TARS/robot.xml"  # Update this path

# Load the model from the XML file
model = mujoco_py.load_model_from_path(model_path)

# Create a MuJoCo simulation from the loaded model
sim = mujoco_py.MjSim(model)

# Create a viewer to visualize the simulation
viewer = mujoco_py.MjViewer(sim)

print("Use the viewer window to see the simulation. Press 'Esc' to exit.")

# Run the simulation and update the viewer
while True:
    sim.step()
    viewer.render()

    # Break the loop when the viewer window is closed (optional)
    if viewer.exit:
        break
