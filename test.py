from pathlib import Path
import mujoco
import mujoco.viewer
import time
import numpy as np

# Model path based on your notebook
model_dir = Path("tars3d_mujocofiles")
model_xml = model_dir / "scene.xml"

# Load the model
model = mujoco.MjModel.from_xml_path(str(model_xml))
data = mujoco.MjData(model)


with mujoco.viewer.launch_passive(model, data) as viewer:
    print("MuJoCo viewer launched. Close the window to stop.")

    start = time.time()

    while viewer.is_running():
        t = time.time() - start

        # Set target position for each actuator
        for i in range(model.nu):
            data.ctrl[i] = 0.5 * np.sin(t + i)  # offset sine waves

        mujoco.mj_step(model, data)
        viewer.sync()