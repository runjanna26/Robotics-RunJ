import time
import mujoco
import mujoco.viewer
import numpy as np  # For sine wave calculations

# Load the model and data
model = mujoco.MjModel.from_xml_path("./one_joint.xml")
data = mujoco.MjData(model)

# Get the joint index
joint_name = "Revolute_joint"  # Replace with the actual joint name
joint_id = model.joint(name=joint_name).qposadr


with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()


        # # Apply control
        # # Set desired position (in radians)
        desired_position = 1.0 * np.sin(2 * np.pi * 0.5 * step_start)  # Example: sine wave
        data.ctrl[0] = desired_position

        # # Print the joint angle
        joint_angle = data.qpos[joint_id]
        # print(joint_angle)



        # Step the simulation
        mujoco.mj_step(model, data)

        # Example modification of a viewer option: toggle contact points every two seconds
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        # Sync the viewer with the simulation state
        viewer.sync()

        # Rudimentary time keeping to match the real-time simulation
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
